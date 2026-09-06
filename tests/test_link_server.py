"""
Tests for the control link: robot_state handlers directly (unit-level),
plus a real end-to-end test that starts link.server on a background
thread and talks to it over an actual TCP socket -- no GPIO involved, so
this runs anywhere (not just on the Raspberry Pi).

Run with:
    pytest
"""
import socket
import threading
import time

import pytest

from link.nmea import build_sentence, nmea_to_decimal, parse_sentence
from link.robot_state import CommandError, RobotState
from link.server import ControlServer


# --- RobotState unit tests -------------------------------------------------

def test_drive_updates_state_within_range():
    state = RobotState()
    state.drive(120, -120)
    assert state.left_pwm == 120
    assert state.right_pwm == -120


def test_drive_rejects_out_of_range_pwm():
    state = RobotState()
    with pytest.raises(CommandError) as exc_info:
        state.drive(300, 0)
    assert exc_info.value.code == "02"


def test_stop_zeroes_pwm_and_sets_idle():
    state = RobotState()
    state.drive(200, 200)
    state.stop()
    assert (state.left_pwm, state.right_pwm, state.mode) == (0, 0, "IDLE")


def test_set_mode_rejects_unknown_mode():
    state = RobotState()
    with pytest.raises(CommandError):
        state.set_mode("FLY")


def test_set_route_stores_points_and_arms_first_as_nav_target():
    state = RobotState()
    p1 = ("4723.492", "N", "00044.340", "W")
    p2 = ("4724.010", "N", "00044.500", "W")
    state.set_route(["2", *p1, *p2])
    assert state.route == [p1, p2]
    assert state.route_index == 0
    assert state.nav_target == p1


def test_set_route_rejects_field_count_mismatch():
    state = RobotState()
    with pytest.raises(CommandError) as exc_info:
        state.set_route(["2", "4723.492", "N", "00044.340", "W"])  # only 1 point, count says 2
    assert exc_info.value.code == "13"


def test_set_route_rejects_too_many_points():
    state = RobotState()
    with pytest.raises(CommandError) as exc_info:
        state.set_route(["201"] + ["4723.492", "N", "00044.340", "W"] * 201)
    assert exc_info.value.code == "13"


def test_set_route_rejects_bad_direction_letter():
    state = RobotState()
    with pytest.raises(CommandError):
        state.set_route(["1", "4723.492", "X", "00044.340", "W"])


def test_route_advances_to_next_waypoint_on_arrival():
    state = RobotState()
    p1 = ("4723.492", "N", "00044.340", "W")   # ~ 47.391533, -0.739
    p2 = ("4723.532", "N", "00044.292", "W")   # ~ 95m away
    state.set_route(["2", *p1, *p2])

    # Far from p1 -- must not advance.
    state.update_gps_fix(47.0, -1.0)
    assert state.route_index == 0

    # On top of p1 -- advances to p2.
    state.update_gps_fix(47.391533, -0.739)
    assert state.route_index == 1
    assert state.nav_target == p2

    # On top of p2 -- route completes; nav_target stays on the last point.
    state.update_gps_fix(47.392200, -0.738200)
    assert state.route_index == 2  # == len(route): done
    assert state.nav_target == p2


def test_nav_cancels_an_active_route():
    state = RobotState()
    state.set_route(["1", "4723.492", "N", "00044.340", "W"])
    state.set_nav_target("4724.010", "N", "00044.500", "W")
    assert state.route == []
    assert state.route_index == 0


def test_stop_cancels_an_active_route():
    state = RobotState()
    state.set_route(["1", "4723.492", "N", "00044.340", "W"])
    state.stop()
    assert state.route == []
    assert state.route_index == 0


def test_camera_command_rejects_unknown_action():
    state = RobotState()
    with pytest.raises(CommandError) as exc_info:
        state.camera_command("FLY")
    assert exc_info.value.code == "08"


def test_camera_command_rec_not_implemented_yet():
    # REC_START/REC_STOP: no video recording code exists in this project
    # yet -- only SNAP (below) actually does something now.
    state = RobotState()
    with pytest.raises(CommandError) as exc_info:
        state.camera_command("REC_START")
    assert "NOT_IMPLEMENTED" in exc_info.value.message


def test_camera_command_snap_fails_cleanly_when_camera_unreachable(monkeypatch):
    # No camera/stream_server.py running at this port in tests -- SNAP
    # must turn that into a clean CommandError, not a raw exception.
    monkeypatch.setenv("CAMERA_PORT", "1")  # nothing listens on port 1
    state = RobotState()
    with pytest.raises(CommandError) as exc_info:
        state.camera_command("SNAP")
    assert exc_info.value.code == "12"


def test_camera_command_snap_succeeds_against_a_real_snap_endpoint(monkeypatch):
    # Minimal stand-in for camera/stream_server.py's GET /snap: no OpenCV
    # dependency needed here, just something answering 200 on that path,
    # to check the HTTP round trip and ACK path for real.
    import http.server
    import json
    import threading

    class SnapHandler(http.server.BaseHTTPRequestHandler):
        def do_GET(self):
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(json.dumps({"ok": True, "file": "snap_test.jpg", "count": 1}).encode())

        def log_message(self, format, *args):
            pass

    mock_camera = http.server.HTTPServer(("127.0.0.1", 0), SnapHandler)
    thread = threading.Thread(target=mock_camera.serve_forever, daemon=True)
    thread.start()
    try:
        monkeypatch.setenv("CAMERA_HOST", "127.0.0.1")
        monkeypatch.setenv("CAMERA_PORT", str(mock_camera.server_address[1]))
        state = RobotState()
        state.camera_command("SNAP")  # must not raise
    finally:
        mock_camera.shutdown()
        mock_camera.server_close()


# --- End-to-end socket tests ------------------------------------------------

@pytest.fixture()
def running_server():
    # start_gps=False: tests don't need a real (or attempted) GPS fix, and
    # this avoids every test run trying to open a real serial device --
    # see test_control_server_starts_fine_without_gps_hardware below for a
    # dedicated check that start_gps=True doesn't crash when it can't.
    server = ControlServer("127.0.0.1", 0, start_gps=False)  # port 0 = pick a free port
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        yield server
    finally:
        server.shutdown()
        server.server_close()


def _send_and_receive(port, sentence):
    with socket.create_connection(("127.0.0.1", port), timeout=2) as sock:
        sock.sendall((sentence + "\r\n").encode("ascii"))
        response = sock.makefile("r").readline().strip()
    return response


def test_stp_over_real_socket_returns_ack(running_server):
    port = running_server.server_address[1]
    response = _send_and_receive(port, build_sentence("STP"))
    assert response == build_sentence("ACK", "STP")


def test_drv_out_of_range_over_real_socket_returns_err(running_server):
    port = running_server.server_address[1]
    response = _send_and_receive(port, build_sentence("DRV", 999, 0))
    sentence_type, fields = parse_sentence(response)
    assert sentence_type == "ERR"
    assert fields[0] == "02"


def test_unknown_sentence_type_returns_err(running_server):
    port = running_server.server_address[1]
    response = _send_and_receive(port, build_sentence("ZZZ"))
    sentence_type, fields = parse_sentence(response)
    assert sentence_type == "ERR"
    assert fields[0] == "11"


def test_bad_checksum_returns_err(running_server):
    port = running_server.server_address[1]
    response = _send_and_receive(port, "$PROV,STP*00")
    sentence_type, fields = parse_sentence(response)
    assert sentence_type == "ERR"
    assert fields[0] == "00"


def test_state_is_shared_across_connections(running_server):
    port = running_server.server_address[1]
    _send_and_receive(port, build_sentence("DRV", 80, 80))
    time.sleep(0.05)
    assert running_server.state.left_pwm == 80
    assert running_server.state.right_pwm == 80


def test_sta_without_nav_target_or_gps_fix_reports_placeholders(running_server):
    port = running_server.server_address[1]
    response = _send_and_receive(port, build_sentence("STA"))
    sentence_type, fields = parse_sentence(response)
    assert sentence_type == "STA"
    # lat, lat_dir, lon, lon_dir, cap, speed, left_pwm, right_pwm, battery,
    # mode, target_lat, target_lat_dir, target_lon, target_lon_dir
    assert fields[:4] == ["0.0", "N", "0.0", "E"]  # no real GPS fix yet
    assert fields[4:6] == ["0.0", "0.0"]  # cap, speed
    assert fields[10:] == ["0.0", "N", "0.0", "E"]  # no NAV received yet


def test_sta_after_nav_reports_that_target(running_server):
    port = running_server.server_address[1]
    _send_and_receive(port, build_sentence("NAV", 4807.038, "N", 1131.000, "E"))
    response = _send_and_receive(port, build_sentence("STA"))
    sentence_type, fields = parse_sentence(response)
    assert sentence_type == "STA"
    assert fields[10:] == ["4807.038", "N", "1131.0", "E"]


def test_sta_reports_a_real_gps_fix_once_one_arrives(running_server):
    # Simulates what link.gps_reader.GPSReader would do once a receiver is
    # attached and gets a fix -- this project's robot operates just west
    # of the meridian, so the negative longitude sign is the interesting
    # part to check (a hardcoded "assume East" bug shipped here once).
    running_server.state.update_gps_fix(47.391033, -0.738500, speed_kmh=3.7, cap=284.5)
    port = running_server.server_address[1]
    response = _send_and_receive(port, build_sentence("STA"))
    sentence_type, fields = parse_sentence(response)
    assert sentence_type == "STA"
    lat = nmea_to_decimal(fields[0], fields[1])
    lon = nmea_to_decimal(fields[2], fields[3])
    assert lat == pytest.approx(47.391033, abs=1e-4)
    assert lon == pytest.approx(-0.738500, abs=1e-4)
    assert fields[4] == "284.5"  # cap
    assert fields[5] == "3.7"    # speed_kmh


def test_rte_over_real_socket_returns_ack(running_server):
    port = running_server.server_address[1]
    response = _send_and_receive(
        port,
        build_sentence("RTE", "2", "4723.492", "N", "00044.340", "W", "4724.010", "N", "00044.500", "W"),
    )
    assert response == build_sentence("ACK", "RTE")


def test_rte_bad_count_over_real_socket_returns_err(running_server):
    port = running_server.server_address[1]
    response = _send_and_receive(port, build_sentence("RTE", "not-a-number"))
    sentence_type, fields = parse_sentence(response)
    assert sentence_type == "ERR"
    assert fields[0] == "13"


def test_sta_target_reflects_route_first_waypoint(running_server):
    port = running_server.server_address[1]
    _send_and_receive(
        port,
        build_sentence("RTE", "1", "4723.492", "N", "00044.340", "W"),
    )
    response = _send_and_receive(port, build_sentence("STA"))
    sentence_type, fields = parse_sentence(response)
    assert sentence_type == "STA"
    assert fields[10:] == ["4723.492", "N", "00044.340", "W"]


def test_control_server_starts_fine_without_gps_hardware(monkeypatch):
    # start_gps=True (the default), but pointed at a device path that
    # cannot possibly exist -- the point is confirming this never crashes
    # server startup, on this dev machine or on the Pi without a receiver
    # plugged in.
    monkeypatch.setenv("GPS_DEVICE", "/dev/definitely-not-a-real-device")
    server = ControlServer("127.0.0.1", 0)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        response = _send_and_receive(
            server.server_address[1], build_sentence("STA")
        )
        assert parse_sentence(response)[0] == "STA"
    finally:
        server.shutdown()
        server.server_close()
