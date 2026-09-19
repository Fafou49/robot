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
from unittest.mock import patch

import pytest

from link.nmea import build_sentence, nmea_to_decimal, parse_sentence
from link.robot_state import CommandError, RobotState
from link.server import SHUTDOWN_CMD, ControlServer


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


# --- RobotState <-> motor_driver wiring (2026-09-07) ------------------------
# RobotState itself stays hardware-free (motor_driver=None, the default,
# used by every other test in this file) -- these few tests specifically
# check the *wiring*: that drive()/stop()/set_mode() call into an injected
# motor_driver at exactly the right moments, using a fake standing in for
# motor_control.motor_driver.MotorDriver so no gpiod/hardware is involved.

class _FakeMotorDriver:
    def __init__(self):
        self.calls = []

    def drive(self, left, right):
        self.calls.append((left, right))


def test_drive_forwards_to_motor_driver():
    fake = _FakeMotorDriver()
    state = RobotState(motor_driver=fake)
    state.drive(120, -120)
    assert fake.calls == [(120, -120)]


def test_stop_forwards_zero_to_motor_driver():
    fake = _FakeMotorDriver()
    state = RobotState(motor_driver=fake)
    state.drive(200, 200)
    state.stop()
    assert fake.calls[-1] == (0, 0)


def test_set_mode_away_from_manual_zeroes_motor_driver():
    fake = _FakeMotorDriver()
    state = RobotState(motor_driver=fake)
    state.set_mode("MANUAL")
    state.drive(100, 100)
    fake.calls.clear()
    state.set_mode("AUTO")  # leaving MANUAL -- must stop the motors
    assert fake.calls == [(0, 0)]


def test_set_mode_staying_manual_does_not_touch_motor_driver():
    fake = _FakeMotorDriver()
    state = RobotState(motor_driver=fake)
    state.set_mode("MANUAL")
    assert fake.calls == []  # already zero, no driver call needed


def test_bare_robot_state_has_no_motor_driver_by_default():
    # Every other test in this file constructs RobotState() with no
    # motor_driver -- confirms that stays fully inert (None), matching
    # this module's docstring.
    state = RobotState()
    assert state.motor_driver is None
    state.drive(50, 50)  # must not raise just because there's no driver


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


# --- AUTO-mode autonomous driving (2026-09-07) ------------------------------
# update_gps_fix() now actually drives the motors while mode == "AUTO" (see
# link/autopilot.py) -- these use a _FakeMotorDriver (defined above) to
# check the wiring without any real PID tuning/GPS math assertions (that
# belongs in tests/test_autopilot.py).

def test_auto_mode_drives_toward_nav_target_on_gps_fix():
    fake = _FakeMotorDriver()
    state = RobotState(motor_driver=fake)
    state.set_nav_target("4723.532", "N", "00044.292", "W")  # far from the fix below
    state.set_mode("AUTO")
    fake.calls.clear()  # drop the zero-pwm call set_mode("AUTO") itself makes

    state.update_gps_fix(47.0, -1.0)  # far from the target

    assert len(fake.calls) == 1
    left, right = fake.calls[0]
    assert (left, right) == (state.left_pwm, state.right_pwm)
    assert left != 0 or right != 0  # far away -- must actually be driving


def test_manual_mode_does_not_autonomously_drive_on_gps_fix():
    fake = _FakeMotorDriver()
    state = RobotState(motor_driver=fake)
    state.set_nav_target("4723.532", "N", "00044.292", "W")
    state.set_mode("MANUAL")
    fake.calls.clear()

    state.update_gps_fix(47.0, -1.0)

    assert fake.calls == []  # not in AUTO -- update_gps_fix must not drive


def test_auto_mode_with_no_target_does_not_drive():
    fake = _FakeMotorDriver()
    state = RobotState(motor_driver=fake)
    state.set_mode("AUTO")
    fake.calls.clear()

    state.update_gps_fix(47.0, -1.0)  # AUTO armed, but neither NAV nor RTE ever sent

    assert fake.calls == []


def test_auto_mode_stops_once_arrived_at_a_lone_nav_target():
    fake = _FakeMotorDriver()
    state = RobotState(motor_driver=fake)
    state.set_nav_target("4723.492", "N", "00044.340", "W")  # ~47.391533, -0.739
    state.set_mode("AUTO")
    fake.calls.clear()

    state.update_gps_fix(47.391533, -0.739)  # right on top of the target

    assert fake.calls[-1] == (0, 0)
    assert (state.left_pwm, state.right_pwm) == (0, 0)


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


# --- has_nav_target (2026-09-12, gates the gamepad's BTN_Y) -----------------

def test_has_nav_target_false_by_default():
    state = RobotState()
    assert state.has_nav_target() is False


def test_has_nav_target_true_after_nav():
    state = RobotState()
    state.set_nav_target("4723.492", "N", "00044.340", "W")
    assert state.has_nav_target() is True


def test_has_nav_target_true_after_route():
    state = RobotState()
    state.set_route(["1", "4723.492", "N", "00044.340", "W"])
    assert state.has_nav_target() is True


def test_has_nav_target_false_again_after_stop():
    # stop() (STP / BTN_B / BTN_START) cancels the route but leaves the
    # last nav_target in place (same as today's STA behavior) -- so
    # has_nav_target() stays True, matching "there's still something to
    # send BTN_Y back to" rather than requiring a fresh NAV/RTE after
    # every stop.
    state = RobotState()
    state.set_nav_target("4723.492", "N", "00044.340", "W")
    state.stop()
    assert state.has_nav_target() is True


# --- is_manual (2026-09-12, fixes the gamepad drive handler stomping on
#     AUTO's own PWM output -- see link/gamepad_handler.py) --------------

def test_is_manual_false_by_default():
    state = RobotState()
    assert state.is_manual() is False


def test_is_manual_true_after_drive():
    # DRV / a genuine gamepad stick push switches to MANUAL.
    state = RobotState()
    state.drive(100, 100)
    state.set_mode("MANUAL")
    assert state.is_manual() is True


def test_is_manual_false_once_auto_is_armed():
    state = RobotState()
    state.set_nav_target("4723.492", "N", "00044.340", "W")
    state.set_mode("AUTO")
    assert state.is_manual() is False


# --- PID: live gain update (2026-09-07 -- now really tunes link.autopilot) --

def test_set_pid_gains_rejects_unknown_loop():
    state = RobotState()
    with pytest.raises(CommandError) as exc_info:
        state.set_pid_gains("Z", 1.0, 0.0, 0.0)
    assert exc_info.value.code == "06"


def test_set_pid_gains_rejects_non_numeric_value():
    state = RobotState()
    with pytest.raises(CommandError) as exc_info:
        state.set_pid_gains("D", "oops", 0.0, 0.0)
    assert exc_info.value.code == "07"


def test_set_pid_gains_updates_the_live_autopilot():
    state = RobotState()
    state.set_pid_gains("D", 2.0, 0.1, 0.3)
    state.set_pid_gains("A", 0.7, 0.0, 0.9)
    assert state.pid_gains["D"] == (2.0, 0.1, 0.3)
    assert state.pid_gains["A"] == (0.7, 0.0, 0.9)
    assert (state.autopilot._pid_distance.kp, state.autopilot._pid_distance.ki,
            state.autopilot._pid_distance.kd) == (2.0, 0.1, 0.3)
    assert (state.autopilot._pid_angle.kp, state.autopilot._pid_angle.ki,
            state.autopilot._pid_angle.kd) == (0.7, 0.0, 0.9)


def test_camera_command_rejects_unknown_action():
    state = RobotState()
    with pytest.raises(CommandError) as exc_info:
        state.camera_command("FLY")
    assert exc_info.value.code == "08"


def test_camera_command_rec_start_fails_cleanly_when_camera_unreachable(monkeypatch):
    # 2026-09-18: REC_START/REC_STOP are genuinely implemented now (see
    # camera/recordings.py's VideoRecorder) -- no camera/stream_server.py
    # running at this port in tests, so this must turn into a clean
    # CommandError, not a raw exception, same as SNAP already does.
    monkeypatch.setenv("CAMERA_PORT", "1")  # nothing listens on port 1
    state = RobotState()
    with pytest.raises(CommandError) as exc_info:
        state.camera_command("REC_START")
    assert exc_info.value.code == "12"
    assert state.is_recording is False  # never flips True on a failed call


def test_camera_command_rec_start_then_stop_against_a_real_endpoint(monkeypatch):
    # Minimal stand-in for camera/stream_server.py's /rec/start and
    # /rec/stop -- same spirit as the SNAP round-trip test below, just
    # checking the REC_START/REC_STOP HTTP calls and is_recording
    # bookkeeping, not camera/recordings.py's actual VideoWriter usage
    # (that's exercised in tests/test_stream_server.py and
    # tests/test_recordings.py instead).
    import http.server
    import json
    import threading

    class RecHandler(http.server.BaseHTTPRequestHandler):
        def do_GET(self):
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            if self.path == "/rec/start":
                self.wfile.write(json.dumps({"ok": True, "recording": True}).encode())
            else:
                self.wfile.write(json.dumps({"ok": True, "recording": False, "file": "rec_test.mp4"}).encode())

        def log_message(self, format, *args):
            pass

    mock_camera = http.server.HTTPServer(("127.0.0.1", 0), RecHandler)
    thread = threading.Thread(target=mock_camera.serve_forever, daemon=True)
    thread.start()
    try:
        monkeypatch.setenv("CAMERA_HOST", "127.0.0.1")
        monkeypatch.setenv("CAMERA_PORT", str(mock_camera.server_address[1]))
        state = RobotState()
        assert state.is_recording is False

        state.camera_command("REC_START")
        assert state.is_recording is True

        state.camera_command("REC_STOP")
        assert state.is_recording is False
    finally:
        mock_camera.shutdown()
        mock_camera.server_close()


def test_camera_command_rec_start_while_already_recording_is_a_no_op(monkeypatch):
    # A direct CAM,REC_START from the website console while a
    # gamepad-started recording is already running must not make a
    # second, pointless HTTP call -- monkeypatching urlopen to raise lets
    # this test prove it's never even attempted.
    state = RobotState()
    state.is_recording = True

    def _fail_if_called(*args, **kwargs):
        raise AssertionError("must not call the camera when already recording")

    monkeypatch.setattr("link.robot_state.urllib.request.urlopen", _fail_if_called)
    state.camera_command("REC_START")  # must not raise, must not call urlopen
    assert state.is_recording is True


def test_camera_command_rec_stop_while_not_recording_is_a_no_op(monkeypatch):
    state = RobotState()

    def _fail_if_called(*args, **kwargs):
        raise AssertionError("must not call the camera when not recording")

    monkeypatch.setattr("link.robot_state.urllib.request.urlopen", _fail_if_called)
    state.camera_command("REC_STOP")  # must not raise, must not call urlopen
    assert state.is_recording is False


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


# --- save_waypoint (gamepad's BTN_X, 2026-09-18) ----------------------------

def test_save_waypoint_raises_with_no_gps_fix_yet():
    state = RobotState()
    with pytest.raises(CommandError) as exc_info:
        state.save_waypoint()
    assert exc_info.value.code == "14"


def test_save_waypoint_appends_a_lat_lon_timestamp_line(tmp_path, monkeypatch):
    waypoints_file = tmp_path / "subdir" / "waypoints.txt"
    monkeypatch.setenv("WAYPOINTS_FILE", str(waypoints_file))

    state = RobotState()
    state.update_gps_fix(47.391534, -0.739006)
    path = state.save_waypoint()

    assert path == str(waypoints_file)
    assert waypoints_file.exists()  # os.makedirs() created the parent dir too
    line = waypoints_file.read_text().strip()
    lat_str, lon_str, timestamp = line.split(",")
    assert float(lat_str) == pytest.approx(47.391534, abs=1e-6)
    assert float(lon_str) == pytest.approx(-0.739006, abs=1e-6)
    assert timestamp  # non-empty -- exact format isn't this test's concern


def test_save_waypoint_is_compatible_with_the_gps_route_upload_format(tmp_path, monkeypatch):
    # robot-webserver's own "GPS Driving" file upload parses one
    # "lat,lon" per line in decimal degrees, ignoring any extra columns
    # -- this is what makes a waypoint saved here re-uploadable there
    # with zero conversion (see save_waypoint()'s docstring).
    waypoints_file = tmp_path / "waypoints.txt"
    monkeypatch.setenv("WAYPOINTS_FILE", str(waypoints_file))

    state = RobotState()
    state.update_gps_fix(47.391534, -0.739006)
    state.save_waypoint()
    state.update_gps_fix(48.117300, 11.516667)
    state.save_waypoint()

    def parse_gps_route_file(text):
        # Same logic as robot-webserver's parseGpsRouteFile (JS) -- see
        # app.py in that repo -- reimplemented here in Python just to
        # prove this file parses the same way, not to duplicate that
        # project's own tests.
        points = []
        for raw_line in text.splitlines():
            line = raw_line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split(",")
            if len(parts) < 2:
                continue
            points.append((float(parts[0]), float(parts[1])))
        return points

    points = parse_gps_route_file(waypoints_file.read_text())
    assert points == [
        pytest.approx((47.391534, -0.739006), abs=1e-6),
        pytest.approx((48.117300, 11.516667), abs=1e-6),
    ]


def test_save_waypoint_appends_rather_than_overwrites(tmp_path, monkeypatch):
    waypoints_file = tmp_path / "waypoints.txt"
    monkeypatch.setenv("WAYPOINTS_FILE", str(waypoints_file))

    state = RobotState()
    state.update_gps_fix(1.0, 1.0)
    state.save_waypoint()
    state.update_gps_fix(2.0, 2.0)
    state.save_waypoint()

    assert len(waypoints_file.read_text().strip().splitlines()) == 2


# --- list_waypoints() / get_route() (2026-09-19, for robot-webserver's map) -

def test_list_waypoints_is_empty_when_the_file_does_not_exist_yet(tmp_path, monkeypatch):
    monkeypatch.setenv("WAYPOINTS_FILE", str(tmp_path / "never_written.txt"))
    state = RobotState()
    assert state.list_waypoints() == []


def test_list_waypoints_parses_points_saved_via_the_gamepad(tmp_path, monkeypatch):
    waypoints_file = tmp_path / "waypoints.txt"
    monkeypatch.setenv("WAYPOINTS_FILE", str(waypoints_file))

    state = RobotState()
    state.update_gps_fix(47.391534, -0.739006)
    state.save_waypoint()
    state.update_gps_fix(48.117300, 11.516667)
    state.save_waypoint()

    assert state.list_waypoints() == [
        pytest.approx((47.391534, -0.739006), abs=1e-6),
        pytest.approx((48.117300, 11.516667), abs=1e-6),
    ]


def test_list_waypoints_skips_blank_comment_and_malformed_lines(tmp_path, monkeypatch):
    # Same tolerant parsing as robot-webserver's own GPS-route-file upload
    # -- this file is hand-editable, a stray line shouldn't break WPT.
    waypoints_file = tmp_path / "waypoints.txt"
    waypoints_file.write_text(
        "47.391534,-0.739006,2026-09-19T10:00:00\n"
        "\n"
        "# a comment a human added by hand\n"
        "not,a,number\n"
        "only_one_field\n"
        "48.117300,11.516667,2026-09-19T10:01:00\n"
    )
    monkeypatch.setenv("WAYPOINTS_FILE", str(waypoints_file))

    state = RobotState()
    assert state.list_waypoints() == [
        pytest.approx((47.391534, -0.739006), abs=1e-6),
        pytest.approx((48.117300, 11.516667), abs=1e-6),
    ]


def test_get_route_is_empty_by_default():
    state = RobotState()
    assert state.get_route() == []


def test_get_route_reflects_the_active_route_after_rte():
    state = RobotState()
    state.set_route(["2", "4807.038", "N", "1131.000", "E", "4823.192", "N", "1152.500", "E"])
    assert state.get_route() == [
        ("4807.038", "N", "1131.000", "E"),
        ("4823.192", "N", "1152.500", "E"),
    ]


def test_get_route_returns_a_copy_not_the_live_list():
    # A caller mutating (or just holding onto) the returned list must
    # never see a later RTE/NAV/STP change out from under it -- see
    # get_route()'s own docstring.
    state = RobotState()
    state.set_route(["1", "4807.038", "N", "1131.000", "E"])
    snapshot = state.get_route()
    state.stop()  # clears state.route via set_mode(), see stop()
    assert snapshot == [("4807.038", "N", "1131.000", "E")]


# --- End-to-end socket tests ------------------------------------------------

@pytest.fixture()
def running_server():
    # start_gps/start_motor/start_gamepad=False: tests don't need a real
    # (or attempted) GPS fix, GPIO chip, or gamepad, and this avoids every
    # test run making real subprocess/device-scan calls (gpiodetect,
    # evdev.list_devices) -- see
    # test_control_server_starts_fine_without_gps_hardware below for a
    # dedicated check that leaving all three at their True default doesn't
    # crash when the hardware behind them isn't there.
    server = ControlServer(
        "127.0.0.1", 0,  # port 0 = pick a free port
        start_gps=False, start_motor=False, start_gamepad=False,
    )
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


def test_drv_asserts_manual_mode(running_server):
    # A manual DRV always takes back control -- same "manual override
    # always wins" convention NAV/STP already use for the route (see
    # link/robot_state.py's set_mode()/docstring). Confirmed via a real
    # AUTO->DRV transition, not just a bare mode check.
    running_server.state.set_mode("AUTO")
    port = running_server.server_address[1]
    _send_and_receive(port, build_sentence("DRV", 80, 80))
    time.sleep(0.05)
    assert running_server.state.mode == "MANUAL"
    assert running_server.state.left_pwm == 80


def test_sta_without_nav_target_or_gps_fix_reports_placeholders(running_server):
    port = running_server.server_address[1]
    response = _send_and_receive(port, build_sentence("STA"))
    sentence_type, fields = parse_sentence(response)
    assert sentence_type == "STA"
    # lat, lat_dir, lon, lon_dir, cap, speed, left_pwm, right_pwm, battery,
    # mode, target_lat, target_lat_dir, target_lon, target_lon_dir, dgps
    assert fields[:4] == ["0.0", "N", "0.0", "E"]  # no real GPS fix yet
    assert fields[4:6] == ["0.0", "0.0"]  # cap, speed
    assert fields[10:14] == ["0.0", "N", "0.0", "E"]  # no NAV received yet
    assert fields[14] == "UNKNOWN"  # no GGA quality seen yet


def test_sta_after_nav_reports_that_target(running_server):
    port = running_server.server_address[1]
    _send_and_receive(port, build_sentence("NAV", 4807.038, "N", 1131.000, "E"))
    response = _send_and_receive(port, build_sentence("STA"))
    sentence_type, fields = parse_sentence(response)
    assert sentence_type == "STA"
    assert fields[10:14] == ["4807.038", "N", "1131.0", "E"]


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


# --- STA: DGPS field (2026-09-19) -------------------------------------------

def test_sta_dgps_field_is_unknown_before_any_gga_quality_seen(running_server):
    # update_gps_fix() without is_dgps= (e.g. an RMC-only fix, or a test
    # like the ones above that doesn't pass it) must not silently claim
    # "GPS" -- "no quality info yet" and "not a DGPS fix" are different
    # states, see RobotState.is_dgps's own docstring.
    running_server.state.update_gps_fix(47.391033, -0.738500, speed_kmh=3.7, cap=284.5)
    port = running_server.server_address[1]
    response = _send_and_receive(port, build_sentence("STA"))
    _, fields = parse_sentence(response)
    assert fields[14] == "UNKNOWN"


def test_sta_dgps_field_reports_dgps_once_a_corrected_fix_arrives(running_server):
    running_server.state.update_gps_fix(47.391033, -0.738500, is_dgps=True)
    port = running_server.server_address[1]
    response = _send_and_receive(port, build_sentence("STA"))
    _, fields = parse_sentence(response)
    assert fields[14] == "DGPS"


def test_sta_dgps_field_reports_gps_for_an_uncorrected_fix(running_server):
    running_server.state.update_gps_fix(47.391033, -0.738500, is_dgps=False)
    port = running_server.server_address[1]
    response = _send_and_receive(port, build_sentence("STA"))
    _, fields = parse_sentence(response)
    assert fields[14] == "GPS"


def test_sta_dgps_field_switches_back_from_dgps_to_gps(running_server):
    # is_dgps is meant to track the *latest* fix, not latch true forever.
    running_server.state.update_gps_fix(47.391033, -0.738500, is_dgps=True)
    running_server.state.update_gps_fix(47.391034, -0.738501, is_dgps=False)
    port = running_server.server_address[1]
    response = _send_and_receive(port, build_sentence("STA"))
    _, fields = parse_sentence(response)
    assert fields[14] == "GPS"


# --- WPT / GRT (2026-09-19, for robot-webserver's /control map) ------------

def test_wpt_over_real_socket_returns_no_points_when_none_saved_yet(running_server):
    port = running_server.server_address[1]
    response = _send_and_receive(port, build_sentence("WPT"))
    sentence_type, fields = parse_sentence(response)
    assert sentence_type == "WPT"
    assert fields == ["0"]


def test_wpt_over_real_socket_returns_saved_waypoints(running_server, tmp_path, monkeypatch):
    monkeypatch.setenv("WAYPOINTS_FILE", str(tmp_path / "waypoints.txt"))
    running_server.state.update_gps_fix(47.391534, -0.739006)
    running_server.state.save_waypoint()

    port = running_server.server_address[1]
    response = _send_and_receive(port, build_sentence("WPT"))
    sentence_type, fields = parse_sentence(response)
    assert sentence_type == "WPT"
    assert fields[0] == "1"
    lat = nmea_to_decimal(fields[1], fields[2])
    lon = nmea_to_decimal(fields[3], fields[4])
    assert lat == pytest.approx(47.391534, abs=1e-4)
    assert lon == pytest.approx(-0.739006, abs=1e-4)


def test_grt_over_real_socket_returns_no_points_when_no_route_sent(running_server):
    port = running_server.server_address[1]
    response = _send_and_receive(port, build_sentence("GRT"))
    sentence_type, fields = parse_sentence(response)
    assert sentence_type == "GRT"
    assert fields == ["0"]


def test_grt_over_real_socket_returns_the_active_route_after_rte(running_server):
    port = running_server.server_address[1]
    _send_and_receive(port, build_sentence(
        "RTE", 2, 4807.038, "N", 1131.000, "E", 4823.192, "N", 1152.500, "E",
    ))
    response = _send_and_receive(port, build_sentence("GRT"))
    sentence_type, fields = parse_sentence(response)
    assert sentence_type == "GRT"
    assert fields[0] == "2"
    assert fields[1:5] == ["4807.038", "N", "1131.0", "E"]
    assert fields[5:9] == ["4823.192", "N", "1152.5", "E"]


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
    assert fields[10:14] == ["4723.492", "N", "00044.340", "W"]


def test_control_server_starts_fine_without_gps_hardware(monkeypatch):
    # start_gps/start_motor/start_gamepad all default to True here, with
    # GPS pointed at a device path that cannot possibly exist -- and, in
    # this sandbox, gpiod/evdev not installed at all for the motor
    # driver/gamepad. The point is confirming none of the three ever
    # crashes server startup, whether on this dev machine or on a Pi
    # missing one piece of hardware (no receiver plugged in, no
    # controller connected, motor driver board unplugged...).
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


# --- BTN_START -> ControlServer._shutdown_pi (2026-09-12) -------------------
# subprocess.run is mocked in every test here -- this must NEVER actually
# run `sudo poweroff` (or whatever SHUTDOWN_CMD is set to) against the
# machine running the test suite.

def test_shutdown_pi_runs_the_configured_shutdown_command(running_server):
    with patch("link.server.subprocess.run") as mock_run:
        running_server._shutdown_pi()
    mock_run.assert_called_once_with(SHUTDOWN_CMD, check=True, timeout=10)


def test_shutdown_pi_stops_serve_forever_even_if_the_shutdown_command_fails():
    # The whole point of the try/finally in _shutdown_pi(): a missing
    # sudoers entry (very plausible on a Pi that hasn't had the one-time
    # setup done yet) must not leave the control server running as if
    # nothing happened -- this robot's own scripts still need to stop.
    server = ControlServer("127.0.0.1", 0, start_gps=False, start_motor=False, start_gamepad=False)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        with patch("link.server.subprocess.run", side_effect=OSError("sudo: command not found")):
            server._shutdown_pi()  # must not raise despite the failed subprocess call
        thread.join(timeout=2)
        assert not thread.is_alive()  # serve_forever() actually returned
    finally:
        server.shutdown()
        server.server_close()
