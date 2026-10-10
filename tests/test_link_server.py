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


# --- NAV/RTE plausibility check (2026-10-07) --------------------------------
# Catches the "typed in plain decimal degrees instead of this protocol's
# ddmm.mmmm wire format" mistake -- see RAW_LAT_MAGNITUDE_FLOOR's honesty
# note in robot_state.py. Both are "valid floats" so the older
# BAD_LAT_LON_VALUE check alone never caught this.

def test_set_nav_target_rejects_plain_decimal_degrees():
    state = RobotState()
    with pytest.raises(CommandError) as exc_info:
        # 47.391534 is the SAME point as 4723.492 above, just typed as
        # decimal degrees -- a raw magnitude of 47 is far below
        # RAW_LAT_MAGNITUDE_FLOOR (90), so this must be rejected rather
        # than silently decoding to ~0.79 degrees North.
        state.set_nav_target("47.391534", "N", "0.739006", "W")
    assert exc_info.value.code == "22"
    assert "NAV_LAT_LON_IMPLAUSIBLE" in str(exc_info.value)
    # Rejected before anything else changes -- no half-applied NAV.
    assert state.nav_target is None


def test_set_nav_target_accepts_genuine_ddmm_values_far_from_france():
    # Sanity check that the floor doesn't reject real points -- including
    # ones from well outside this project's own usual area (same Munich
    # example the over-the-wire tests already use), confirming this is a
    # format check, not a "near France" geography check.
    state = RobotState()
    state.set_nav_target("4807.038", "N", "1131.000", "E")
    assert state.nav_target == ("4807.038", "N", "1131.000", "E")


def test_set_route_rejects_plain_decimal_degrees_point():
    state = RobotState()
    with pytest.raises(CommandError) as exc_info:
        state.set_route(["1", "47.391534", "N", "0.739006", "W"])
    assert exc_info.value.code == "13"
    assert "RTE_LAT_LON_IMPLAUSIBLE" in str(exc_info.value)
    assert state.route == []


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


def test_get_route_progress_reflects_index_and_return_flag(tmp_path, monkeypatch):
    waypoints_file = tmp_path / "waypoints.txt"
    waypoints_file.write_text("10.0,20.0,t1\n")
    monkeypatch.setenv("WAYPOINTS_FILE", str(waypoints_file))
    state = RobotState()

    # No route yet.
    route, index, is_return = state.get_route_progress()
    assert (route, index, is_return) == ([], 0, False)

    # An ordinary GPS Driving route.
    state.set_route(["2", "4807.038", "N", "1131.000", "E", "4823.192", "N", "1152.500", "E"])
    route, index, is_return = state.get_route_progress()
    assert route == [("4807.038", "N", "1131.000", "E"), ("4823.192", "N", "1152.500", "E")]
    assert (index, is_return) == (0, False)

    # A waypoint return overrides it.
    state.update_gps_fix(10.0, 20.0)
    state.start_waypoint_return()
    route, index, is_return = state.get_route_progress()
    assert len(route) == 1
    assert (index, is_return) == (0, True)


def test_get_route_progress_returns_a_copy_not_the_live_list():
    state = RobotState()
    state.set_route(["1", "4807.038", "N", "1131.000", "E"])
    snapshot, _, _ = state.get_route_progress()
    state.stop()
    assert snapshot == [("4807.038", "N", "1131.000", "E")]


# --- End-to-end socket tests ------------------------------------------------

@pytest.fixture()
def running_server():
    # start_gps/start_motor/start_gamepad/start_tracer/start_history=False:
    # tests don't need a real (or attempted) GPS fix, GPIO chip, gamepad,
    # RS485 read, or a background history logger writing to data/
    # power_history.db in whatever directory the test suite happens to
    # run from -- this avoids every test run making real subprocess/
    # device-scan/serial-open calls (gpiodetect, evdev.list_devices,
    # serial.Serial(...)) or leaving SQLite files behind. See
    # test_control_server_starts_fine_without_gps_hardware and
    # test_control_server_starts_fine_without_tracer_hardware below for
    # dedicated checks that leaving these at their True default doesn't
    # crash when the hardware behind them isn't there, and
    # test_his_* below (own fixture, own tmp DB) for the history logger.
    server = ControlServer(
        "127.0.0.1", 0,  # port 0 = pick a free port
        start_gps=False, start_motor=False, start_gamepad=False, start_tracer=False,
        start_history=False,
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


def test_nav_rejects_plain_decimal_degrees_over_real_socket(running_server):
    # Same mistake as test_set_nav_target_rejects_plain_decimal_degrees
    # (unit-level), checked end-to-end over the actual TCP socket this
    # time -- see RAW_LAT_MAGNITUDE_FLOOR's honesty note in robot_state.py.
    port = running_server.server_address[1]
    response = _send_and_receive(port, build_sentence("NAV", 47.391534, "N", 0.739006, "W"))
    sentence_type, fields = parse_sentence(response)
    assert sentence_type == "ERR"
    assert fields[0] == "22"


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
    # Trailing route_index/mode fields (2026-10-07): 0 and "DRIVE" are
    # arbitrary-but-consistent defaults here since count is already 0 --
    # see get_route_progress()/server.py's GRT handler.
    assert fields == ["0", "0", "DRIVE"]


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
    # Trailing route_index/mode fields (2026-10-07): a fresh RTE upload is
    # a "GPS Driving" route, not yet advanced past its first point.
    assert fields[9] == "0"
    assert fields[10] == "DRIVE"


def test_grt_over_real_socket_reports_return_mode_after_waypoint_return(
    running_server, tmp_path, monkeypatch
):
    # Isolate WAYPOINTS_FILE (same reasoning as
    # test_wpt_over_real_socket_returns_saved_waypoints above) -- without
    # this, save_waypoint() below appends to the real default
    # waypoints/waypoints.txt, which other running_server-based tests in
    # this same file may have already written to, making the "exactly 1
    # waypoint" assumption below flaky depending on test order.
    monkeypatch.setenv("WAYPOINTS_FILE", str(tmp_path / "waypoints.txt"))
    port = running_server.server_address[1]
    running_server.state.update_gps_fix(10.0, 20.0)
    running_server.state.save_waypoint()
    running_server.state.start_waypoint_return()

    response = _send_and_receive(port, build_sentence("GRT"))
    sentence_type, fields = parse_sentence(response)
    assert sentence_type == "GRT"
    assert fields[0] == "1"
    assert fields[-2:] == ["0", "RETURN"]


def test_grt_over_real_socket_reports_route_index_as_robot_advances(running_server):
    port = running_server.server_address[1]
    p1 = ("4723.492", "N", "00044.340", "W")   # ~ 47.391533, -0.739
    p2 = ("4723.532", "N", "00044.292", "W")   # ~ 95m away
    _send_and_receive(port, build_sentence("RTE", 2, *p1, *p2))

    response = _send_and_receive(port, build_sentence("GRT"))
    _, fields = parse_sentence(response)
    assert fields[-2:] == ["0", "DRIVE"]  # not yet arrived at p1

    running_server.state.update_gps_fix(47.391533, -0.739)  # on top of p1

    response = _send_and_receive(port, build_sentence("GRT"))
    _, fields = parse_sentence(response)
    assert fields[0] == "2"  # both points still listed -- the map's full route is unaffected
    assert fields[-2:] == ["1", "DRIVE"]  # but route_index advanced to the next leg


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


def test_control_server_starts_fine_without_tracer_hardware(monkeypatch):
    # Same idea as test_control_server_starts_fine_without_gps_hardware,
    # for the RS485/Tracer reader (2026-10-03): a device path that cannot
    # possibly exist must never crash server startup, and PWR must still
    # answer (with the "unavailable" placeholder) rather than hang or
    # raise.
    monkeypatch.setenv("TRACER_DEVICE", "/dev/definitely-not-a-real-device")
    server = ControlServer("127.0.0.1", 0)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        response = _send_and_receive(
            server.server_address[1], build_sentence("PWR")
        )
        sentence_type, fields = parse_sentence(response)
        assert sentence_type == "PWR"
        assert fields[13] == "0"  # available=0 -- no Tracer ever read
    finally:
        server.shutdown()
        server.server_close()


# --- PWR (2026-10-03, for robot-webserver's /power page) --------------------
# cpu_temp (2026-10-05) is monkeypatched to a fixed value in every test
# below rather than left to the real /sys/class/thermal/thermal_zone0/temp
# read: it's Pi #1's own CPU, nothing to do with the Tracer fields these
# tests are actually about, and leaving it unpatched would make these
# tests' expected fields depend on whether they happen to run on a real
# Raspberry Pi (see link.cpu_temp) -- patched where robot_state looks it
# up (link.robot_state.read_cpu_temperature_c), not where it's defined.

def test_pwr_over_real_socket_reports_unavailable_before_any_reading(running_server, monkeypatch):
    monkeypatch.setattr("link.robot_state.read_cpu_temperature_c", lambda: None)
    port = running_server.server_address[1]
    response = _send_and_receive(port, build_sentence("PWR"))
    sentence_type, fields = parse_sentence(response)
    assert sentence_type == "PWR"
    # 9 Tracer flow fields + battery_soc/battery_temp/controller_temp +
    # cpu_temp (also "0.0" here -- the monkeypatched reader returned None)
    # + the available flag. onboard_time/onboard_time_source/uptime_s
    # (2026-10-10) are checked separately below -- none of them is ever a
    # fixed value.
    assert fields[:13] == ["0.0"] * 9 + ["0.0", "0.0", "0.0"] + ["0.0"]
    assert fields[13] == "0"
    assert fields[14].isdigit()
    assert fields[15] == "SYS"  # no GPS fix either -- falls back to the system clock
    assert fields[16].isdigit()


def test_pwr_over_real_socket_reports_a_real_reading(running_server, monkeypatch):
    monkeypatch.setattr("link.robot_state.read_cpu_temperature_c", lambda: 46.5)
    port = running_server.server_address[1]
    running_server.state.update_power_reading(
        available=True,
        pv_voltage=15.82, pv_current=1.2, pv_power=18.98,
        battery_voltage=13.1, battery_charging_current=1.1, battery_charging_power=14.41,
        load_voltage=12.9, load_current=0.5, load_power=6.45,
        battery_soc=87, battery_temp=24.3, controller_temp=28.1,
    )
    response = _send_and_receive(port, build_sentence("PWR"))
    sentence_type, fields = parse_sentence(response)
    assert sentence_type == "PWR"
    assert fields[0:3] == ["15.82", "1.2", "18.98"]
    assert fields[3:6] == ["13.1", "1.1", "14.41"]
    assert fields[6:9] == ["12.9", "0.5", "6.45"]
    assert fields[9:12] == ["87.0", "24.3", "28.1"]
    assert fields[12] == "46.5"  # cpu_temp -- independent of the Tracer's own available flag
    assert fields[13] == "1"
    assert fields[14].isdigit()  # onboard_time -- whole-second Unix timestamp
    assert fields[15] == "SYS"
    assert fields[16].isdigit()  # uptime_s


def test_pwr_cpu_temp_is_reported_even_when_tracer_is_unavailable(running_server, monkeypatch):
    # The whole point of reading cpu_temp outside of update_power_reading():
    # an unplugged Tracer cable must not blank out Pi #1's own CPU reading.
    monkeypatch.setattr("link.robot_state.read_cpu_temperature_c", lambda: 52.1)
    port = running_server.server_address[1]
    response = _send_and_receive(port, build_sentence("PWR"))
    sentence_type, fields = parse_sentence(response)
    assert sentence_type == "PWR"
    assert fields[12] == "52.1"
    assert fields[13] == "0"  # Tracer itself still correctly reported as unavailable


def test_pwr_reports_unavailable_again_after_a_failed_poll_but_keeps_last_values(running_server, monkeypatch):
    monkeypatch.setattr("link.robot_state.read_cpu_temperature_c", lambda: None)
    port = running_server.server_address[1]
    running_server.state.update_power_reading(available=True, pv_voltage=15.82)
    running_server.state.update_power_reading(available=False)
    response = _send_and_receive(port, build_sentence("PWR"))
    sentence_type, fields = parse_sentence(response)
    assert sentence_type == "PWR"
    assert fields[0] == "15.82"  # last known-good value, not reset to 0.0
    assert fields[13] == "0"     # but correctly flagged as stale/unavailable


def test_pwr_onboard_time_is_a_live_unix_timestamp_independent_of_tracer(running_server, monkeypatch):
    # Same spirit as cpu_temp above: Pi #1's own system clock has nothing
    # to do with the Tracer, so it's reported -- and advances -- whether
    # or not the Tracer is available. Explicit user request (2026-10-10):
    # the /power page shows this next to the operator's own watch to spot
    # a drifted Pi #1 clock.
    monkeypatch.setattr("link.robot_state.read_cpu_temperature_c", lambda: None)
    port = running_server.server_address[1]
    before = time.time()
    response = _send_and_receive(port, build_sentence("PWR"))
    after = time.time()
    _, fields = parse_sentence(response)
    onboard_time = int(fields[14])
    assert int(before) - 1 <= onboard_time <= int(after) + 1


def test_pwr_falls_back_to_gps_time_when_the_system_clock_is_implausible(running_server, monkeypatch):
    # Explicit user request (2026-10-10): "si le onboard time n'est pas
    # réglé par le Wifi, prends celui du GPS" -- once a GPS fix has
    # arrived, its own UTC time is correct independently of WiFi/NTP, so
    # it's used in place of the (here, deliberately implausible) system
    # clock, and onboard_time_source is labelled accordingly so /power
    # can show the operator which source is live.
    monkeypatch.setattr("link.robot_state.read_cpu_temperature_c", lambda: None)
    monkeypatch.setattr(time, "time", lambda: 1000.0)  # 1970-01-01ish -- implausible
    running_server.state.update_gps_fix(47.392343, -0.739000, gps_utc_ts=1780000000.0)
    port = running_server.server_address[1]
    response = _send_and_receive(port, build_sentence("PWR"))
    _, fields = parse_sentence(response)
    assert fields[14] == "1780000000"
    assert fields[15] == "GPS"


def test_pwr_falls_back_to_the_system_clock_when_no_gps_fix_has_ever_arrived(running_server, monkeypatch):
    # Same implausible clock as above, but no GPS fix at all -- there is
    # nothing better to fall back to, so onboard_time still reports the
    # (known wrong) system clock, correctly labelled "SYS" either way --
    # a plausible clock and "no better option" both report the same
    # source label, only the plausibility check differs.
    monkeypatch.setattr("link.robot_state.read_cpu_temperature_c", lambda: None)
    monkeypatch.setattr(time, "time", lambda: 1000.0)
    port = running_server.server_address[1]
    response = _send_and_receive(port, build_sentence("PWR"))
    _, fields = parse_sentence(response)
    assert fields[14] == "1000"
    assert fields[15] == "SYS"


def test_pwr_uptime_s_reports_the_mocked_read_uptime_s_value(running_server, monkeypatch):
    # uptime_s (2026-10-10, explicit user request -- the /power page's
    # small "lifetime" readout) comes from link.uptime.read_uptime_s(),
    # read fresh outside RobotState's lock the same way cpu_temp is --
    # mocked the same way (patched where robot_state looks it up, not
    # where it's defined).
    monkeypatch.setattr("link.robot_state.read_cpu_temperature_c", lambda: None)
    monkeypatch.setattr("link.robot_state.read_uptime_s", lambda: 12345)
    port = running_server.server_address[1]
    response = _send_and_receive(port, build_sentence("PWR"))
    _, fields = parse_sentence(response)
    assert fields[16] == "12345"


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


# --- HIS / power history (2026-10-05) --------------------------------------

@pytest.fixture()
def history_server(tmp_path, monkeypatch):
    """Own fixture rather than reusing `running_server`: HIS needs
    start_history=True and a throwaway POWER_HISTORY_DB_PATH (tmp_path,
    so nothing is left behind in the repo and tests never share a
    database with each other)."""
    monkeypatch.setenv("POWER_HISTORY_DB_PATH", str(tmp_path / "power_history.db"))
    server = ControlServer(
        "127.0.0.1", 0,
        start_gps=False, start_motor=False, start_gamepad=False, start_tracer=False,
        start_history=True,
    )
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        yield server
    finally:
        server.shutdown()
        server.server_close()


def test_his_empty_database_returns_zero_rows(history_server):
    port = history_server.server_address[1]
    response = _send_and_receive(port, build_sentence("HIS", "DAY", 0))
    sentence_type, fields = parse_sentence(response)
    assert sentence_type == "HIS"
    assert fields[:4] == ["DAY", "0", "0", "0"]


def test_his_rejects_bad_period(history_server):
    port = history_server.server_address[1]
    response = _send_and_receive(port, build_sentence("HIS", "YEAR", 0))
    sentence_type, fields = parse_sentence(response)
    assert sentence_type == "ERR"
    assert fields[0] == "16"


def test_his_rejects_wrong_field_count(history_server):
    port = history_server.server_address[1]
    response = _send_and_receive(port, build_sentence("HIS", "DAY"))
    sentence_type, fields = parse_sentence(response)
    assert sentence_type == "ERR"
    assert fields[0] == "15"


def test_his_returns_logged_rows_paginated(history_server, monkeypatch):
    from link import power_history as ph

    db_path = ph.resolve_db_path()
    now = int(time.time())
    # 5 rows, 5 minutes apart, well inside the DAY window.
    for i in range(5):
        sample = {key: float(i) for key in ph.FIELD_ORDER}
        sample["ts"] = now - i * 300
        sample["lat"] = None
        sample["lon"] = None
        ph.insert_sample(db_path, sample)

    port = history_server.server_address[1]

    # HIS_CHUNK_ROWS is 100 by default, well above 5 -- one page covers
    # everything, but patch it down to 2 here so the pagination offset
    # logic itself (not just "small enough to fit in one page") is
    # actually exercised.
    monkeypatch.setattr(ph, "HIS_CHUNK_ROWS", 2)

    collected = []
    offset = 0
    while True:
        response = _send_and_receive(port, build_sentence("HIS", "DAY", offset))
        sentence_type, fields = parse_sentence(response)
        assert sentence_type == "HIS"
        period, total_count, resp_offset, returned_count = (
            fields[0], int(fields[1]), int(fields[2]), int(fields[3]),
        )
        assert period == "DAY"
        assert total_count == 5
        assert resp_offset == offset
        row_fields = fields[4:]
        assert len(row_fields) == returned_count * len(ph.FIELD_ORDER)
        collected.extend(row_fields)
        offset += returned_count
        if returned_count == 0 or offset >= total_count:
            break

    assert len(collected) == 5 * len(ph.FIELD_ORDER)


def test_power_history_logger_skips_writes_when_tracer_unavailable(history_server):
    # The control server's own PowerHistoryLogger ticks every
    # POWER_LOG_INTERVAL_S (5 minutes by default) -- far too slow for a
    # test, so this calls its _log_once() directly rather than waiting.
    # No Tracer is running in this fixture (start_tracer=False), so
    # state.power_status()["available"] is always False/None here --
    # exactly the case test_his_empty_database_returns_zero_rows above
    # already covers end-to-end; this test names the gating rule
    # explicitly so a future change that logs unconditionally fails
    # loudly.
    history_server.power_history_logger._log_once()
    from link import power_history as ph
    total, _ = ph.fetch_period_chunk(ph.resolve_db_path(), "DAY", 0)
    assert total == 0


def test_snapshot_is_geotagged_on_successful_cam_snap(history_server, monkeypatch):
    # _request_snapshot() calls camera/stream_server.py over HTTP --
    # mock urlopen so this runs without a camera process actually
    # running.
    import json

    class FakeResponse:
        status = 200

        def read(self):
            return json.dumps({"ok": True, "file": "snap_0001.jpg"}).encode()

        def __enter__(self):
            return self

        def __exit__(self, *args):
            return False

    monkeypatch.setattr("link.robot_state.urllib.request.urlopen", lambda *a, **k: FakeResponse())

    state = history_server.state
    with state._lock:
        state.current_lat = 48.8566
        state.current_lon = 2.3522
    state.camera_command("SNAP")

    from link import power_history as ph
    import sqlite3
    conn = sqlite3.connect(ph.resolve_db_path())
    row = conn.execute(
        "SELECT filename, lat, lon FROM snapshots WHERE filename=?", ("snap_0001.jpg",)
    ).fetchone()
    assert row == ("snap_0001.jpg", 48.8566, 2.3522)


def test_log_media_skips_db_write_while_clock_is_implausible(history_server, monkeypatch):
    # Same clock-plausibility gate as PowerHistoryLogger._log_once() and
    # RobotState.update_gps_fix()'s solar-survey buffering (2026-10-10,
    # explicit user request) -- no racing background thread involved
    # here, log_media() is only ever called synchronously from
    # camera_command(), never from the logger's own background tick.
    monkeypatch.setattr(time, "time", lambda: 1000.0)
    from link import power_history as ph
    ph.log_media(ph.resolve_db_path(), "snap_implausible.jpg", "photo", 1.0, 1.0)

    conn = __import__("sqlite3").connect(ph.resolve_db_path())
    row = conn.execute(
        "SELECT filename FROM snapshots WHERE filename=?", ("snap_implausible.jpg",)
    ).fetchone()
    assert row is None


def test_snapshot_is_not_geotagged_while_clock_is_implausible(history_server, monkeypatch):
    # End-to-end version of the test above, through camera_command("SNAP")
    # -- the file itself would still exist on disk (camera/stream_server.py
    # is the one actually writing it, mocked out here), only the map
    # geotag is skipped.
    import json

    class FakeResponse:
        status = 200

        def read(self):
            return json.dumps({"ok": True, "file": "snap_0002.jpg"}).encode()

        def __enter__(self):
            return self

        def __exit__(self, *args):
            return False

    monkeypatch.setattr("link.robot_state.urllib.request.urlopen", lambda *a, **k: FakeResponse())
    monkeypatch.setattr(time, "time", lambda: 1000.0)

    state = history_server.state
    with state._lock:
        state.current_lat = 48.8566
        state.current_lon = 2.3522
    state.camera_command("SNAP")

    from link import power_history as ph
    import sqlite3
    conn = sqlite3.connect(ph.resolve_db_path())
    row = conn.execute(
        "SELECT filename FROM snapshots WHERE filename=?", ("snap_0002.jpg",)
    ).fetchone()
    assert row is None


# --- MED: geotagged photos/videos for the /control map's violet markers,
# and the BTN_A waypoint-return feature (2026-10-05) ------------------------

def test_recording_is_geotagged_at_start_position_not_stop_position(history_server, monkeypatch):
    # Explicit user requirement: a video's geotag is the position live
    # when REC_START succeeded, even if the robot has since moved by the
    # time REC_STOP actually hands back the filename (VideoRecorder only
    # assigns one lazily, on its first written frame -- see
    # link/robot_state.py's _request_recording()).
    import json

    class FakeResponse:
        def __init__(self, body):
            self._body = body
            self.status = 200

        def read(self):
            return self._body

        def __enter__(self):
            return self

        def __exit__(self, *args):
            return False

    def fake_urlopen(url, timeout=None):
        if url.endswith("/rec/start"):
            return FakeResponse(json.dumps({"ok": True}).encode())
        return FakeResponse(json.dumps({"ok": True, "recording": False, "file": "rec_0001.mp4"}).encode())

    monkeypatch.setattr("link.robot_state.urllib.request.urlopen", fake_urlopen)

    state = history_server.state
    with state._lock:
        state.current_lat, state.current_lon = 48.0, 2.0
    state.camera_command("REC_START")
    with state._lock:
        state.current_lat, state.current_lon = 99.0, 99.0  # moved mid-"recording"
    state.camera_command("REC_STOP")

    from link import power_history as ph
    import sqlite3
    conn = sqlite3.connect(ph.resolve_db_path())
    row = conn.execute(
        "SELECT filename, kind, lat, lon FROM snapshots WHERE filename=?", ("rec_0001.mp4",)
    ).fetchone()
    assert row == ("rec_0001.mp4", "video", 48.0, 2.0)


def test_media_positions_excludes_files_rotated_out_of_the_camera_stores(history_server, monkeypatch):
    # A geotag whose file has since been pruned by camera/snapshots.py's
    # or camera/recordings.py's capped-5-FIFO stores must never show up as
    # a dead marker on the map -- media_positions() only returns rows for
    # filenames the camera process's /snapshots or /recordings listing
    # still actually reports.
    import json

    from link import power_history as ph
    ph.log_media(ph.resolve_db_path(), "snap_still_here.jpg", "photo", 1.0, 1.0)
    ph.log_media(ph.resolve_db_path(), "snap_rotated_out.jpg", "photo", 2.0, 2.0)

    def fake_urlopen(url, timeout=None):
        class FakeResponse:
            def read(self_inner):
                if url.endswith("/snapshots"):
                    return json.dumps({"ok": True, "snapshots": ["snap_still_here.jpg"]}).encode()
                return json.dumps({"ok": True, "recordings": []}).encode()

            def __enter__(self_inner):
                return self_inner

            def __exit__(self_inner, *a):
                return False

        return FakeResponse()

    monkeypatch.setattr("link.robot_state.urllib.request.urlopen", fake_urlopen)
    positions = history_server.state.media_positions()
    names = {p[0] for p in positions}
    assert names == {"snap_still_here.jpg"}


def test_med_sentence_over_real_socket(history_server, monkeypatch):
    import json

    from link import power_history as ph
    ph.log_media(ph.resolve_db_path(), "snap_a.jpg", "photo", 10.0, 20.0)

    def fake_urlopen(url, timeout=None):
        class FakeResponse:
            def read(self_inner):
                if url.endswith("/snapshots"):
                    return json.dumps({"ok": True, "snapshots": ["snap_a.jpg"]}).encode()
                return json.dumps({"ok": True, "recordings": []}).encode()

            def __enter__(self_inner):
                return self_inner

            def __exit__(self_inner, *a):
                return False

        return FakeResponse()

    monkeypatch.setattr("link.robot_state.urllib.request.urlopen", fake_urlopen)

    port = history_server.server_address[1]
    response = _send_and_receive(port, build_sentence("MED"))

    resp_type, fields = parse_sentence(response)
    assert resp_type == "MED"
    assert fields[0] == "1"
    assert fields[1] == "snap_a.jpg"
    assert fields[2] == "SNAP"


def test_start_waypoint_return_raises_with_no_gps_fix():
    state = RobotState()
    with pytest.raises(CommandError) as exc_info:
        state.start_waypoint_return()
    assert exc_info.value.code == "14"


def test_start_waypoint_return_raises_with_no_waypoints_saved(tmp_path, monkeypatch):
    monkeypatch.setenv("WAYPOINTS_FILE", str(tmp_path / "waypoints.txt"))
    state = RobotState()
    state.update_gps_fix(1.0, 1.0)
    with pytest.raises(CommandError) as exc_info:
        state.start_waypoint_return()
    assert exc_info.value.code == "17"


def test_start_waypoint_return_drives_waypoints_in_reverse_save_order(tmp_path, monkeypatch):
    waypoints_file = tmp_path / "waypoints.txt"
    waypoints_file.write_text(
        "10.000000,20.000000,t1\n"
        "10.001000,20.001000,t2\n"
        "10.002000,20.002000,t3\n"
    )
    monkeypatch.setenv("WAYPOINTS_FILE", str(waypoints_file))
    state = RobotState()
    state.update_gps_fix(10.0, 20.0)

    count = state.start_waypoint_return()
    assert count == 3
    assert state.route_is_return is True
    assert state.mode == "AUTO"
    # Reversed: the LAST-saved point (10.002/20.002) is chased first.
    assert nmea_to_decimal(state.nav_target[0], state.nav_target[1]) == pytest.approx(10.002, abs=1e-4)


def test_start_waypoint_return_overrides_an_active_gps_driving_route(tmp_path, monkeypatch):
    waypoints_file = tmp_path / "waypoints.txt"
    waypoints_file.write_text("10.0,20.0,t1\n")
    monkeypatch.setenv("WAYPOINTS_FILE", str(waypoints_file))
    state = RobotState()
    state.update_gps_fix(10.0, 20.0)
    state.set_route([1, "0600.000", "N", "02000.000", "E"])
    assert state.route_is_return is False

    state.start_waypoint_return()
    assert state.route_is_return is True
    assert len(state.route) == 1


def test_start_waypoint_return_deletes_each_waypoint_as_it_is_reached(tmp_path, monkeypatch):
    waypoints_file = tmp_path / "waypoints.txt"
    waypoints_file.write_text(
        "10.000000,20.000000,t1\n"
        "10.001000,20.001000,t2\n"
    )
    monkeypatch.setenv("WAYPOINTS_FILE", str(waypoints_file))
    state = RobotState()
    state.update_gps_fix(10.0, 20.0)
    state.start_waypoint_return()

    # First leg: the last-saved point (10.001/20.001).
    state.update_gps_fix(10.001, 20.001)
    remaining = [l for l in waypoints_file.read_text().splitlines() if l.strip()]
    assert remaining == ["10.000000,20.000000,t1"]

    # Second (final) leg: the first-saved point (10.000/20.000).
    state.update_gps_fix(10.000, 20.000)
    remaining = [l for l in waypoints_file.read_text().splitlines() if l.strip()]
    assert remaining == []
    assert state.route_index == len(state.route)


def test_a_fresh_rte_after_a_return_clears_route_is_return(tmp_path, monkeypatch):
    waypoints_file = tmp_path / "waypoints.txt"
    waypoints_file.write_text("10.0,20.0,t1\n")
    monkeypatch.setenv("WAYPOINTS_FILE", str(waypoints_file))
    state = RobotState()
    state.update_gps_fix(10.0, 20.0)
    state.start_waypoint_return()
    assert state.route_is_return is True

    state.set_route([1, "0600.000", "N", "02000.000", "E"])
    assert state.route_is_return is False


# --- BTN_A/MANUAL bug fix (2026-10-10): a return route must be fully
# re-armable after the operator takes back manual control mid-route --
# https://github.com/Fafou49/robot issue reported 2026-10-10: pressing
# BTN_A again after switching back to MANUAL used to leave the waypoints
# stuck "NAV" (route_is_return still True) and unreachable. ------------------

def test_returning_to_manual_clears_an_active_return_route(tmp_path, monkeypatch):
    waypoints_file = tmp_path / "waypoints.txt"
    waypoints_file.write_text("10.0,20.0,t1\n10.001,20.001,t2\n")
    monkeypatch.setenv("WAYPOINTS_FILE", str(waypoints_file))
    state = RobotState()
    state.update_gps_fix(10.0, 20.0)
    state.start_waypoint_return()
    assert state.route_is_return is True
    assert state.route != []

    state.set_mode("MANUAL")
    assert state.route_is_return is False
    assert state.route == []
    assert state.route_index == 0


def test_returning_to_idle_also_clears_an_active_return_route(tmp_path, monkeypatch):
    waypoints_file = tmp_path / "waypoints.txt"
    waypoints_file.write_text("10.0,20.0,t1\n")
    monkeypatch.setenv("WAYPOINTS_FILE", str(waypoints_file))
    state = RobotState()
    state.update_gps_fix(10.0, 20.0)
    state.start_waypoint_return()

    state.set_mode("IDLE")
    assert state.route_is_return is False
    assert state.route == []


def test_manual_driving_no_longer_silently_consumes_return_waypoints(tmp_path, monkeypatch):
    # Before the fix, _advance_route_if_arrived() kept running on every GPS
    # fix regardless of mode, so simply driving the robot manually near the
    # (now-abandoned) return route's next target would delete it from
    # waypoints.txt even though the robot never actually auto-navigated
    # there. Clearing the route on leaving AUTO (set_mode) makes this a
    # no-op once back in MANUAL.
    waypoints_file = tmp_path / "waypoints.txt"
    waypoints_file.write_text("10.000000,20.000000,t1\n10.001000,20.001000,t2\n")
    monkeypatch.setenv("WAYPOINTS_FILE", str(waypoints_file))
    state = RobotState()
    state.update_gps_fix(10.0, 20.0)
    state.start_waypoint_return()  # chases t2 (last-saved) first

    state.set_mode("MANUAL")
    # Manually drive right on top of the point the route used to be
    # chasing -- must NOT advance/consume anything now that the route has
    # been cleared.
    state.update_gps_fix(10.001, 20.001)
    remaining = [l for l in waypoints_file.read_text().splitlines() if l.strip()]
    assert remaining == ["10.000000,20.000000,t1", "10.001000,20.001000,t2"]


def test_fresh_btn_a_after_returning_to_manual_rebuilds_the_route(tmp_path, monkeypatch):
    waypoints_file = tmp_path / "waypoints.txt"
    waypoints_file.write_text("10.000000,20.000000,t1\n10.001000,20.001000,t2\n")
    monkeypatch.setenv("WAYPOINTS_FILE", str(waypoints_file))
    state = RobotState()
    state.update_gps_fix(10.0, 20.0)
    state.start_waypoint_return()

    state.set_mode("MANUAL")
    count = state.start_waypoint_return()
    assert count == 2
    assert state.route_is_return is True
    assert state.mode == "AUTO"


def test_set_mode_to_manual_does_not_clear_a_lone_nav_target(tmp_path, monkeypatch):
    # nav_target is deliberately left alone (matches stop()'s own
    # precedent, see test_has_nav_target_false_again_after_stop above) --
    # only the route/route_is_return bookkeeping is cleared.
    state = RobotState()
    state.set_nav_target("4723.492", "N", "00044.340", "W")
    state.set_mode("MANUAL")
    assert state.has_nav_target() is True


# --- WPD / MDD: the /control map's right-click "delete this point"
# (2026-10-05), for blue waypoints and violet photos/videos ----------------

def test_delete_waypoint_removes_only_the_targeted_entry(tmp_path, monkeypatch):
    waypoints_file = tmp_path / "waypoints.txt"
    waypoints_file.write_text(
        "# a comment, must survive\n"
        "10.000000,20.000000,t1\n"
        "\n"
        "10.001000,20.001000,t2\n"
        "10.002000,20.002000,t3\n"
    )
    monkeypatch.setenv("WAYPOINTS_FILE", str(waypoints_file))
    state = RobotState()

    state.delete_waypoint(1)  # the middle real entry (t2)

    lines = waypoints_file.read_text().splitlines()
    assert "# a comment, must survive" in lines
    assert "" in lines
    assert "10.000000,20.000000,t1" in lines
    assert "10.002000,20.002000,t3" in lines
    assert not any("10.001000" in l for l in lines)


def test_delete_waypoint_out_of_range_raises(tmp_path, monkeypatch):
    waypoints_file = tmp_path / "waypoints.txt"
    waypoints_file.write_text("10.0,20.0,t1\n")
    monkeypatch.setenv("WAYPOINTS_FILE", str(waypoints_file))
    state = RobotState()

    with pytest.raises(CommandError) as exc_info:
        state.delete_waypoint(5)
    assert exc_info.value.code == "18"


def test_delete_waypoint_on_a_missing_file_raises(tmp_path, monkeypatch):
    monkeypatch.setenv("WAYPOINTS_FILE", str(tmp_path / "does_not_exist.txt"))
    state = RobotState()
    with pytest.raises(CommandError) as exc_info:
        state.delete_waypoint(0)
    assert exc_info.value.code == "18"


def test_wpd_sentence_deletes_over_a_real_socket(tmp_path, monkeypatch):
    waypoints_file = tmp_path / "waypoints.txt"
    waypoints_file.write_text("11.0,21.0,t1\n11.1,21.1,t2\n")
    monkeypatch.setenv("WAYPOINTS_FILE", str(waypoints_file))

    server = ControlServer(
        "127.0.0.1", 0,
        start_gps=False, start_motor=False, start_gamepad=False, start_tracer=False,
    )
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        port = server.server_address[1]
        response = _send_and_receive(port, build_sentence("WPD", 0))
        resp_type, fields = parse_sentence(response)
        assert resp_type == "ACK"
        assert fields == ["WPD"]
    finally:
        server.shutdown()
        server.server_close()

    remaining = [l for l in waypoints_file.read_text().splitlines() if l.strip()]
    assert remaining == ["11.1,21.1,t2"]


def test_wpd_sentence_rejects_wrong_field_count(tmp_path, monkeypatch):
    monkeypatch.setenv("WAYPOINTS_FILE", str(tmp_path / "waypoints.txt"))
    server = ControlServer(
        "127.0.0.1", 0,
        start_gps=False, start_motor=False, start_gamepad=False, start_tracer=False,
    )
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        port = server.server_address[1]
        response = _send_and_receive(port, build_sentence("WPD"))
        resp_type, fields = parse_sentence(response)
        assert resp_type == "ERR"
        assert fields[0] == "10"
    finally:
        server.shutdown()
        server.server_close()


def test_delete_media_removes_camera_file_and_db_row(history_server, monkeypatch):
    import json

    from link import power_history as ph
    ph.log_media(ph.resolve_db_path(), "snap_del.jpg", "photo", 5.0, 6.0)

    calls = []

    class FakeResponse:
        def __init__(self, body):
            self._body = body

        def read(self):
            return self._body

        def __enter__(self):
            return self

        def __exit__(self, *a):
            return False

    def fake_urlopen(request, timeout=None):
        calls.append((request.full_url, request.get_method()))
        return FakeResponse(json.dumps({"ok": True, "file": "snap_del.jpg"}).encode())

    monkeypatch.setattr("link.robot_state.urllib.request.urlopen", fake_urlopen)

    history_server.state.delete_media("snap_del.jpg", "photo")

    assert calls[-1][1] == "DELETE"
    assert calls[-1][0].endswith("/snapshots/snap_del.jpg")

    import sqlite3
    conn = sqlite3.connect(ph.resolve_db_path())
    row = conn.execute("SELECT filename FROM snapshots WHERE filename=?", ("snap_del.jpg",)).fetchone()
    assert row is None


def test_delete_media_404_from_camera_raises_but_still_cleans_db(history_server, monkeypatch):
    import urllib.error

    from link import power_history as ph
    ph.log_media(ph.resolve_db_path(), "snap_gone.jpg", "photo", 1.0, 1.0)

    def fake_urlopen(request, timeout=None):
        raise urllib.error.HTTPError(request.full_url, 404, "Not Found", {}, None)

    monkeypatch.setattr("link.robot_state.urllib.request.urlopen", fake_urlopen)

    with pytest.raises(CommandError) as exc_info:
        history_server.state.delete_media("snap_gone.jpg", "photo")
    assert exc_info.value.code == "20"

    import sqlite3
    conn = sqlite3.connect(ph.resolve_db_path())
    row = conn.execute("SELECT filename FROM snapshots WHERE filename=?", ("snap_gone.jpg",)).fetchone()
    assert row is None


def test_delete_media_camera_unreachable_leaves_db_row_alone(history_server, monkeypatch):
    import urllib.error

    from link import power_history as ph
    ph.log_media(ph.resolve_db_path(), "snap_unreachable.jpg", "photo", 2.0, 2.0)

    def fake_urlopen(request, timeout=None):
        raise urllib.error.URLError("connection refused")

    monkeypatch.setattr("link.robot_state.urllib.request.urlopen", fake_urlopen)

    with pytest.raises(CommandError) as exc_info:
        history_server.state.delete_media("snap_unreachable.jpg", "photo")
    assert exc_info.value.code == "12"

    import sqlite3
    conn = sqlite3.connect(ph.resolve_db_path())
    row = conn.execute("SELECT filename FROM snapshots WHERE filename=?", ("snap_unreachable.jpg",)).fetchone()
    assert row is not None


def test_delete_media_rejects_bad_kind():
    state = RobotState()
    with pytest.raises(CommandError) as exc_info:
        state.delete_media("whatever.jpg", "audio")
    assert exc_info.value.code == "19"


def test_mdd_sentence_over_real_socket(history_server, monkeypatch):
    import json

    from link import power_history as ph
    ph.log_media(ph.resolve_db_path(), "snap_mdd.jpg", "photo", 9.0, 9.0)

    class FakeResponse:
        def read(self):
            return json.dumps({"ok": True, "file": "snap_mdd.jpg"}).encode()

        def __enter__(self):
            return self

        def __exit__(self, *a):
            return False

    monkeypatch.setattr("link.robot_state.urllib.request.urlopen", lambda *a, **k: FakeResponse())

    port = history_server.server_address[1]
    response = _send_and_receive(port, build_sentence("MDD", "snap_mdd.jpg", "SNAP"))
    resp_type, fields = parse_sentence(response)
    assert resp_type == "ACK"
    assert fields == ["MDD"]


def test_mdd_sentence_rejects_bad_wire_kind(history_server):
    port = history_server.server_address[1]
    response = _send_and_receive(port, build_sentence("MDD", "x.jpg", "AUDIO"))
    resp_type, fields = parse_sentence(response)
    assert resp_type == "ERR"
    assert fields[0] == "19"


# -- RTD: delete one point from the active route (2026-10-06) ---------------
def _route_point(i):
    return (f"47{i:02d}.000", "N", f"000{i:02d}.000", "W")


def test_delete_route_point_ahead_of_current_target_leaves_index_untouched():
    state = RobotState()
    state.route = [_route_point(1), _route_point(2), _route_point(3)]
    state.route_index = 0
    state.nav_target = state.route[0]

    state.delete_route_point(2)

    assert state.route == [_route_point(1), _route_point(2)]
    assert state.route_index == 0
    assert state.nav_target == _route_point(1)


def test_delete_route_point_behind_current_target_shifts_index_down():
    state = RobotState()
    state.route = [_route_point(1), _route_point(2), _route_point(3)]
    state.route_index = 2
    state.nav_target = state.route[2]

    state.delete_route_point(0)

    assert state.route == [_route_point(2), _route_point(3)]
    assert state.route_index == 1
    assert state.nav_target == _route_point(3)  # untouched -- index 0 < route_index 2


def test_delete_route_point_currently_chased_advances_nav_target():
    state = RobotState()
    state.route = [_route_point(1), _route_point(2), _route_point(3)]
    state.route_index = 1
    state.nav_target = state.route[1]

    state.delete_route_point(1)

    assert state.route == [_route_point(1), _route_point(3)]
    assert state.route_index == 1
    assert state.nav_target == _route_point(3)


def test_delete_last_route_point_while_current_target_empties_route():
    state = RobotState()
    state.route = [_route_point(1)]
    state.route_index = 0
    state.nav_target = state.route[0]

    state.delete_route_point(0)

    assert state.route == []
    assert state.route_index == 0
    # Left as-is (the just-deleted point's own coordinates), same "done"
    # convention _advance_route_if_arrived() already uses once a route
    # finishes on its own -- see delete_route_point()'s docstring.
    assert state.nav_target == _route_point(1)


def test_delete_route_point_out_of_range_raises():
    state = RobotState()
    state.route = [_route_point(1)]
    with pytest.raises(CommandError) as exc_info:
        state.delete_route_point(5)
    assert exc_info.value.code == "21"


def test_delete_route_point_keeps_return_raw_lines_in_lockstep():
    state = RobotState()
    state.route = [_route_point(1), _route_point(2)]
    state.route_index = 0
    state.nav_target = state.route[0]
    state.route_is_return = True
    state._return_raw_lines = ["line1\n", "line2\n"]

    state.delete_route_point(0)

    assert state._return_raw_lines == ["line2\n"]
    assert state.route == [_route_point(2)]
    assert state.nav_target == _route_point(2)


def test_rtd_sentence_deletes_over_a_real_socket(running_server):
    port = running_server.server_address[1]
    _send_and_receive(
        port,
        build_sentence("RTE", "2", "4723.492", "N", "00044.340", "W", "4724.010", "N", "00044.500", "W"),
    )
    response = _send_and_receive(port, build_sentence("RTD", 0))
    resp_type, fields = parse_sentence(response)
    assert resp_type == "ACK"
    assert fields == ["RTD"]

    grt_response = _send_and_receive(port, build_sentence("GRT"))
    _, grt_fields = parse_sentence(grt_response)
    assert grt_fields[0] == "1"
    assert grt_fields[1:5] == ["4724.010", "N", "00044.500", "W"]


def test_rtd_sentence_rejects_wrong_field_count(running_server):
    port = running_server.server_address[1]
    response = _send_and_receive(port, build_sentence("RTD"))
    resp_type, fields = parse_sentence(response)
    assert resp_type == "ERR"
    assert fields[0] == "10"


def test_rtd_sentence_out_of_range_over_real_socket_returns_err(running_server):
    port = running_server.server_address[1]
    response = _send_and_receive(port, build_sentence("RTD", 0))
    resp_type, fields = parse_sentence(response)
    assert resp_type == "ERR"
    assert fields[0] == "21"


# --- Solar-exposure survey/map (2026-10-07, explicit user request) --------
# update_gps_fix()'s own buffering gate is tested directly against
# RobotState (no real socket needed, same as the RobotState unit tests at
# the top of this file); the DB/wire side (log_solar_survey_batch/
# recompute_solar_map/fetch_solar_map_chunk/SMP) reuses the HIS section's
# own history_server fixture above.

def test_update_gps_fix_does_not_buffer_without_a_power_reading(tmp_path, monkeypatch):
    # power_available defaults to None (no Tracer reading has ever come
    # in) -- must not buffer anything, same gating rule as
    # link.power_history's power_log.
    monkeypatch.setenv("SOLAR_SURVEY_TMP_PATH", str(tmp_path / "solar_survey_tmp.jsonl"))
    state = RobotState()
    state.update_gps_fix(47.391533, -0.739000)
    from link import solar_map
    assert solar_map.drain_points(state._solar_survey_tmp_path) == []


def test_update_gps_fix_does_not_buffer_while_tracer_unavailable(tmp_path, monkeypatch):
    monkeypatch.setenv("SOLAR_SURVEY_TMP_PATH", str(tmp_path / "solar_survey_tmp.jsonl"))
    state = RobotState()
    state.update_power_reading(available=False)
    state.update_gps_fix(47.391533, -0.739000)
    from link import solar_map
    assert solar_map.drain_points(state._solar_survey_tmp_path) == []


def test_update_gps_fix_buffers_first_point_once_tracer_is_available(tmp_path, monkeypatch):
    monkeypatch.setenv("SOLAR_SURVEY_TMP_PATH", str(tmp_path / "solar_survey_tmp.jsonl"))
    state = RobotState()
    state.update_power_reading(available=True, pv_power=42.5)
    state.update_gps_fix(47.391533, -0.739000)
    from link import solar_map
    points = solar_map.drain_points(state._solar_survey_tmp_path)
    assert len(points) == 1
    ts, lat, lon, pv_power = points[0]
    assert (lat, lon, pv_power) == (47.391533, -0.739000, 42.5)


def test_update_gps_fix_does_not_rebuffer_under_5_metres(tmp_path, monkeypatch):
    monkeypatch.setenv("SOLAR_SURVEY_TMP_PATH", str(tmp_path / "solar_survey_tmp.jsonl"))
    state = RobotState()
    state.update_power_reading(available=True, pv_power=10.0)
    state.update_gps_fix(47.391533, -0.739000)
    # ~1m away -- well under SOLAR_SURVEY_MIN_DISTANCE_M (5m).
    state.update_gps_fix(47.391542, -0.739000)
    from link import solar_map
    assert len(solar_map.drain_points(state._solar_survey_tmp_path)) == 1


def test_update_gps_fix_buffers_again_past_5_metres(tmp_path, monkeypatch):
    monkeypatch.setenv("SOLAR_SURVEY_TMP_PATH", str(tmp_path / "solar_survey_tmp.jsonl"))
    state = RobotState()
    state.update_power_reading(available=True, pv_power=10.0)
    state.update_gps_fix(47.391533, -0.739000)
    # ~90m north -- comfortably past the 5m threshold.
    state.update_gps_fix(47.392343, -0.739000)
    from link import solar_map
    assert len(solar_map.drain_points(state._solar_survey_tmp_path)) == 2


# --- Clock-plausibility gate (2026-10-10, explicit user request) ----------
# This Pi has no RTC battery -- right after boot, before NTP has had a
# chance to correct the clock, time.time() can read months/years off (see
# link.power_history.system_clock_is_plausible's own docstring). Patching
# the shared stdlib `time` module's own `time` attribute affects every
# module that did `import time; time.time()` (link.robot_state AND
# link.power_history both do), which is exactly what these tests need.

def test_system_clock_is_plausible_true_for_a_real_timestamp(monkeypatch):
    from link import power_history as ph
    monkeypatch.setattr(time, "time", lambda: 1780000000.0)  # 2026-05-28ish
    assert ph.system_clock_is_plausible() is True


def test_system_clock_is_plausible_false_before_the_floor(monkeypatch):
    from link import power_history as ph
    monkeypatch.setattr(time, "time", lambda: 1000.0)  # 1970-01-01ish
    assert ph.system_clock_is_plausible() is False


def test_update_gps_fix_does_not_buffer_while_clock_is_implausible(tmp_path, monkeypatch):
    monkeypatch.setenv("SOLAR_SURVEY_TMP_PATH", str(tmp_path / "solar_survey_tmp.jsonl"))
    monkeypatch.setattr(time, "time", lambda: 1000.0)
    state = RobotState()
    state.update_power_reading(available=True, pv_power=42.5)
    state.update_gps_fix(47.391533, -0.739000)
    from link import solar_map
    assert solar_map.drain_points(state._solar_survey_tmp_path) == []


def test_update_gps_fix_buffers_again_once_the_clock_recovers(tmp_path, monkeypatch):
    # Also confirms _solar_survey_last_point was NOT updated by the
    # skipped fix above -- a point that lands close to that same skipped
    # position still gets recorded once the clock is plausible again,
    # rather than being treated as "under 5m from the last point".
    monkeypatch.setenv("SOLAR_SURVEY_TMP_PATH", str(tmp_path / "solar_survey_tmp.jsonl"))
    monkeypatch.setattr(time, "time", lambda: 1000.0)
    state = RobotState()
    state.update_power_reading(available=True, pv_power=42.5)
    state.update_gps_fix(47.391533, -0.739000)  # skipped -- clock implausible

    monkeypatch.setattr(time, "time", lambda: 1780000000.0)
    state.update_gps_fix(47.391533, -0.739000)  # same spot, now plausible
    from link import solar_map
    points = solar_map.drain_points(state._solar_survey_tmp_path)
    assert len(points) == 1


def test_power_history_logger_skips_power_log_while_clock_is_implausible(tmp_path, monkeypatch):
    # Deliberately NOT the history_server fixture here -- same race as
    # test_power_history_logger_only_recomputes_solar_map_while_idle
    # above: its PowerHistoryLogger starts a real background thread whose
    # automatic first tick runs with the REAL (plausible) clock, before
    # this test's own monkeypatch.setattr(time, "time", ...) below ever
    # takes effect -- which would write the very row this test asserts
    # never gets written. A manually-constructed, never-started
    # PowerHistoryLogger has nothing ticking except this test's own
    # explicit _log_once() call.
    monkeypatch.setenv("POWER_HISTORY_DB_PATH", str(tmp_path / "power_history.db"))
    from link import power_history as ph

    state = RobotState()
    state.update_power_reading(available=True, pv_voltage=15.82)
    logger = ph.PowerHistoryLogger(state)

    monkeypatch.setattr(time, "time", lambda: 1000.0)
    logger._log_once()

    db_path = ph.resolve_db_path()
    with ph._connect(db_path) as conn:
        count = conn.execute("SELECT COUNT(*) FROM power_log").fetchone()[0]
    assert count == 0


def test_power_history_logger_skips_solar_survey_db_write_while_clock_is_implausible(
    tmp_path, monkeypatch,
):
    # Same reasoning as the test just above for not using history_server.
    # The buffered point is neither written to the DB nor dropped -- it's
    # still sitting in the tmp file, waiting for a tick where the clock
    # looks plausible.
    monkeypatch.setenv("SOLAR_SURVEY_TMP_PATH", str(tmp_path / "solar_survey_tmp.jsonl"))
    monkeypatch.setenv("POWER_HISTORY_DB_PATH", str(tmp_path / "power_history.db"))
    from link import power_history as ph
    from link import solar_map

    state = RobotState()
    logger = ph.PowerHistoryLogger(state)

    solar_map.append_point(solar_map.resolve_tmp_path(), 1000.0, 47.4, -0.74, 15.0)
    monkeypatch.setattr(time, "time", lambda: 1000.0)
    logger._log_once()

    with ph._connect(ph.resolve_db_path()) as conn:
        count = conn.execute("SELECT COUNT(*) FROM solar_survey_raw").fetchone()[0]
    assert count == 0
    remaining = solar_map.drain_points(solar_map.resolve_tmp_path())
    assert len(remaining) == 1


def test_power_history_logger_flushes_solar_survey_even_when_tracer_unavailable(
    history_server, tmp_path, monkeypatch,
):
    # The tmp-buffer flush is unconditional (each line was already gated
    # at write time) -- distinct from power_log's own row, which test_
    # power_history_logger_skips_writes_when_tracer_unavailable above
    # confirms IS still gated on power_status()["available"].
    monkeypatch.setenv("SOLAR_SURVEY_TMP_PATH", str(tmp_path / "solar_survey_tmp.jsonl"))
    from link import power_history as ph
    from link import solar_map

    solar_map.append_point(solar_map.resolve_tmp_path(), time.time(), 47.4, -0.74, 15.0)
    history_server.power_history_logger._log_once()

    total, rows = ph.fetch_solar_map_chunk(ph.resolve_db_path(), 0)
    # mode defaults to "IDLE" (RobotState.__init__), so this same tick
    # also recomputes the grid -- one buffered point makes one cell.
    assert total == 1
    assert rows[0][2] == 15.0  # avg_pv_power
    assert rows[0][3] == 1     # sample_count


def test_power_history_logger_only_recomputes_solar_map_while_idle(tmp_path, monkeypatch):
    # Deliberately NOT the history_server fixture (same reasoning as
    # test_smp_sentence_returns_cells_paginated above): that fixture's
    # PowerHistoryLogger starts its own background thread immediately,
    # which fires an automatic first tick of its own -- a real race
    # against this test's own set_mode("MANUAL")/_log_once() sequence
    # (the automatic tick could land while mode is still the default
    # "IDLE", recomputing the grid before this test ever gets to assert
    # that it shouldn't be). A manually-constructed, never-started
    # PowerHistoryLogger removes that race entirely: nothing ticks except
    # this test's own explicit _log_once() call.
    monkeypatch.setenv("SOLAR_SURVEY_TMP_PATH", str(tmp_path / "solar_survey_tmp.jsonl"))
    monkeypatch.setenv("POWER_HISTORY_DB_PATH", str(tmp_path / "power_history.db"))
    from link import power_history as ph
    from link import solar_map

    state = RobotState()
    state.set_mode("MANUAL")
    logger = ph.PowerHistoryLogger(state)

    solar_map.append_point(solar_map.resolve_tmp_path(), time.time(), 47.4, -0.74, 15.0)
    logger._log_once()

    # The raw sample was still flushed (unconditional)...
    db_path = ph.resolve_db_path()
    with ph._connect(db_path) as conn:
        raw_count = conn.execute("SELECT COUNT(*) FROM solar_survey_raw").fetchone()[0]
    assert raw_count == 1
    # ...but the grid itself was NOT recomputed, since mode != "IDLE".
    total_cells, _ = ph.fetch_solar_map_chunk(ph.resolve_db_path(), 0)
    assert total_cells == 0


def test_recompute_solar_map_drops_cells_whose_raw_rows_were_pruned(history_server):
    from link import power_history as ph

    db_path = ph.resolve_db_path()
    ph.log_solar_survey_batch(db_path, [(1, 47.4, -0.74, 20.0)])
    ph.recompute_solar_map(db_path)
    total, _ = ph.fetch_solar_map_chunk(db_path, 0)
    assert total == 1

    # Prune with a retention window that excludes ts=1 entirely, then
    # recompute again -- the now-empty group must disappear, not linger
    # as a stale cell.
    ph.prune_old_solar_survey_rows(db_path, retention_days=0)
    ph.recompute_solar_map(db_path)
    total_after, _ = ph.fetch_solar_map_chunk(db_path, 0)
    assert total_after == 0


def test_smp_sentence_rejects_wrong_field_count(history_server):
    port = history_server.server_address[1]
    response = _send_and_receive(port, build_sentence("SMP"))
    sentence_type, fields = parse_sentence(response)
    assert sentence_type == "ERR"
    assert fields[0] == "23"


def test_smp_sentence_rejects_bad_offset(history_server):
    port = history_server.server_address[1]
    response = _send_and_receive(port, build_sentence("SMP", "not-a-number"))
    sentence_type, fields = parse_sentence(response)
    assert sentence_type == "ERR"
    assert fields[0] == "24"


def test_smp_sentence_empty_map_returns_zero_rows(history_server):
    port = history_server.server_address[1]
    response = _send_and_receive(port, build_sentence("SMP", 0))
    sentence_type, fields = parse_sentence(response)
    assert sentence_type == "SMP"
    assert fields[:3] == ["0", "0", "0"]


def test_smp_sentence_returns_cells_paginated(tmp_path, monkeypatch):
    # Deliberately `running_server` (start_history=False), not
    # `history_server`: this test seeds solar_survey_raw/solar_map_cells
    # directly, and a live PowerHistoryLogger thread ticking in the
    # background would race its own recompute_solar_map() call against
    # this one against the same database -- the SMP sentence handler
    # itself only ever reads via fetch_solar_map_chunk(), which doesn't
    # need a running logger at all.
    monkeypatch.setenv("POWER_HISTORY_DB_PATH", str(tmp_path / "power_history.db"))
    from link import power_history as ph

    db_path = ph.resolve_db_path()
    # 5 distinct cells, far enough apart (~1 degree) to never collide.
    points = [(i, 47.0 + i, -1.0 - i, float(i) * 10.0) for i in range(5)]
    ph.log_solar_survey_batch(db_path, points)
    ph.recompute_solar_map(db_path)

    server = ControlServer(
        "127.0.0.1", 0,
        start_gps=False, start_motor=False, start_gamepad=False, start_tracer=False,
        start_history=False,
    )
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        port = server.server_address[1]

        # Same "patch the chunk size down" trick as test_his_returns_
        # logged_rows_paginated above, to actually exercise the
        # pagination loop.
        monkeypatch.setattr(ph, "SOLAR_MAP_CHUNK_ROWS", 2)

        collected = []
        offset = 0
        while True:
            response = _send_and_receive(port, build_sentence("SMP", offset))
            sentence_type, fields = parse_sentence(response)
            assert sentence_type == "SMP"
            total_count, resp_offset, returned_count = (
                int(fields[0]), int(fields[1]), int(fields[2]),
            )
            assert total_count == 5
            assert resp_offset == offset
            row_fields = fields[3:]
            assert len(row_fields) == returned_count * 5  # 5 fields/row
            collected.extend(row_fields)
            offset += returned_count
            if returned_count == 0 or offset >= total_count:
                break

        assert len(collected) == 5 * 5
    finally:
        server.shutdown()
        server.server_close()
