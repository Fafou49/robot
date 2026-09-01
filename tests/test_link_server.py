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

from link.nmea import build_sentence, parse_sentence
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


def test_camera_command_not_implemented_yet():
    state = RobotState()
    with pytest.raises(CommandError) as exc_info:
        state.camera_command("SNAP")
    assert "NOT_IMPLEMENTED" in exc_info.value.message


# --- End-to-end socket tests ------------------------------------------------

@pytest.fixture()
def running_server():
    server = ControlServer("127.0.0.1", 0)  # port 0 = pick a free port
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
