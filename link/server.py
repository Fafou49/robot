"""TCP control server for the robot (Raspberry Pi #1): receives NMEA-style
sentences from the web server (Raspberry Pi #2) over the local WiFi
network, one sentence per line, and replies with ACK/ERR/STA sentences.

Run with:
    python3 -m link            # binds 0.0.0.0:5050 by default
    CONTROL_HOST=0.0.0.0 CONTROL_PORT=5050 python3 -m link

Protocol reference: pages/protocole_controle.html in the robot-webserver
project documents every sentence type this dispatch table implements.
"""

import logging
import os
import socketserver

from link.gps_reader import DEFAULT_BAUDRATE, DEFAULT_DEVICE, GPSReader
from link.nmea import SentenceError, build_sentence, decimal_to_nmea, parse_sentence
from link.robot_state import CommandError, RobotState

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
log = logging.getLogger("link.server")

DEFAULT_HOST = "0.0.0.0"
DEFAULT_PORT = 5050


def _handle_sentence(state: RobotState, sentence_type: str, fields: list) -> str:
    """Dispatches one parsed sentence to a RobotState method and returns
    the response sentence (ACK/STA) to send back. Raises CommandError
    (caught by the caller) for invalid input or an unimplemented command."""
    if sentence_type == "STP":
        state.stop()
        return build_sentence("ACK", "STP")

    if sentence_type == "DRV":
        if len(fields) != 2:
            raise CommandError("10", "DRV_NEEDS_2_FIELDS")
        state.drive(*fields)
        return build_sentence("ACK", "DRV")

    if sentence_type == "MOD":
        if len(fields) != 1:
            raise CommandError("10", "MOD_NEEDS_1_FIELD")
        state.set_mode(*fields)
        return build_sentence("ACK", "MOD")

    if sentence_type == "NAV":
        if len(fields) != 4:
            raise CommandError("10", "NAV_NEEDS_4_FIELDS")
        state.set_nav_target(*fields)
        return build_sentence("ACK", "NAV")

    if sentence_type == "PID":
        if len(fields) != 4:
            raise CommandError("10", "PID_NEEDS_4_FIELDS")
        state.set_pid_gains(*fields)
        return build_sentence("ACK", "PID")

    if sentence_type == "CAM":
        if len(fields) != 1:
            raise CommandError("10", "CAM_NEEDS_1_FIELD")
        # SNAP calls out to camera/stream_server.py's /snap endpoint (see
        # RobotState.camera_command) and can genuinely succeed; REC_START/
        # REC_STOP still always raise (not implemented).
        state.camera_command(*fields)
        return build_sentence("ACK", "CAM")

    if sentence_type == "STA":
        s = state.status()
        target = s["nav_target"] or (0.0, "N", 0.0, "E")

        # Current position: real once link.gps_reader.GPSReader has a fix
        # (current_lat/current_lon are no longer None), the honest 0.0
        # placeholder otherwise (no receiver attached, or no fix yet).
        # Same ddmm.mmmm + direction format as NAV/target, built with
        # decimal_to_nmea so lat/lon and their direction letters stay
        # adjacent -- this used to be two separate, far-apart fields
        # (a hardcoded "N"/"E" assumption that was actually wrong for
        # this robot, which operates west of the meridian -- see
        # gps/gps_transfer.py's lon_cible) and has been consolidated here.
        if s["current_lat"] is not None and s["current_lon"] is not None:
            lat_str, lat_dir = decimal_to_nmea(s["current_lat"], is_longitude=False)
            lon_str, lon_dir = decimal_to_nmea(s["current_lon"], is_longitude=True)
        else:
            lat_str, lat_dir, lon_str, lon_dir = "0.0", "N", "0.0", "E"

        # Fields, in order: current position (lat, lat_dir, lon, lon_dir --
        # matching NAV's own field order), cap (course over ground) and
        # speed (km/h, both from GPRMC once a fix is available, 0.0
        # otherwise), left_pwm, right_pwm, battery (still a placeholder --
        # no battery sensor in this project), mode, then target position
        # (target_lat, target_lat_dir, target_lon, target_lon_dir -- last
        # waypoint received via NAV, if any).
        return build_sentence(
            "STA",
            lat_str, lat_dir, lon_str, lon_dir,
            round(s["cap"], 1), round(s["speed_kmh"], 2),
            s["left_pwm"], s["right_pwm"], 0, s["mode"],
            target[0], target[1], target[2], target[3],
        )

    raise CommandError("11", f"UNKNOWN_SENTENCE_TYPE:{sentence_type}")


class ControlHandler(socketserver.StreamRequestHandler):
    def handle(self):
        peer = self.client_address[0]
        log.info("connection from %s", peer)
        for raw_line in self.rfile:
            line = raw_line.decode("ascii", errors="replace").strip()
            if not line:
                continue

            try:
                sentence_type, fields = parse_sentence(line)
            except SentenceError as exc:
                log.warning("malformed sentence from %s: %s (%s)", peer, line, exc)
                response = build_sentence("ERR", "00", str(exc).replace(",", ";"))
                self._reply(response)
                continue

            try:
                response = _handle_sentence(self.server.state, sentence_type, fields)
                log.info("%s -> %s -> %s", peer, line, response)
            except CommandError as exc:
                response = build_sentence("ERR", exc.code, exc.message.replace(",", ";"))
                log.info("%s -> %s -> %s", peer, line, response)

            self._reply(response)

    def _reply(self, sentence: str):
        self.wfile.write((sentence + "\r\n").encode("ascii"))


class ControlServer(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True

    def __init__(self, host, port, state: RobotState = None, start_gps: bool = True):
        super().__init__((host, port), ControlHandler)
        self.state = state or RobotState()
        self.gps_reader = None
        if start_gps:
            # GPS_ENABLED=false disables this entirely -- useful if
            # gps/gps_transfer.py or gps/gps_parse.py is already holding
            # the serial port (only one process can own it at a time).
            if os.environ.get("GPS_ENABLED", "true").lower() not in ("false", "0", "no"):
                self.gps_reader = GPSReader(
                    self.state,
                    device=os.environ.get("GPS_DEVICE", DEFAULT_DEVICE),
                    baudrate=int(os.environ.get("GPS_BAUDRATE", DEFAULT_BAUDRATE)),
                )
                self.gps_reader.start()


def main():
    host = os.environ.get("CONTROL_HOST", DEFAULT_HOST)
    port = int(os.environ.get("CONTROL_PORT", DEFAULT_PORT))
    server = ControlServer(host, port)
    log.info("control server listening on %s:%s", host, port)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        log.info("shutting down")
        server.shutdown()


if __name__ == "__main__":
    main()
