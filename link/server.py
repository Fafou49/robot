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
import shlex
import socketserver
import subprocess

from link.gamepad_handler import GamepadReader, robot_state_button_handler, robot_state_drive_handler
from link.gps_reader import DEFAULT_BAUDRATE, DEFAULT_DEVICE, GPSReader
from link.nmea import SentenceError, build_sentence, decimal_to_nmea, parse_sentence
from link.robot_state import CommandError, RobotState
from motor_control.motor_driver import MotorDriver

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
log = logging.getLogger("link.server")

DEFAULT_HOST = "0.0.0.0"
DEFAULT_PORT = 5050

# What BTN_START on the gamepad runs to power off the Raspberry Pi (see
# ControlServer._shutdown_pi() below and link/gamepad_handler.py's
# robot_state_button_handler()). Overridable via SHUTDOWN_CMD for testing
# or a different init system; whatever user runs run_robot.sh needs
# passwordless sudo for exactly this command -- see this project's
# README ("Pilotage moteur et manette") for the one-time sudoers setup.
SHUTDOWN_CMD = shlex.split(os.environ.get("SHUTDOWN_CMD", "sudo poweroff"))

# Which gamepad button does what, by evdev.ecodes attribute name -- see
# link/gamepad_handler.py's robot_state_button_handler() docstring for
# the full rationale (2026-09-12, extended 2026-09-18) and for why these
# are overridable at all: this project's actual controller/receiver may
# report a physical button under a different evdev code than its label
# suggests (a field report that pressing "Y" behaves like "A" -- see that
# module's "BUTTON MAPPING" comment). Run `python3 -m
# motor_control.dump_gamepad_buttons` on the Pi, press each physical
# button in turn, and set whichever variable below (in .env, see
# .env.example) doesn't match its default if the real code differs.
#
# 2026-09-18 right-cluster remap: A arms AUTO (was Y), B toggles video
# recording (new), X saves the current GPS fix to a waypoints file (new),
# Y takes a camera snapshot (new). GAMEPAD_STOP_BTN is unset (None) by
# default -- B used to be a dedicated full-stop button, but now records
# video instead; see robot_state_button_handler()'s docstring for the
# safety reasoning and how to rebind a stop button (e.g. to a shoulder
# button/bumper) if wanted.
GAMEPAD_ARM_AUTO_BTN = os.environ.get("GAMEPAD_ARM_AUTO_BTN", "BTN_A")
GAMEPAD_RECORD_BTN = os.environ.get("GAMEPAD_RECORD_BTN", "BTN_B")
GAMEPAD_SAVE_WAYPOINT_BTN = os.environ.get("GAMEPAD_SAVE_WAYPOINT_BTN", "BTN_X")
GAMEPAD_SNAPSHOT_BTN = os.environ.get("GAMEPAD_SNAPSHOT_BTN", "BTN_Y")
GAMEPAD_SHUTDOWN_BTN = os.environ.get("GAMEPAD_SHUTDOWN_BTN", "BTN_START")
GAMEPAD_STOP_BTN = os.environ.get("GAMEPAD_STOP_BTN") or None

# How long the gamepad buzzes when the live GPS fix's quality changes
# (see ControlServer._on_gps_quality_change() and
# link.gamepad_handler.GamepadReader.pulse()) -- strong for
# DGPS-acquired, weak for DGPS-lost, both this same duration. 0.5s is
# long enough to clearly notice, short enough to stay a "ping" rather
# than a "the controller is stuck buzzing".
DGPS_PULSE_DURATION_S = 0.5


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
        # A manual DRV always asserts direct control -- same "manual
        # override always wins" convention NAV/STP already use for the
        # route (see link/robot_state.py). Set BEFORE drive() so an
        # autonomous tick racing on another thread (link.gps_reader's
        # background thread, see RobotState.update_gps_fix) can't slot in
        # between the mode switch and the requested pwm actually landing.
        state.set_mode("MANUAL")
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

    if sentence_type == "RTE":
        # Field count isn't fixed here (depends on how many waypoints the
        # route has) -- state.set_route() validates the leading count field
        # against the rest and raises CommandError("13", ...) for anything
        # inconsistent, so there's nothing more to check before calling it.
        if len(fields) < 1:
            raise CommandError("10", "RTE_NEEDS_COUNT_AND_POINTS")
        state.set_route(fields)
        return build_sentence("ACK", "RTE")

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

    if sentence_type == "WPT":
        # Query, no fields: returns every waypoint saved so far (gamepad's
        # X button, see RobotState.save_waypoint()/list_waypoints()) for
        # robot-webserver's /control map (2026-09-19) -- its blue markers.
        # Same "count then count*4 fields" shape as RTE's own request, so
        # the website can reuse one parser for both; unlike the route,
        # decimal degrees straight out of the waypoints file need
        # converting to this protocol's ddmm.mmmm+direction pairs first.
        points = state.list_waypoints()
        fields = [len(points)]
        for lat, lon in points:
            lat_str, lat_dir = decimal_to_nmea(lat, is_longitude=False)
            lon_str, lon_dir = decimal_to_nmea(lon, is_longitude=True)
            fields.extend([lat_str, lat_dir, lon_str, lon_dir])
        return build_sentence("WPT", *fields)

    if sentence_type == "GRT":
        # Query, no fields: returns the currently active route (the last
        # RTE upload, i.e. "GPS Driving" -- see RobotState.set_route()) for
        # robot-webserver's /control map (2026-09-19) -- its yellow
        # markers. self.route already holds each point pre-encoded as
        # (lat, lat_dir, lon, lon_dir) exactly like RTE's own fields, so
        # this just flattens it back out -- no further conversion needed.
        # Empty (count 0, no point fields) once no route has been sent yet
        # or after STP/a fresh NAV cleared it (see set_nav_target()/stop()).
        route = state.get_route()
        fields = [len(route)]
        for point in route:
            fields.extend(point)
        return build_sentence("GRT", *fields)

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

        # DGPS fix-quality field (2026-09-19, extend-only -- see
        # RobotState.status()'s own comments anticipating this): "DGPS"
        # once a GGA sentence has reported a differentially-corrected fix
        # (link/gps_reader.py's DGPS_QUALITY), "GPS" once one has reported
        # a fix that isn't DGPS-corrected, "UNKNOWN" before any GGA
        # quality has been seen at all (no receiver attached, RMC-only
        # sentences so far, or no fix yet) -- three states, not a bool,
        # matching RobotState.is_dgps's own None/True/False tri-state.
        if s["is_dgps"] is None:
            dgps_field = "UNKNOWN"
        elif s["is_dgps"]:
            dgps_field = "DGPS"
        else:
            dgps_field = "GPS"

        # Fields, in order: current position (lat, lat_dir, lon, lon_dir --
        # matching NAV's own field order), cap (course over ground) and
        # speed (km/h, both from GPRMC once a fix is available, 0.0
        # otherwise), left_pwm, right_pwm, battery (still a placeholder --
        # no battery sensor in this project), mode, then target position
        # (target_lat, target_lat_dir, target_lon, target_lon_dir -- last
        # waypoint received via NAV, if any), then the DGPS field above
        # (new trailing field, extend-only).
        return build_sentence(
            "STA",
            lat_str, lat_dir, lon_str, lon_dir,
            round(s["cap"], 1), round(s["speed_kmh"], 2),
            s["left_pwm"], s["right_pwm"], 0, s["mode"],
            target[0], target[1], target[2], target[3],
            dgps_field,
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

    def __init__(self, host, port, state: RobotState = None, start_gps: bool = True,
                 start_motor: bool = True, start_gamepad: bool = True):
        super().__init__((host, port), ControlHandler)

        # MotorDriver is always constructed (cheap -- no hardware is
        # touched until .start()) so it can be wired into RobotState right
        # away; whether it actually opens the GPIO chip is controlled by
        # start_motor/MOTOR_ENABLED below. If an existing `state` is
        # passed in (not done anywhere in this repo today, but the
        # parameter has always allowed it) it's used as-is and this
        # MotorDriver is simply never linked to it -- same "caller owns
        # what they passed in" behavior as before this change.
        self.motor_driver = MotorDriver()
        self.state = state or RobotState(motor_driver=self.motor_driver)

        # Set BEFORE the GPS reader is constructed/started (even though
        # the gamepad itself is only actually created further down) --
        # GPSReader runs its own background thread and can call
        # self._on_gps_quality_change() (wired in as on_gps_quality=
        # below) the moment a fix arrives, which could in principle race
        # ahead of the gamepad block below on a very fast first fix.
        # self.gamepad_reader existing as None from the start (rather
        # than not existing as an attribute at all until later) means
        # that callback's own `if self.gamepad_reader is not None` guard
        # is always safe to evaluate, whichever order things actually
        # finish in.
        self.gamepad_reader = None

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
                    on_gps_quality=self._on_gps_quality_change,
                )
                self.gps_reader.start()

        if start_motor:
            # MOTOR_ENABLED=false skips opening the GPIO chip entirely --
            # useful on a dev machine, or if some other process
            # (motor_control/remote_control.py, run standalone) already
            # owns the motor lines. drive()/stop()/set_mode() stay safe
            # to call either way (see RobotState's motor_driver docs);
            # they just have no physical effect when this is off.
            if os.environ.get("MOTOR_ENABLED", "true").lower() not in ("false", "0", "no"):
                self.motor_driver.start()

        # The gamepad is a second input source feeding the SAME
        # RobotState as the TCP commands above (see link/gamepad_handler.
        # py's module docstring) -- a physical operator's joystick/button
        # presses and the website's STP/DRV/MOD/NAV/RTE commands are
        # peers, both just calling RobotState methods. (self.gamepad_reader
        # was already initialized to None above, before the GPS reader.)
        if start_gamepad:
            # GAMEPAD_ENABLED=false disables this entirely -- useful on a
            # dev machine with no controller plugged in (though this
            # degrades gracefully even when left on, see GamepadReader).
            if os.environ.get("GAMEPAD_ENABLED", "true").lower() not in ("false", "0", "no"):
                self.gamepad_reader = GamepadReader(
                    on_drive=robot_state_drive_handler(self.state),
                    on_button=robot_state_button_handler(
                        self.state,
                        on_shutdown=self._shutdown_pi,
                        arm_auto_btn=GAMEPAD_ARM_AUTO_BTN,
                        record_btn=GAMEPAD_RECORD_BTN,
                        save_waypoint_btn=GAMEPAD_SAVE_WAYPOINT_BTN,
                        snapshot_btn=GAMEPAD_SNAPSHOT_BTN,
                        shutdown_btn=GAMEPAD_SHUTDOWN_BTN,
                        stop_btn=GAMEPAD_STOP_BTN,
                    ),
                )
                self.gamepad_reader.start()

    def _on_gps_quality_change(self, is_dgps):
        """Wired in as link.gps_reader.GPSReader's on_gps_quality callback
        above -- called on the GPS background thread the instant the live
        fix's quality genuinely flips between DGPS-corrected and not (see
        GPSReader's own docstring for the exact edge-triggering rule).
        Physically confirms it to whoever's holding the controller: a
        strong DGPS_PULSE_DURATION_S pulse on gaining DGPS, a weak one of
        the same duration on losing it -- see
        link.gamepad_handler.GamepadReader.pulse().

        Guarded by `is not None` rather than assuming the gamepad is
        always present: GAMEPAD_ENABLED=false, no controller plugged in
        (still degrades gracefully -- pulse() itself is also a safe
        no-op with nothing connected, this guard just avoids calling a
        method on a None reader entirely when the whole feature is off),
        or -- start_gps=True/start_gamepad=False in a test/dev run -- the
        GPS reader firing before the gamepad block above ever runs (see
        the comment where self.gamepad_reader is first set to None)."""
        if self.gamepad_reader is not None:
            self.gamepad_reader.pulse(strong=is_dgps, duration_s=DGPS_PULSE_DURATION_S)

    def _shutdown_pi(self):
        """Called by the gamepad's BTN_START handler (see
        link/gamepad_handler.py's robot_state_button_handler(), which
        already stops the motors before calling this): powers off the
        whole Raspberry Pi, not just this process. Runs on the
        GamepadReader background thread -- a different thread from
        whichever one is running serve_forever() below, which is exactly
        the situation self.shutdown() is documented to require (calling
        it from the SAME thread that runs serve_forever() would deadlock).

        Requires passwordless sudo for SHUTDOWN_CMD (default `sudo
        poweroff`) for whichever user runs run_robot.sh -- see this
        project's README for the one-time sudoers setup. If that's
        missing, or the command otherwise fails, this logs an error and
        still stops this robot's own scripts (the `finally` below) rather
        than leaving the control server running silently as if nothing
        was wrong.

        Order matters: the OS poweroff is requested FIRST, then this
        process asks its own serve_forever() loop to stop. Once
        self.shutdown() returns, main() falls through and the process
        exits on its own (MotorDriver's atexit hook releases the GPIO
        chip) -- and since self.gamepad_reader.start() runs this as a
        daemon thread, doing the (slower, more important) OS-level call
        first means it isn't at risk of being cut short if the interpreter
        starts tearing down daemon threads once main() returns."""
        log.warning(
            "BTN_START pressed on the gamepad -- powering off the Raspberry "
            "Pi (%s) and stopping this robot's own control scripts.",
            " ".join(SHUTDOWN_CMD),
        )
        try:
            subprocess.run(SHUTDOWN_CMD, check=True, timeout=10)
        except (OSError, subprocess.CalledProcessError, subprocess.TimeoutExpired) as exc:
            log.error(
                "could not power off the Raspberry Pi (%s) -- check the "
                "passwordless-sudo setup in README.md ('Pilotage moteur et "
                "manette'). Stopping this robot's own scripts anyway.",
                exc,
            )
        finally:
            self.shutdown()


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
