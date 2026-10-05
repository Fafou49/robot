"""Live MJPEG camera stream for Raspberry Pi #1 (robot side).

Captures frames from a USB webcam via OpenCV and serves them over plain
HTTP as a multipart/x-mixed-replace stream -- the "MJPEG over HTTP" trick
that every browser already knows how to display through a plain <img> tag,
no plugin and no WebRTC signaling needed.

This is intentionally independent from the NMEA control link
(link/server.py): CAM,SNAP (and, since 2026-09-18, CAM,REC_START/
REC_STOP) is handled there by making a plain HTTP request to this
process's own /snap (or /rec/start, /rec/stop) endpoint below, rather
than the two talking over the NMEA link itself -- see
link/robot_state.py's camera_command().

UPDATE (2026-09-18): REC_START/REC_STOP are now genuinely implemented --
see camera/recordings.py's VideoRecorder. FrameGrabber now optionally
feeds every raw frame it captures to a VideoRecorder (in addition to
JPEG-encoding it for the live stream, as before) whenever one is armed
via /rec/start; /rec/stop closes the file. This used to be entirely
unimplemented, with CAM,REC_START/REC_STOP always answering
CAM_NOT_IMPLEMENTED on the NMEA link.

The web server (Raspberry Pi #2, robot-webserver repo) does not load this
stream directly in the browser -- it proxies it from its own /media/camera
route (see app.py there), so the feed stays behind the site's login.

Run with:
    python3 -m camera

Configuration (environment variables, all optional -- read from a local
.env file at repo root if present, see .env.example, or set on the
command line, e.g. `CAMERA_DEVICE=/dev/video2 python3 -m camera`; an
explicit command-line value always wins over .env):
    CAMERA_DEVICE   video device index or path (default: 0, i.e. /dev/video0)
    CAMERA_WIDTH    capture width in pixels (default: 640)
    CAMERA_HEIGHT   capture height in pixels (default: 480)
    CAMERA_FPS      target capture/stream rate (default: 15)
    CAMERA_HOST     interface to listen on (default: 0.0.0.0)
    CAMERA_PORT     port to listen on (default: 8000)
    CAMERA_SNAPSHOT_DIR  where GET /snap saves files (default: camera/tmp/,
                    see camera/snapshots.py -- never more than 5 at once)
    CAMERA_RECORDING_DIR where GET /rec/start-initiated recordings are
                    saved (default: camera/recordings/, see
                    camera/recordings.py -- never more than 5 at once)
    CAMERA_ON_DEMAND  "true" (default) releases the camera device whenever
                    no viewer is connected to /stream.mjpg and no
                    recording is armed, to save power; "false" keeps the
                    previous always-on behavior (see the 2026-10-03 UPDATE
                    above)

UPDATE (2026-10-03): the capture device is now opened only while at least
one viewer is actually connected to /stream.mjpg (or a recording is
armed, see below) -- CAMERA_ON_DEMAND below. Unlike GPS (which keeps
transmitting over its serial link regardless of whether the Pi reads it,
so pausing the read loop saves no real power), a UVC webcam only draws
its full streaming current while the host has actually told it to start
streaming (cv2.VideoCapture open + reading). Releasing the device
(cv2.VideoCapture.release(), not just skipping read() calls -- merely not
calling read() does NOT stop the device's own USB streaming) when nobody
is watching the live feed is expected to meaningfully cut power on a
battery-powered outdoor robot where the feed often isn't being watched.
See FrameGrabber._should_capture() for the exact rule (a viewer connected
on /stream.mjpg, OR a recording currently armed -- REC_START must keep
working even if nobody has the live feed open in a browser). Set
CAMERA_ON_DEMAND=false to go back to the previous always-on behavior.
GET /snap and /rec/start (typically triggered from the gamepad, not from
someone also watching /stream.mjpg in a browser -- see
link/robot_state.py's camera_command()) wake a released device up and
wait up to CAMERA_WAKE_TIMEOUT_S for a first frame before answering --
see FrameGrabber.wait_for_frame() -- rather than seeing "device released"
and immediately answering 503 the way a bare latest_jpeg() check would.
Still answers that same 503 NO_FRAME_YET if no frame arrives within that
budget (no camera plugged in at all, or one too slow to open in time).
Like the rest of this module, NOT verified against a real
webcam's actual power draw in this environment -- confirm with a current
meter on the Pi before relying on the savings.

NOTE: written and reviewed against the OpenCV/http.server APIs, but not
run against a real webcam in this environment -- test on the Pi with the
actual camera plugged in before relying on it.

Diagnostics: watch this script's own console output when the live feed
doesn't show up on /control -- it now distinguishes "device won't open at
all" (a repeating message every few seconds while it keeps retrying --
see CAMERA_RETRY_INTERVAL below; no camera plugged in at all is the most
common real-world cause and is NOT fatal, see the note just below) from
"device opened fine but never delivers a frame" (a repeating message every
few seconds naming the likely causes: unsupported resolution/FPS, wrong
/dev/videoN node, or another process holding the camera) from "camera OK,
first frame captured" (one-line confirmation once frames start flowing).

UPDATE (2026-09-19): added GET /snapshots and GET /recordings (JSON
listings, newest first) and GET /snapshots/<filename> and
/recordings/<filename> (raw file bytes) so the web server's new "Media"
page can list and display the CAM,SNAP snapshot buffer and the
CAM,REC_START/REC_STOP recording buffer -- neither was reachable over
HTTP before this, only the live /stream.mjpg feed and the action-only
/snap, /rec/start, /rec/stop endpoints. See StreamHandler._handle_list()
and ._handle_file().

No camera is not a startup crash: `main()` used to construct FrameGrabber
synchronously and let it raise RuntimeError the moment `cv2.VideoCapture`
failed to open, which killed the whole process (and, via run_robot.sh,
used to take link/server.py down with it -- see that script's own history)
before the HTTP server even started. FrameGrabber now opens the device
lazily in its own background thread and retries every
CAMERA_RETRY_INTERVAL seconds instead of raising, so `python3 -m camera`
starts and stays up with no camera attached at all: /stream.mjpg opens the
connection and simply waits (same as the existing "opened but no frame
yet" case), /snap answers HTTP 503 (see StreamHandler._handle_snap), and
the feed picks up on its own as soon as a camera is plugged in.
"""
import json
import os
import shutil
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
from dotenv import load_dotenv

from camera.recordings import VideoRecorder
from camera.snapshots import SnapshotStore

# Load CAMERA_* settings from a local .env file (see .env.example at repo
# root) so CAMERA_DEVICE doesn't need to be retyped on every launch -- same
# pattern as gps/dgps_transfer.py. An explicit environment variable (e.g.
# `CAMERA_DEVICE=/dev/video2 python3 -m camera`) still overrides .env,
# since load_dotenv() never replaces a variable that's already set.
load_dotenv()

CAMERA_DEVICE = os.environ.get("CAMERA_DEVICE", "0")
CAMERA_WIDTH = int(os.environ.get("CAMERA_WIDTH", "640"))
CAMERA_HEIGHT = int(os.environ.get("CAMERA_HEIGHT", "480"))
CAMERA_FPS = float(os.environ.get("CAMERA_FPS", "15"))
CAMERA_HOST = os.environ.get("CAMERA_HOST", "0.0.0.0")
CAMERA_PORT = int(os.environ.get("CAMERA_PORT", "8000"))
CAMERA_ON_DEMAND = os.environ.get("CAMERA_ON_DEMAND", "true").lower() not in ("false", "0", "no")
# How long /snap and /rec/start wait for a released (CAMERA_ON_DEMAND),
# on-demand camera to wake up and deliver a first frame -- see
# FrameGrabber.wait_for_frame(). Must stay safely under link/
# robot_state.py's own CAMERA_SNAP_TIMEOUT (3s as of this writing), the
# client-side timeout the gamepad/website's CAM,SNAP and CAM,REC_START
# calls use to reach these same two endpoints: this budget has to leave
# that caller enough time to actually receive our response (success or a
# clean 503) rather than timing out on its own socket first, which would
# surface as an unhelpful network error instead of the real reason.
CAMERA_WAKE_TIMEOUT_S = float(os.environ.get("CAMERA_WAKE_TIMEOUT_S", "2.0"))

JPEG_QUALITY = 80  # 0-100, trade-off between bandwidth and image quality


# How often (seconds) to re-print the "opened but no frame yet" diagnostic
# below while it's still true, so it's impossible to miss in the terminal
# but doesn't spam it either.
NO_FRAME_DIAGNOSTIC_INTERVAL = 5.0

# How often (seconds) FrameGrabber retries opening the camera device while
# it isn't available yet (no webcam plugged in, wrong CAMERA_DEVICE...).
# Same idea as link/gamepad_handler.py's own retry_interval for a missing
# controller -- a device that isn't there yet is a normal, recoverable
# state, not a reason to crash the process.
CAMERA_RETRY_INTERVAL = 5.0


class FrameGrabber:
    """Continuously reads frames from the webcam in a background thread and
    keeps only the latest one, already JPEG-encoded. Running the capture
    loop independently from the HTTP handler means a slow or disconnected
    viewer never blocks the camera read loop, and several viewers (or a
    viewer that reconnects) can all share the same encoded frame.

    Diagnostics: `cv2.VideoCapture.isOpened()` can return True (so
    _open_capture() below reports success) even when the camera never
    actually delivers a usable frame afterwards -- e.g. CAMERA_DEVICE
    pointing at a UVC metadata-only node instead of the real capture node,
    or a resolution/FPS combination the camera doesn't support. Without
    extra logging that failure mode is silent: /stream.mjpg just never
    sends anything and /control quietly falls back to the recorded
    playlist, with nothing in the terminal to say why. `_loop()` below
    prints once when the first real frame comes in, and periodically while
    none has, so "opens fine but no image", "device won't open at all",
    and "camera OK" are all distinguishable from the console output.

    The device is opened lazily, inside `_loop()` (the background thread
    started by `start()`), and retried every `retry_interval` seconds
    while it isn't available -- NOT in __init__/eagerly, so a camera that
    is missing or not yet plugged in never raises out of the constructor.
    It used to: main() called this constructor synchronously and let that
    RuntimeError kill the whole `python3 -m camera` process before the
    HTTP server even started (and, via run_robot.sh, link/server.py along
    with it). A missing camera is a normal, recoverable state -- same
    reasoning as link/gamepad_handler.py's own retry loop for a
    disconnected controller -- not a reason to crash."""

    def __init__(self, device, width, height, fps, retry_interval=CAMERA_RETRY_INTERVAL,
                 recorder=None, on_demand=True):
        # device is usually a numeric index ("0") but OpenCV also accepts a
        # path like "/dev/video2" -- try int first, fall back to the string.
        try:
            device = int(device)
        except ValueError:
            pass
        self.device = device
        self.requested_width = width
        self.requested_height = height
        self.requested_fps = fps
        self.retry_interval = retry_interval
        # Optional camera.recordings.VideoRecorder (2026-09-18) -- when
        # given, every raw frame this loop reads is also handed to it via
        # .write(frame), in addition to being JPEG-encoded for the live
        # stream below. VideoRecorder.write() is itself a no-op unless
        # armed (see that class), so this costs nothing when nobody has
        # called /rec/start. None (the default) keeps every existing
        # caller/test that doesn't care about recording unaffected.
        self.recorder = recorder

        # on_demand (2026-10-03, CAMERA_ON_DEMAND above): when True, _loop()
        # only opens/reads the device while _should_capture() is True (a
        # live viewer connected, or a recording armed) and releases it
        # otherwise. When False, _should_capture() always returns True --
        # the previous always-on behavior, kept as an easy rollback.
        self.on_demand = on_demand
        self._viewer_count = 0

        self.frame_interval = 1.0 / fps if fps > 0 else 0.05
        self._capture = None
        self._lock = threading.Lock()
        self._latest_jpeg = None
        self._running = False

    def start(self):
        self._running = True
        threading.Thread(target=self._loop, daemon=True).start()

    def add_viewer(self):
        """Called by StreamHandler when a /stream.mjpg client connects.
        While on_demand is True, the capture loop only actually opens the
        camera device while this count is above zero (or a recording is
        armed -- see _should_capture()) -- see this file's module
        docstring (2026-10-03 UPDATE) for why that saves real power,
        unlike the equivalent idea for the GPS serial link."""
        with self._lock:
            self._viewer_count += 1

    def remove_viewer(self):
        """Counterpart to add_viewer(), called once a /stream.mjpg client
        disconnects. Clamped at 0 so a stray extra call (there shouldn't
        be one, but StreamHandler's own exception handling makes an
        exactly-once call hard to guarantee absolutely) can never make a
        still-connected viewer look like zero."""
        with self._lock:
            self._viewer_count = max(0, self._viewer_count - 1)

    def _should_capture(self):
        """True while the camera device should be open and actively
        read: always True when on_demand is disabled (previous
        behavior); otherwise True only while at least one /stream.mjpg
        viewer is connected OR a recording is currently armed (REC_START
        must keep producing a video even if nobody has the live feed
        open in a browser at the time -- see this file's module
        docstring). False tells _loop() to release the device instead of
        reading from it."""
        if not self.on_demand:
            return True
        with self._lock:
            has_viewer = self._viewer_count > 0
        return has_viewer or (self.recorder is not None and self.recorder.is_recording)

    def wait_for_frame(self, timeout):
        """Used by StreamHandler._handle_snap() and ._handle_rec_start()
        (2026-10-03) instead of a plain latest_jpeg() check, so a gamepad
        button press (CAM,SNAP / CAM,REC_START, arriving here with nobody
        watching /stream.mjpg at all -- the normal case for a field robot
        driven by gamepad, not by someone also having the website's
        /control page open) actually wakes a released, on-demand camera
        up rather than immediately answering 503 NO_FRAME_YET the way a
        bare latest_jpeg() would (a released device is indistinguishable
        from "no camera plugged in" to that check alone).

        Counts as a transient viewer for the duration of the wait --
        add_viewer()/remove_viewer(), the same mechanism a real
        /stream.mjpg connection uses -- which is enough on its own to
        force _should_capture() True and let _loop() open the device,
        regardless of whether a recording ends up getting armed
        afterwards. Returns the first available frame, or None if none
        arrived within `timeout` seconds (a real "no camera" -- or a
        camera too slow to open in time; see CAMERA_WAKE_TIMEOUT_S above
        for why this budget is kept short). A frame already available
        (an existing viewer already has the device open) returns
        immediately, no transient wake needed."""
        frame = self.latest_jpeg()
        if frame is not None:
            return frame

        self.add_viewer()
        try:
            deadline = time.time() + timeout
            while time.time() < deadline:
                frame = self.latest_jpeg()
                if frame is not None:
                    return frame
                time.sleep(0.05)
            return None
        finally:
            self.remove_viewer()

    def _open_capture(self):
        """One attempt at opening self.device. Returns True and leaves
        self._capture set on success; returns False (never raises) on
        failure, so the caller can just retry later."""
        capture = cv2.VideoCapture(self.device)
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.requested_width)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.requested_height)
        capture.set(cv2.CAP_PROP_FPS, self.requested_fps)

        if not capture.isOpened():
            capture.release()
            return False

        # .set() above can silently fail to apply (common when the camera
        # doesn't support the exact requested mode) -- .get() reports what
        # was actually negotiated, which is worth knowing up front rather
        # than only discovering it once frames fail to show up.
        actual_width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = capture.get(cv2.CAP_PROP_FPS)
        print(f"Camera device {self.device!r} opened (isOpened() = True). "
              f"Requested {self.requested_width}x{self.requested_height}@{self.requested_fps}fps, "
              f"camera reports {actual_width}x{actual_height}@{actual_fps:g}fps.")
        if (actual_width, actual_height) != (self.requested_width, self.requested_height):
            print("  Note: reported resolution differs from what was requested -- "
                  "this camera may not support the exact mode asked for; usually "
                  "harmless, but worth knowing if the stream never starts below.")

        self._capture = capture
        return True

    def _loop(self):
        got_first_frame = False
        consecutive_failures = 0
        last_diagnostic_at = 0.0
        last_open_attempt_at = 0.0
        started_at = time.time()

        while self._running:
            if not self._should_capture():
                # 2026-10-03: nobody is watching /stream.mjpg and no
                # recording is armed -- release the device instead of
                # reading from it, so the webcam actually stops streaming
                # (see this file's module docstring for why just skipping
                # read() calls below would NOT save power: the device
                # keeps streaming over USB until the capture is actually
                # released). last_open_attempt_at is deliberately left
                # alone here -- reset to 0.0 below so the very next time
                # capture is needed, _open_capture() is tried immediately
                # rather than waiting out whatever was left of
                # retry_interval from the last attempt before this one.
                if self._capture is not None:
                    self._capture.release()
                    self._capture = None
                    with self._lock:
                        self._latest_jpeg = None
                    got_first_frame = False
                    last_open_attempt_at = 0.0
                    print(
                        f"Camera device {self.device!r} released -- no viewer "
                        f"connected and no recording in progress. Reconnecting "
                        f"to /stream.mjpg (or starting a recording) re-opens it "
                        f"automatically; set CAMERA_ON_DEMAND=false to disable "
                        f"this and keep the camera always on."
                    )
                time.sleep(0.2)
                continue

            if self._capture is None:
                now = time.time()
                if now - last_open_attempt_at < self.retry_interval:
                    time.sleep(0.1)
                    continue
                last_open_attempt_at = now
                if not self._open_capture():
                    print(
                        f"Camera device {self.device!r} not available (could not open) -- "
                        f"retrying every {self.retry_interval:g}s. The live feed stays "
                        f"unavailable (/snap returns HTTP 503) until a camera is plugged "
                        f"in and detected; this no longer crashes the process."
                    )
                    continue
                started_at = time.time()  # restart the "no frame yet" clock from the actual open

            try:
                ok, frame = self._capture.read()
            except Exception as exc:
                # Same failure class as the remote_control.py PWM-thread bug
                # found earlier in this project: an uncaught exception in a
                # background thread just prints a traceback and kills the
                # thread silently, with /control simply never showing the
                # feed and no obvious reason why. Caught explicitly so it's
                # visible and the loop keeps retrying instead of dying here.
                print(f"Camera read() raised {exc!r} -- retrying.")
                ok, frame = False, None

            if ok:
                consecutive_failures = 0
                if self.recorder is not None:
                    # Raw frame, before JPEG encoding below -- a no-op
                    # unless /rec/start has armed the recorder (see
                    # camera/recordings.py's VideoRecorder.write()).
                    self.recorder.write(frame)
                ok, buffer = cv2.imencode(
                    ".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY]
                )
                if ok:
                    with self._lock:
                        self._latest_jpeg = buffer.tobytes()
                    if not got_first_frame:
                        got_first_frame = True
                        h, w = frame.shape[:2]
                        print(f"Camera OK: first frame captured ({w}x{h}) "
                              f"after {time.time() - started_at:.1f}s -- "
                              f"the live feed should now appear on /control.")
            else:
                consecutive_failures += 1
                now = time.time()
                if not got_first_frame and now - last_diagnostic_at >= NO_FRAME_DIAGNOSTIC_INTERVAL:
                    last_diagnostic_at = now
                    print(
                        f"Camera device {self.device!r} opened successfully "
                        f"(isOpened() was True) but read() has failed "
                        f"{consecutive_failures} time(s) in a row and no frame "
                        f"has been captured yet, {now - started_at:.0f}s after "
                        f"startup. The device opening is NOT the problem here -- "
                        f"likely causes: (1) the requested capture mode "
                        f"({self.requested_width}x{self.requested_height}@"
                        f"{self.requested_fps:g}fps) isn't actually supported by "
                        f"this camera -- check with `v4l2-ctl --list-formats-ext "
                        f"-d {self.device}`; (2) CAMERA_DEVICE points at the "
                        f"wrong /dev/videoN node -- a UVC webcam often exposes a "
                        f"second, metadata-only node next to the real capture "
                        f"one, try the other index; (3) another process already "
                        f"has this device open -- check with `fuser {self.device}` "
                        f"or `lsof {self.device}`."
                    )

            time.sleep(self.frame_interval)

    def latest_jpeg(self):
        with self._lock:
            return self._latest_jpeg


grabber = None  # created in main()
snapshot_store = None  # created in main()
recorder = None  # created in main() (2026-09-18, see camera/recordings.py)


class StreamHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == "/snap":
            self._handle_snap()
            return
        if self.path == "/rec/start":
            self._handle_rec_start()
            return
        if self.path == "/rec/stop":
            self._handle_rec_stop()
            return
        if self.path == "/snapshots":
            self._handle_list(snapshot_store, "snapshots")
            return
        if self.path.startswith("/snapshots/"):
            filename = self.path[len("/snapshots/"):]
            self._handle_file(snapshot_store, filename, "image/jpeg")
            return
        if self.path == "/recordings":
            self._handle_list(recorder, "recordings")
            return
        if self.path.startswith("/recordings/"):
            filename = self.path[len("/recordings/"):]
            self._handle_file(recorder, filename, "video/mp4")
            return
        if self.path != "/stream.mjpg":
            self.send_error(404)
            return

        # 2026-10-03: counts this connection as a "viewer" for as long as
        # it stays open -- see FrameGrabber.add_viewer()/_should_capture()
        # (module docstring above has the full reasoning). The matching
        # remove_viewer() is in `finally` below so it fires however this
        # loop ends (viewer closing the tab -- the expected case, caught
        # below -- but also any other exception), never leaving a stale
        # viewer counted as still connected.
        grabber.add_viewer()
        try:
            self.send_response(200)
            self.send_header("Age", "0")
            self.send_header("Cache-Control", "no-cache, private")
            self.send_header("Pragma", "no-cache")
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=FRAME")
            self.end_headers()

            while True:
                frame = grabber.latest_jpeg()
                if frame is None:
                    time.sleep(0.1)
                    continue
                self.wfile.write(b"--FRAME\r\n")
                self.send_header("Content-Type", "image/jpeg")
                self.send_header("Content-Length", str(len(frame)))
                self.end_headers()
                self.wfile.write(frame)
                self.wfile.write(b"\r\n")
                time.sleep(grabber.frame_interval)
        except (BrokenPipeError, ConnectionResetError):
            # Normal: the viewer (or the web server proxying us) closed the
            # connection -- nothing to log, just stop this handler's loop.
            pass
        finally:
            grabber.remove_viewer()

    def _handle_snap(self):
        """GET /snap: saves the current frame into the capped snapshot
        store (camera/snapshots.py, max 5 files, oldest deleted first) for
        later processing, and reports which file was written. This is
        what link/robot_state.py's camera_command("SNAP") calls over
        plain HTTP -- see that module for why (CAM,SNAP arrives on the
        NMEA link, a separate process from this one).

        UPDATE (2026-10-03): uses wait_for_frame() rather than a plain
        latest_jpeg() check -- with CAMERA_ON_DEMAND, a gamepad-triggered
        SNAP is very often the ONLY thing asking for a frame right now
        (nobody necessarily has /stream.mjpg open at the same time), so a
        bare latest_jpeg() would see a deliberately-released device and
        answer 503 immediately, every time, regardless of whether a real
        camera is actually plugged in. wait_for_frame() wakes it and gives
        it up to CAMERA_WAKE_TIMEOUT_S to deliver a first frame instead."""
        frame = grabber.wait_for_frame(CAMERA_WAKE_TIMEOUT_S) if grabber else None
        if frame is None:
            self.send_response(503)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(json.dumps({"ok": False, "error": "NO_FRAME_YET"}).encode("utf-8"))
            return

        filename = snapshot_store.save(frame)
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.end_headers()
        self.wfile.write(json.dumps({
            "ok": True,
            "file": filename,
            "count": snapshot_store.count(),
        }).encode("utf-8"))

    def _handle_rec_start(self):
        """GET /rec/start: arms camera/recordings.py's VideoRecorder --
        this is what link/robot_state.py's camera_command("REC_START")
        calls over plain HTTP (same cross-process reasoning as SNAP).
        Answers 503 the same way /snap does when there's no frame yet
        (grabber hasn't produced one -- no camera plugged in, or not
        opened yet): REC_START refusing cleanly here, rather than arming a
        recorder with nothing to encode, is exactly what "record a video
        if the camera is present" (this feature's original request) means
        in practice.

        UPDATE (2026-10-03): same wait_for_frame() change as _handle_snap()
        above, same reason -- a gamepad's record button is the normal way
        this gets triggered on a field robot, very often with nobody
        watching /stream.mjpg at the same time, so an on-demand-released
        camera must get a real chance to wake up and deliver a frame
        before this gives up, not an instant 503 every time. Once a frame
        arrives, recorder.start() below arms recording properly, which is
        itself one of _should_capture()'s own conditions -- so the device
        then stays open on its own for the rest of the recording, with no
        further need for this request's own transient wake (already
        released again by the time wait_for_frame() returns)."""
        frame = grabber.wait_for_frame(CAMERA_WAKE_TIMEOUT_S) if grabber else None
        if frame is None:
            self.send_response(503)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(json.dumps({"ok": False, "error": "NO_FRAME_YET"}).encode("utf-8"))
            return

        recorder.start()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.end_headers()
        self.wfile.write(json.dumps({"ok": True, "recording": True}).encode("utf-8"))

    def _handle_rec_stop(self):
        """GET /rec/stop: disarms the recorder and closes the file, if one
        was actually opened (see VideoRecorder.stop() -- possible that no
        frame ever arrived between /rec/start and this, e.g. camera
        unplugged mid-recording). Always answers 200 -- "stop recording"
        genuinely succeeding even when nothing was written is not an error
        condition the way "no frame yet" is for /rec/start above."""
        filename = recorder.stop() if recorder else None
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.end_headers()
        self.wfile.write(json.dumps({
            "ok": True, "recording": False, "file": filename,
        }).encode("utf-8"))

    def _handle_list(self, store, label):
        """GET /snapshots or /recordings: JSON listing of what's currently
        on disk, newest first (store.list_files()) -- what the web
        server's Media page (robot-webserver repo) polls to build its
        thumbnail/link grid for the "Photos Snap" and "Vidéos
        enregistrées" panels. `store` is None only if main() hasn't run
        yet (shouldn't happen once the server is actually serving
        requests), handled the same defensive way _handle_snap() etc.
        handle a None grabber."""
        files = store.list_files() if store else []
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.end_headers()
        self.wfile.write(json.dumps({"ok": True, label: files}).encode("utf-8"))

    def _handle_file(self, store, filename, content_type):
        """GET /snapshots/<filename> or /recordings/<filename>: streams
        one file's raw bytes back. `filename` is validated against
        store.list_files() rather than trusted as-is -- this rejects both
        a stale name (already pruned by the rolling FIFO cap, see
        SnapshotStore/VideoRecorder) and any path-traversal attempt
        (../../etc/passwd and the like can never appear in list_files()'s
        own output, since that's built from a plain os.listdir() filtered
        to this store's own naming pattern).

        Uses shutil.copyfileobj() to stream the file in chunks rather than
        reading it whole into memory first -- snapshots are small JPEGs so
        it wouldn't matter much there, but recordings can be several
        megabytes and this is a Raspberry Pi serving other real-time work
        at the same time."""
        if store is None or filename not in store.list_files():
            self.send_error(404)
            return

        path = os.path.join(store.directory, filename)
        try:
            file_size = os.path.getsize(path)
        except OSError:
            # Gone between the list_files() check above and now (e.g.
            # pruned by a save() that raced this request) -- same
            # "already gone -- fine" spirit as the stores' own pruning.
            self.send_error(404)
            return

        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(file_size))
        self.end_headers()
        try:
            with open(path, "rb") as f:
                shutil.copyfileobj(f, self.wfile)
        except (BrokenPipeError, ConnectionResetError):
            # Normal: the viewer (or the web server proxying us) closed
            # the connection mid-transfer -- same handling as the live
            # stream loop above.
            pass

    def log_message(self, format, *args):
        pass  # silence the default per-request stderr logging


def main():
    global grabber, snapshot_store, recorder
    recording_dir = os.environ.get("CAMERA_RECORDING_DIR")
    recorder = VideoRecorder(recording_dir) if recording_dir else VideoRecorder(fps=CAMERA_FPS)
    grabber = FrameGrabber(CAMERA_DEVICE, CAMERA_WIDTH, CAMERA_HEIGHT, CAMERA_FPS,
                           recorder=recorder, on_demand=CAMERA_ON_DEMAND)
    grabber.start()
    snapshot_dir = os.environ.get("CAMERA_SNAPSHOT_DIR")
    snapshot_store = SnapshotStore(snapshot_dir) if snapshot_dir else SnapshotStore()

    server = ThreadingHTTPServer((CAMERA_HOST, CAMERA_PORT), StreamHandler)
    print(f"Camera stream: http://{CAMERA_HOST}:{CAMERA_PORT}/stream.mjpg")
    print(f"Snapshots (max {snapshot_store.max_snapshots}): http://{CAMERA_HOST}:{CAMERA_PORT}/snap -> {snapshot_store.directory}")
    print(f"Recordings (max {recorder.max_recordings}): http://{CAMERA_HOST}:{CAMERA_PORT}/rec/start|stop -> {recorder.directory}")
    print(f"  Listing/download: http://{CAMERA_HOST}:{CAMERA_PORT}/snapshots and /recordings")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()


if __name__ == "__main__":
    main()
