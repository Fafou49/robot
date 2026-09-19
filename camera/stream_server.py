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
                 recorder=None):
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

        self.frame_interval = 1.0 / fps if fps > 0 else 0.05
        self._capture = None
        self._lock = threading.Lock()
        self._latest_jpeg = None
        self._running = False

    def start(self):
        self._running = True
        threading.Thread(target=self._loop, daemon=True).start()

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

        self.send_response(200)
        self.send_header("Age", "0")
        self.send_header("Cache-Control", "no-cache, private")
        self.send_header("Pragma", "no-cache")
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=FRAME")
        self.end_headers()

        try:
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

    def _handle_snap(self):
        """GET /snap: saves the current frame into the capped snapshot
        store (camera/snapshots.py, max 5 files, oldest deleted first) for
        later processing, and reports which file was written. This is
        what link/robot_state.py's camera_command("SNAP") calls over
        plain HTTP -- see that module for why (CAM,SNAP arrives on the
        NMEA link, a separate process from this one)."""
        frame = grabber.latest_jpeg() if grabber else None
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
        in practice."""
        frame = grabber.latest_jpeg() if grabber else None
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
    grabber = FrameGrabber(CAMERA_DEVICE, CAMERA_WIDTH, CAMERA_HEIGHT, CAMERA_FPS, recorder=recorder)
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
