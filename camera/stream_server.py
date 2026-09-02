"""Live MJPEG camera stream for Raspberry Pi #1 (robot side).

Captures frames from a USB webcam via OpenCV and serves them over plain
HTTP as a multipart/x-mixed-replace stream -- the "MJPEG over HTTP" trick
that every browser already knows how to display through a plain <img> tag,
no plugin and no WebRTC signaling needed.

This is intentionally independent from the NMEA control link
(link/server.py): CAM,SNAP is handled there by making a plain HTTP request
to this process's own /snap endpoint (below) rather than the two talking
over the NMEA link itself -- see link/robot_state.py's camera_command().
REC_START/REC_STOP remain unimplemented; this module only adds one-shot
snapshots on top of the continuous live preview.

The web server (Raspberry Pi #2, robot-webserver repo) does not load this
stream directly in the browser -- it proxies it from its own /media/camera
route (see app.py there), so the feed stays behind the site's login.

Run with:
    python3 -m camera

Configuration (environment variables, all optional):
    CAMERA_DEVICE   video device index or path (default: 0, i.e. /dev/video0)
    CAMERA_WIDTH    capture width in pixels (default: 640)
    CAMERA_HEIGHT   capture height in pixels (default: 480)
    CAMERA_FPS      target capture/stream rate (default: 15)
    CAMERA_HOST     interface to listen on (default: 0.0.0.0)
    CAMERA_PORT     port to listen on (default: 8000)
    CAMERA_SNAPSHOT_DIR  where GET /snap saves files (default: camera/tmp/,
                    see camera/snapshots.py -- never more than 5 at once)

NOTE: written and reviewed against the OpenCV/http.server APIs, but not
run against a real webcam in this environment -- test on the Pi with the
actual camera plugged in before relying on it.
"""
import json
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2

from camera.snapshots import SnapshotStore

CAMERA_DEVICE = os.environ.get("CAMERA_DEVICE", "0")
CAMERA_WIDTH = int(os.environ.get("CAMERA_WIDTH", "640"))
CAMERA_HEIGHT = int(os.environ.get("CAMERA_HEIGHT", "480"))
CAMERA_FPS = float(os.environ.get("CAMERA_FPS", "15"))
CAMERA_HOST = os.environ.get("CAMERA_HOST", "0.0.0.0")
CAMERA_PORT = int(os.environ.get("CAMERA_PORT", "8000"))

JPEG_QUALITY = 80  # 0-100, trade-off between bandwidth and image quality


class FrameGrabber:
    """Continuously reads frames from the webcam in a background thread and
    keeps only the latest one, already JPEG-encoded. Running the capture
    loop independently from the HTTP handler means a slow or disconnected
    viewer never blocks the camera read loop, and several viewers (or a
    viewer that reconnects) can all share the same encoded frame."""

    def __init__(self, device, width, height, fps):
        # device is usually a numeric index ("0") but OpenCV also accepts a
        # path like "/dev/video2" -- try int first, fall back to the string.
        try:
            device = int(device)
        except ValueError:
            pass

        self.frame_interval = 1.0 / fps if fps > 0 else 0.05
        self._capture = cv2.VideoCapture(device)
        self._capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self._capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self._capture.set(cv2.CAP_PROP_FPS, fps)
        self._lock = threading.Lock()
        self._latest_jpeg = None
        self._running = False

        if not self._capture.isOpened():
            raise RuntimeError(f"Could not open camera device {device!r}")

    def start(self):
        self._running = True
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        while self._running:
            ok, frame = self._capture.read()
            if ok:
                ok, buffer = cv2.imencode(
                    ".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY]
                )
                if ok:
                    with self._lock:
                        self._latest_jpeg = buffer.tobytes()
            time.sleep(self.frame_interval)

    def latest_jpeg(self):
        with self._lock:
            return self._latest_jpeg


grabber = None  # created in main()
snapshot_store = None  # created in main()


class StreamHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == "/snap":
            self._handle_snap()
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

    def log_message(self, format, *args):
        pass  # silence the default per-request stderr logging


def main():
    global grabber, snapshot_store
    grabber = FrameGrabber(CAMERA_DEVICE, CAMERA_WIDTH, CAMERA_HEIGHT, CAMERA_FPS)
    grabber.start()
    snapshot_dir = os.environ.get("CAMERA_SNAPSHOT_DIR")
    snapshot_store = SnapshotStore(snapshot_dir) if snapshot_dir else SnapshotStore()

    server = ThreadingHTTPServer((CAMERA_HOST, CAMERA_PORT), StreamHandler)
    print(f"Camera stream: http://{CAMERA_HOST}:{CAMERA_PORT}/stream.mjpg")
    print(f"Snapshots (max {snapshot_store.max_snapshots}): http://{CAMERA_HOST}:{CAMERA_PORT}/snap -> {snapshot_store.directory}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()


if __name__ == "__main__":
    main()
