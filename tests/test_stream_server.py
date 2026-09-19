"""Tests for camera/stream_server.py's FrameGrabber -- specifically that a
missing/unavailable camera no longer crashes the process.

Regression covered here: FrameGrabber used to call cv2.VideoCapture()
synchronously in __init__ and raise RuntimeError the moment isOpened()
came back False, which killed `python3 -m camera` before the HTTP server
even started -- and, via run_robot.sh, took link/server.py down with it
(see that script's own history). The device is now opened lazily inside
the background thread and retried instead of raising.

cv2 IS installed in this sandbox (unlike evdev for the gamepad -- see
tests/test_gamepad_handler.py's own docstring), so these tests mock
cv2.VideoCapture directly rather than stubbing the whole module. They
exercise FrameGrabber's own open/retry control flow against a fake
capture object, not real camera hardware.
"""
import http.server
import json
import threading
import time
import urllib.error
import urllib.request
from unittest.mock import MagicMock, patch

import camera.stream_server as stream_server
from camera.stream_server import FrameGrabber, StreamHandler


class _FakeCapture:
    """Stands in for cv2.VideoCapture. `opened` controls isOpened(); once
    open, read() cycles through `frames` (a list of (ok, frame) pairs)."""

    def __init__(self, opened=False, frames=None):
        self.opened = opened
        self.frames = frames or []
        self._frame_index = 0
        self.released = False

    def set(self, prop, value):
        pass

    def get(self, prop):
        return 0

    def isOpened(self):
        return self.opened

    def read(self):
        if not self.frames:
            return False, None
        frame = self.frames[self._frame_index % len(self.frames)]
        self._frame_index += 1
        return frame

    def release(self):
        self.released = True


def test_constructor_never_raises_when_camera_missing():
    """The actual regression: this used to raise RuntimeError."""
    with patch("camera.stream_server.cv2.VideoCapture", return_value=_FakeCapture(opened=False)):
        grabber = FrameGrabber(device=0, width=640, height=480, fps=15)
    assert grabber.latest_jpeg() is None


def test_open_capture_returns_false_and_releases_when_device_wont_open():
    fake = _FakeCapture(opened=False)
    with patch("camera.stream_server.cv2.VideoCapture", return_value=fake):
        grabber = FrameGrabber(device=0, width=640, height=480, fps=15)
        assert grabber._open_capture() is False
    assert fake.released is True
    assert grabber._capture is None


def test_open_capture_succeeds_and_keeps_capture_when_device_opens():
    fake = _FakeCapture(opened=True)
    with patch("camera.stream_server.cv2.VideoCapture", return_value=fake):
        grabber = FrameGrabber(device=0, width=640, height=480, fps=15)
        assert grabber._open_capture() is True
    assert grabber._capture is fake


def test_loop_retries_opening_until_camera_becomes_available():
    """Simulates a camera that isn't there yet, then gets plugged in --
    the background loop should pick it up on its own, no restart needed."""
    attempts = {"count": 0}

    def fake_video_capture(device):
        attempts["count"] += 1
        if attempts["count"] == 1:
            return _FakeCapture(opened=False)  # not plugged in yet
        frame = MagicMock()
        frame.shape = (480, 640, 3)
        return _FakeCapture(opened=True, frames=[(True, frame)])

    fake_buffer = MagicMock()
    fake_buffer.tobytes.return_value = b"jpeg-bytes"

    with patch("camera.stream_server.cv2.VideoCapture", side_effect=fake_video_capture), \
         patch("camera.stream_server.cv2.imencode", return_value=(True, fake_buffer)):
        grabber = FrameGrabber(device=0, width=640, height=480, fps=15, retry_interval=0.05)
        grabber.start()
        deadline = time.time() + 2.0
        while grabber.latest_jpeg() is None and time.time() < deadline:
            time.sleep(0.02)
        grabber._running = False

    assert grabber.latest_jpeg() == b"jpeg-bytes"
    assert attempts["count"] >= 2  # at least one failed attempt, then a success


# --- FrameGrabber + VideoRecorder wiring (2026-09-18) -----------------------

def test_frame_grabber_feeds_raw_frames_to_an_optional_recorder():
    # Every frame successfully read is handed to recorder.write() in
    # addition to being JPEG-encoded for the live stream -- see
    # camera/recordings.py's VideoRecorder (a no-op there unless armed,
    # but that's VideoRecorder's own test's concern, not this one's).
    frame = MagicMock()
    frame.shape = (480, 640, 3)
    fake_capture = _FakeCapture(opened=True, frames=[(True, frame)])
    fake_recorder = MagicMock()

    fake_buffer = MagicMock()
    fake_buffer.tobytes.return_value = b"jpeg-bytes"

    with patch("camera.stream_server.cv2.VideoCapture", return_value=fake_capture), \
         patch("camera.stream_server.cv2.imencode", return_value=(True, fake_buffer)):
        grabber = FrameGrabber(device=0, width=640, height=480, fps=15, recorder=fake_recorder)
        grabber.start()
        deadline = time.time() + 2.0
        while grabber.latest_jpeg() is None and time.time() < deadline:
            time.sleep(0.02)
        grabber._running = False

    fake_recorder.write.assert_called_with(frame)


def test_frame_grabber_recorder_defaults_to_none():
    # Existing callers/tests that don't pass recorder= must be unaffected.
    with patch("camera.stream_server.cv2.VideoCapture", return_value=_FakeCapture(opened=False)):
        grabber = FrameGrabber(device=0, width=640, height=480, fps=15)
    assert grabber.recorder is None


# --- StreamHandler: /rec/start, /rec/stop (2026-09-18) ----------------------

def _start_test_server():
    server = http.server.HTTPServer(("127.0.0.1", 0), StreamHandler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    return server, thread


def test_rec_start_arms_the_recorder_when_a_frame_is_already_available(monkeypatch):
    fake_grabber = MagicMock()
    fake_grabber.latest_jpeg.return_value = b"jpeg-bytes"
    fake_recorder = MagicMock()
    monkeypatch.setattr(stream_server, "grabber", fake_grabber)
    monkeypatch.setattr(stream_server, "recorder", fake_recorder)

    server, thread = _start_test_server()
    try:
        port = server.server_address[1]
        with urllib.request.urlopen(f"http://127.0.0.1:{port}/rec/start", timeout=2) as resp:
            body = json.loads(resp.read())
    finally:
        server.shutdown()
        server.server_close()

    assert body == {"ok": True, "recording": True}
    fake_recorder.start.assert_called_once()


def test_rec_start_answers_503_with_no_frame_yet(monkeypatch):
    # Same convention as /snap's own 503 -- REC_START refuses cleanly
    # rather than arming a recorder with nothing to encode (this is what
    # makes "record a video if the camera is present" true in practice).
    fake_grabber = MagicMock()
    fake_grabber.latest_jpeg.return_value = None
    fake_recorder = MagicMock()
    monkeypatch.setattr(stream_server, "grabber", fake_grabber)
    monkeypatch.setattr(stream_server, "recorder", fake_recorder)

    server, thread = _start_test_server()
    try:
        port = server.server_address[1]
        try:
            urllib.request.urlopen(f"http://127.0.0.1:{port}/rec/start", timeout=2)
            raise AssertionError("expected an HTTPError (503)")
        except urllib.error.HTTPError as exc:
            assert exc.code == 503
            body = json.loads(exc.read())
    finally:
        server.shutdown()
        server.server_close()

    assert body == {"ok": False, "error": "NO_FRAME_YET"}
    fake_recorder.start.assert_not_called()


def test_rec_stop_reports_the_filename_that_was_written(monkeypatch):
    fake_recorder = MagicMock()
    fake_recorder.stop.return_value = "rec_20260918_120000_000001.mp4"
    monkeypatch.setattr(stream_server, "recorder", fake_recorder)

    server, thread = _start_test_server()
    try:
        port = server.server_address[1]
        with urllib.request.urlopen(f"http://127.0.0.1:{port}/rec/stop", timeout=2) as resp:
            assert resp.status == 200
            body = json.loads(resp.read())
    finally:
        server.shutdown()
        server.server_close()

    assert body == {"ok": True, "recording": False, "file": "rec_20260918_120000_000001.mp4"}


def test_rec_stop_reports_no_file_when_nothing_was_recorded(monkeypatch):
    fake_recorder = MagicMock()
    fake_recorder.stop.return_value = None
    monkeypatch.setattr(stream_server, "recorder", fake_recorder)

    server, thread = _start_test_server()
    try:
        port = server.server_address[1]
        with urllib.request.urlopen(f"http://127.0.0.1:{port}/rec/stop", timeout=2) as resp:
            body = json.loads(resp.read())
    finally:
        server.shutdown()
        server.server_close()

    assert body == {"ok": True, "recording": False, "file": None}


# --- StreamHandler: /snapshots, /recordings listing + file serving (2026-09-19) ----

def test_snapshots_listing_reports_newest_first(monkeypatch):
    fake_store = MagicMock()
    fake_store.list_files.return_value = ["snap_20260919_120200_000002.jpg", "snap_20260919_120100_000001.jpg"]
    monkeypatch.setattr(stream_server, "snapshot_store", fake_store)

    server, thread = _start_test_server()
    try:
        port = server.server_address[1]
        with urllib.request.urlopen(f"http://127.0.0.1:{port}/snapshots", timeout=2) as resp:
            assert resp.status == 200
            body = json.loads(resp.read())
    finally:
        server.shutdown()
        server.server_close()

    assert body == {"ok": True, "snapshots": [
        "snap_20260919_120200_000002.jpg", "snap_20260919_120100_000001.jpg",
    ]}


def test_recordings_listing_reports_newest_first(monkeypatch):
    fake_recorder = MagicMock()
    fake_recorder.list_files.return_value = ["rec_20260919_120200_000002.mp4", "rec_20260919_120100_000001.mp4"]
    monkeypatch.setattr(stream_server, "recorder", fake_recorder)

    server, thread = _start_test_server()
    try:
        port = server.server_address[1]
        with urllib.request.urlopen(f"http://127.0.0.1:{port}/recordings", timeout=2) as resp:
            body = json.loads(resp.read())
    finally:
        server.shutdown()
        server.server_close()

    assert body == {"ok": True, "recordings": [
        "rec_20260919_120200_000002.mp4", "rec_20260919_120100_000001.mp4",
    ]}


def test_snapshot_file_is_served_when_it_is_a_known_file(monkeypatch, tmp_path):
    filename = "snap_20260919_120100_000001.jpg"
    (tmp_path / filename).write_bytes(b"\xff\xd8\xff\xd9fake-jpeg")
    fake_store = MagicMock()
    fake_store.list_files.return_value = [filename]
    fake_store.directory = str(tmp_path)
    monkeypatch.setattr(stream_server, "snapshot_store", fake_store)

    server, thread = _start_test_server()
    try:
        port = server.server_address[1]
        with urllib.request.urlopen(f"http://127.0.0.1:{port}/snapshots/{filename}", timeout=2) as resp:
            assert resp.status == 200
            assert resp.headers["Content-Type"] == "image/jpeg"
            body = resp.read()
    finally:
        server.shutdown()
        server.server_close()

    assert body == b"\xff\xd8\xff\xd9fake-jpeg"


def test_unknown_snapshot_filename_is_rejected_with_404(monkeypatch, tmp_path):
    # Covers both a stale (already-pruned) name and a path-traversal
    # attempt -- neither can appear in list_files()'s own output, so both
    # are refused the same way.
    fake_store = MagicMock()
    fake_store.list_files.return_value = ["snap_20260919_120100_000001.jpg"]
    fake_store.directory = str(tmp_path)
    monkeypatch.setattr(stream_server, "snapshot_store", fake_store)

    server, thread = _start_test_server()
    try:
        port = server.server_address[1]
        try:
            urllib.request.urlopen(
                f"http://127.0.0.1:{port}/snapshots/../../etc/passwd", timeout=2
            )
            raise AssertionError("expected an HTTPError (404)")
        except urllib.error.HTTPError as exc:
            status = exc.code
    finally:
        server.shutdown()
        server.server_close()

    assert status == 404


def test_recording_file_is_served_with_video_content_type(monkeypatch, tmp_path):
    filename = "rec_20260919_120100_000001.mp4"
    (tmp_path / filename).write_bytes(b"fake-mp4-bytes")
    fake_recorder = MagicMock()
    fake_recorder.list_files.return_value = [filename]
    fake_recorder.directory = str(tmp_path)
    monkeypatch.setattr(stream_server, "recorder", fake_recorder)

    server, thread = _start_test_server()
    try:
        port = server.server_address[1]
        with urllib.request.urlopen(f"http://127.0.0.1:{port}/recordings/{filename}", timeout=2) as resp:
            assert resp.status == 200
            assert resp.headers["Content-Type"] == "video/mp4"
            body = resp.read()
    finally:
        server.shutdown()
        server.server_close()

    assert body == b"fake-mp4-bytes"
