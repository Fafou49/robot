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

import pytest

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


# --- StreamHandler: DELETE /snapshots/<filename>, /recordings/<filename> (2026-10-05) ----

def test_delete_snapshot_removes_a_known_file(monkeypatch):
    fake_store = MagicMock()
    fake_store.delete.return_value = True
    monkeypatch.setattr(stream_server, "snapshot_store", fake_store)

    server, thread = _start_test_server()
    try:
        port = server.server_address[1]
        req = urllib.request.Request(f"http://127.0.0.1:{port}/snapshots/snap_x.jpg", method="DELETE")
        with urllib.request.urlopen(req, timeout=2) as resp:
            assert resp.status == 200
            body = json.loads(resp.read())
    finally:
        server.shutdown()
        server.server_close()

    assert body == {"ok": True, "file": "snap_x.jpg"}
    fake_store.delete.assert_called_once_with("snap_x.jpg")


def test_delete_snapshot_404s_when_the_file_is_not_present(monkeypatch):
    fake_store = MagicMock()
    fake_store.delete.return_value = False
    monkeypatch.setattr(stream_server, "snapshot_store", fake_store)

    server, thread = _start_test_server()
    try:
        port = server.server_address[1]
        req = urllib.request.Request(f"http://127.0.0.1:{port}/snapshots/gone.jpg", method="DELETE")
        try:
            urllib.request.urlopen(req, timeout=2)
            raise AssertionError("expected an HTTPError (404)")
        except urllib.error.HTTPError as exc:
            assert exc.code == 404
            body = json.loads(exc.read())
    finally:
        server.shutdown()
        server.server_close()

    assert body == {"ok": False, "file": "gone.jpg"}


def test_delete_recording_removes_a_known_file(monkeypatch):
    fake_recorder = MagicMock()
    fake_recorder.delete.return_value = True
    monkeypatch.setattr(stream_server, "recorder", fake_recorder)

    server, thread = _start_test_server()
    try:
        port = server.server_address[1]
        req = urllib.request.Request(f"http://127.0.0.1:{port}/recordings/rec_x.mp4", method="DELETE")
        with urllib.request.urlopen(req, timeout=2) as resp:
            assert resp.status == 200
            body = json.loads(resp.read())
    finally:
        server.shutdown()
        server.server_close()

    assert body == {"ok": True, "file": "rec_x.mp4"}
    fake_recorder.delete.assert_called_once_with("rec_x.mp4")


def test_delete_on_an_unknown_path_is_404(monkeypatch):
    server, thread = _start_test_server()
    try:
        port = server.server_address[1]
        req = urllib.request.Request(f"http://127.0.0.1:{port}/not-a-real-path", method="DELETE")
        try:
            urllib.request.urlopen(req, timeout=2)
            raise AssertionError("expected an HTTPError (404)")
        except urllib.error.HTTPError as exc:
            status = exc.code
    finally:
        server.shutdown()
        server.server_close()

    assert status == 404


# --- FrameGrabber: reactive/eco mode (2026-10-10) ---------------------------
# Explicit user request: "je souhaites la reactivite lorsque le panneau
# solaire charge ou que j'ai assez d'energie pour passer la nuit. En cas
# d'incertitude on passe en mode eco" -- the actual REACTIVE/ECO decision
# lives in link/camera_power_policy.py (tests/test_camera_power_policy.py);
# these tests only cover what FrameGrabber/StreamHandler DO with that
# decision once it's handed to them.

def test_set_mode_rejects_an_invalid_value():
    with patch("camera.stream_server.cv2.VideoCapture", return_value=_FakeCapture(opened=False)):
        grabber = FrameGrabber(device=0, width=640, height=480, fps=15)
    with pytest.raises(ValueError):
        grabber.set_mode("turbo")
    assert grabber.mode == "reactive"  # unchanged by the rejected call


def test_set_mode_accepts_reactive_and_eco():
    with patch("camera.stream_server.cv2.VideoCapture", return_value=_FakeCapture(opened=False)):
        grabber = FrameGrabber(device=0, width=640, height=480, fps=15)
    grabber.set_mode("eco")
    assert grabber.mode == "eco"
    grabber.set_mode("reactive")
    assert grabber.mode == "reactive"


def test_frame_grabber_defaults_to_reactive_mode():
    # Matches this class's own behaviour before eco mode existed --
    # continuous capture regardless of viewers -- for every existing
    # caller/test that never calls set_mode() at all.
    with patch("camera.stream_server.cv2.VideoCapture", return_value=_FakeCapture(opened=False)):
        grabber = FrameGrabber(device=0, width=640, height=480, fps=15)
    assert grabber.mode == "reactive"


def test_viewer_connected_disconnected_tracks_a_count():
    with patch("camera.stream_server.cv2.VideoCapture", return_value=_FakeCapture(opened=False)):
        grabber = FrameGrabber(device=0, width=640, height=480, fps=15)
    grabber.viewer_connected()
    grabber.viewer_connected()
    assert grabber._active_viewers == 2
    grabber.viewer_disconnected()
    assert grabber._active_viewers == 1


def test_viewer_disconnected_never_goes_negative():
    # An extra disconnect (e.g. a bug, or a race between two handler
    # threads) must not leave the count negative, which would otherwise
    # require two more connects than disconnects before eco mode could
    # ever engage again.
    with patch("camera.stream_server.cv2.VideoCapture", return_value=_FakeCapture(opened=False)):
        grabber = FrameGrabber(device=0, width=640, height=480, fps=15)
    grabber.viewer_disconnected()
    assert grabber._active_viewers == 0


def test_request_active_extends_rather_than_shortens_the_window(monkeypatch):
    with patch("camera.stream_server.cv2.VideoCapture", return_value=_FakeCapture(opened=False)):
        grabber = FrameGrabber(device=0, width=640, height=480, fps=15)
    monkeypatch.setattr(time, "time", lambda: 1000.0)
    grabber.request_active(duration=10.0)
    assert grabber._force_active_until == 1010.0
    grabber.request_active(duration=2.0)  # a shorter request right after
    assert grabber._force_active_until == 1010.0  # unchanged -- not shortened


def test_should_release_for_eco_mode_is_always_false_in_reactive_mode():
    with patch("camera.stream_server.cv2.VideoCapture", return_value=_FakeCapture(opened=False)):
        grabber = FrameGrabber(device=0, width=640, height=480, fps=15)
    assert grabber.mode == "reactive"
    grabber.set_mode("reactive")
    assert grabber._should_release_for_eco_mode() is False


def test_should_release_for_eco_mode_true_when_idle_in_eco_mode():
    with patch("camera.stream_server.cv2.VideoCapture", return_value=_FakeCapture(opened=False)):
        grabber = FrameGrabber(device=0, width=640, height=480, fps=15)
    grabber.set_mode("eco")
    assert grabber._should_release_for_eco_mode() is True


def test_should_release_for_eco_mode_false_with_an_active_viewer():
    with patch("camera.stream_server.cv2.VideoCapture", return_value=_FakeCapture(opened=False)):
        grabber = FrameGrabber(device=0, width=640, height=480, fps=15)
    grabber.set_mode("eco")
    grabber.viewer_connected()
    assert grabber._should_release_for_eco_mode() is False


def test_should_release_for_eco_mode_false_during_a_pending_wake_request(monkeypatch):
    with patch("camera.stream_server.cv2.VideoCapture", return_value=_FakeCapture(opened=False)):
        grabber = FrameGrabber(device=0, width=640, height=480, fps=15)
    grabber.set_mode("eco")
    monkeypatch.setattr(time, "time", lambda: 1000.0)
    grabber.request_active(duration=5.0)
    assert grabber._should_release_for_eco_mode() is False
    monkeypatch.setattr(time, "time", lambda: 1006.0)  # past the window
    assert grabber._should_release_for_eco_mode() is True


def test_should_release_for_eco_mode_false_while_a_recording_is_in_progress():
    fake_recorder = MagicMock()
    fake_recorder.is_recording = True
    with patch("camera.stream_server.cv2.VideoCapture", return_value=_FakeCapture(opened=False)):
        grabber = FrameGrabber(device=0, width=640, height=480, fps=15, recorder=fake_recorder)
    grabber.set_mode("eco")
    assert grabber._should_release_for_eco_mode() is False


def test_should_release_for_eco_mode_true_with_no_recorder_at_all():
    with patch("camera.stream_server.cv2.VideoCapture", return_value=_FakeCapture(opened=False)):
        grabber = FrameGrabber(device=0, width=640, height=480, fps=15)
    assert grabber.recorder is None
    grabber.set_mode("eco")
    assert grabber._should_release_for_eco_mode() is True


# --- FrameGrabber._loop(): eco mode actually releasing/reopening the device -

def test_loop_never_opens_the_camera_while_idle_in_eco_mode_from_the_start():
    open_calls = {"count": 0}

    def fake_video_capture(device):
        open_calls["count"] += 1
        frame = MagicMock()
        frame.shape = (480, 640, 3)
        return _FakeCapture(opened=True, frames=[(True, frame)])

    with patch("camera.stream_server.cv2.VideoCapture", side_effect=fake_video_capture):
        grabber = FrameGrabber(device=0, width=640, height=480, fps=15, retry_interval=0.05)
        grabber.set_mode("eco")  # idle from the very first tick: no viewer, no recording
        grabber.start()
        time.sleep(0.3)
        grabber._running = False

    assert open_calls["count"] == 0
    assert grabber.latest_jpeg() is None


def test_loop_releases_the_device_once_eco_mode_engages_with_no_viewers():
    fake_capture = _FakeCapture(opened=True, frames=[])

    with patch("camera.stream_server.cv2.VideoCapture", return_value=fake_capture):
        grabber = FrameGrabber(device=0, width=640, height=480, fps=15, retry_interval=0.05)
        grabber.start()
        deadline = time.time() + 2.0
        while grabber._capture is None and time.time() < deadline:
            time.sleep(0.02)
        assert grabber._capture is fake_capture  # opened under the default reactive mode

        grabber.set_mode("eco")
        deadline = time.time() + 2.0
        while grabber._capture is not None and time.time() < deadline:
            time.sleep(0.02)
        grabber._running = False

    assert fake_capture.released is True
    assert grabber._capture is None


def test_loop_reopens_the_device_once_a_viewer_connects_in_eco_mode():
    open_calls = {"count": 0}

    def fake_video_capture(device):
        open_calls["count"] += 1
        frame = MagicMock()
        frame.shape = (480, 640, 3)
        return _FakeCapture(opened=True, frames=[(True, frame)])

    fake_buffer = MagicMock()
    fake_buffer.tobytes.return_value = b"jpeg-bytes"

    with patch("camera.stream_server.cv2.VideoCapture", side_effect=fake_video_capture), \
         patch("camera.stream_server.cv2.imencode", return_value=(True, fake_buffer)):
        grabber = FrameGrabber(device=0, width=640, height=480, fps=15, retry_interval=0.05)
        grabber.set_mode("eco")
        grabber.start()
        time.sleep(0.2)
        assert open_calls["count"] == 0  # confirmed idle so far

        grabber.viewer_connected()
        deadline = time.time() + 2.0
        while grabber.latest_jpeg() is None and time.time() < deadline:
            time.sleep(0.02)
        grabber._running = False

    assert grabber.latest_jpeg() == b"jpeg-bytes"
    assert open_calls["count"] >= 1


# --- StreamHandler: POST /power_mode (2026-10-10) ---------------------------

def test_power_mode_sets_the_grabbers_mode(monkeypatch):
    fake_grabber = MagicMock()
    monkeypatch.setattr(stream_server, "grabber", fake_grabber)

    server, thread = _start_test_server()
    try:
        port = server.server_address[1]
        req = urllib.request.Request(
            f"http://127.0.0.1:{port}/power_mode",
            data=json.dumps({"mode": "eco"}).encode("utf-8"),
            method="POST",
            headers={"Content-Type": "application/json"},
        )
        with urllib.request.urlopen(req, timeout=2) as resp:
            body = json.loads(resp.read())
    finally:
        server.shutdown()
        server.server_close()

    assert body == {"ok": True, "mode": "eco"}
    fake_grabber.set_mode.assert_called_once_with("eco")


def test_power_mode_rejects_an_invalid_mode(monkeypatch):
    fake_grabber = MagicMock()
    fake_grabber.set_mode.side_effect = ValueError("invalid camera mode: 'turbo'")
    monkeypatch.setattr(stream_server, "grabber", fake_grabber)

    server, thread = _start_test_server()
    try:
        port = server.server_address[1]
        req = urllib.request.Request(
            f"http://127.0.0.1:{port}/power_mode",
            data=json.dumps({"mode": "turbo"}).encode("utf-8"),
            method="POST",
            headers={"Content-Type": "application/json"},
        )
        try:
            urllib.request.urlopen(req, timeout=2)
            raise AssertionError("expected an HTTPError (400)")
        except urllib.error.HTTPError as exc:
            assert exc.code == 400
            body = json.loads(exc.read())
    finally:
        server.shutdown()
        server.server_close()

    assert body["ok"] is False


def test_power_mode_rejects_a_malformed_body(monkeypatch):
    fake_grabber = MagicMock()
    monkeypatch.setattr(stream_server, "grabber", fake_grabber)

    server, thread = _start_test_server()
    try:
        port = server.server_address[1]
        req = urllib.request.Request(
            f"http://127.0.0.1:{port}/power_mode",
            data=b"not json",
            method="POST",
            headers={"Content-Type": "application/json"},
        )
        try:
            urllib.request.urlopen(req, timeout=2)
            raise AssertionError("expected an HTTPError (400)")
        except urllib.error.HTTPError as exc:
            assert exc.code == 400
    finally:
        server.shutdown()
        server.server_close()

    fake_grabber.set_mode.assert_not_called()


def test_power_mode_on_an_unknown_path_is_still_404(monkeypatch):
    fake_grabber = MagicMock()
    monkeypatch.setattr(stream_server, "grabber", fake_grabber)

    server, thread = _start_test_server()
    try:
        port = server.server_address[1]
        req = urllib.request.Request(
            f"http://127.0.0.1:{port}/not-a-real-path", data=b"{}", method="POST",
        )
        try:
            urllib.request.urlopen(req, timeout=2)
            raise AssertionError("expected an HTTPError (404)")
        except urllib.error.HTTPError as exc:
            status = exc.code
    finally:
        server.shutdown()
        server.server_close()

    assert status == 404


def test_power_mode_returns_503_when_grabber_not_yet_constructed(monkeypatch):
    monkeypatch.setattr(stream_server, "grabber", None)

    server, thread = _start_test_server()
    try:
        port = server.server_address[1]
        req = urllib.request.Request(
            f"http://127.0.0.1:{port}/power_mode",
            data=json.dumps({"mode": "eco"}).encode("utf-8"),
            method="POST",
        )
        try:
            urllib.request.urlopen(req, timeout=2)
            raise AssertionError("expected an HTTPError (503)")
        except urllib.error.HTTPError as exc:
            status = exc.code
    finally:
        server.shutdown()
        server.server_close()

    assert status == 503


# --- StreamHandler: /stream.mjpg viewer tracking (2026-10-10) --------------

def test_stream_mjpg_counts_a_viewer_while_connected_and_releases_on_disconnect(monkeypatch):
    fake_grabber = MagicMock()
    fake_grabber.latest_jpeg.return_value = b"jpeg-bytes"
    fake_grabber.frame_interval = 0.01
    monkeypatch.setattr(stream_server, "grabber", fake_grabber)

    server, thread = _start_test_server()
    try:
        port = server.server_address[1]
        resp = urllib.request.urlopen(f"http://127.0.0.1:{port}/stream.mjpg", timeout=2)
        # Read a little of the stream to make sure the handler thread has
        # actually entered its loop (and called viewer_connected()) before
        # disconnecting.
        resp.read(20)
        resp.close()
        deadline = time.time() + 2.0
        while fake_grabber.viewer_disconnected.call_count == 0 and time.time() < deadline:
            time.sleep(0.02)
    finally:
        server.shutdown()
        server.server_close()

    fake_grabber.viewer_connected.assert_called_once()
    fake_grabber.viewer_disconnected.assert_called_once()
