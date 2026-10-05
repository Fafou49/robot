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
        # on_demand=False: this test is about the open/retry control flow
        # itself, not about the 2026-10-03 viewer-gating feature (covered
        # by its own tests below) -- a bare start() with nobody calling
        # add_viewer() would otherwise never even attempt to open the
        # device under the new default.
        grabber = FrameGrabber(device=0, width=640, height=480, fps=15, retry_interval=0.05,
                                on_demand=False)
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
        # on_demand=False: this test is about recorder wiring, not about
        # viewer-gating -- without this, fake_recorder (a bare MagicMock)
        # would make _should_capture() pass by accident, since
        # fake_recorder.is_recording is a truthy MagicMock attribute by
        # default rather than the real False/True this feature expects.
        grabber = FrameGrabber(device=0, width=640, height=480, fps=15, recorder=fake_recorder,
                                on_demand=False)
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


# --- FrameGrabber on-demand capture: viewer-gated power saving (2026-10-03) -

def _wait_until(predicate, timeout=2.0, interval=0.02):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if predicate():
            return True
        time.sleep(interval)
    return False


def test_device_stays_closed_with_on_demand_and_no_viewer():
    """The actual point of this feature: with nobody watching /stream.mjpg
    and no recording armed, the capture device must never even be opened
    -- this is what is expected to save power on a UVC webcam (unlike
    pausing the GPS read loop, see this file's module docstring)."""
    frame = MagicMock()
    frame.shape = (480, 640, 3)
    open_calls = {"count": 0}

    def fake_video_capture(device):
        open_calls["count"] += 1
        return _FakeCapture(opened=True, frames=[(True, frame)])

    with patch("camera.stream_server.cv2.VideoCapture", side_effect=fake_video_capture), \
         patch("camera.stream_server.cv2.imencode", return_value=(True, MagicMock(tobytes=lambda: b"x"))):
        grabber = FrameGrabber(device=0, width=640, height=480, fps=15, retry_interval=0.05,
                                on_demand=True)
        grabber.start()
        time.sleep(0.3)  # several retry_interval cycles -- plenty of chances to (wrongly) open
        grabber._running = False

    assert open_calls["count"] == 0
    assert grabber.latest_jpeg() is None


def test_device_opens_once_a_viewer_connects_and_releases_when_it_leaves():
    frame = MagicMock()
    frame.shape = (480, 640, 3)
    captures = []

    def fake_video_capture(device):
        cap = _FakeCapture(opened=True, frames=[(True, frame)])
        captures.append(cap)
        return cap

    with patch("camera.stream_server.cv2.VideoCapture", side_effect=fake_video_capture), \
         patch("camera.stream_server.cv2.imencode", return_value=(True, MagicMock(tobytes=lambda: b"x"))):
        grabber = FrameGrabber(device=0, width=640, height=480, fps=15, retry_interval=0.05,
                                on_demand=True)
        grabber.start()

        grabber.add_viewer()
        assert _wait_until(lambda: grabber.latest_jpeg() is not None), "device never opened for the viewer"
        assert len(captures) == 1

        grabber.remove_viewer()
        assert _wait_until(lambda: captures[0].released), "device never released after the last viewer left"
        assert _wait_until(lambda: grabber.latest_jpeg() is None)

        grabber._running = False


def test_recording_in_progress_keeps_device_open_with_zero_viewers():
    """REC_START must keep producing a video even if nobody has the live
    feed open in a browser at the time -- see _should_capture()."""
    frame = MagicMock()
    frame.shape = (480, 640, 3)

    class _Recorder:
        is_recording = True

        def write(self, frame):
            pass

    with patch("camera.stream_server.cv2.VideoCapture",
               return_value=_FakeCapture(opened=True, frames=[(True, frame)])), \
         patch("camera.stream_server.cv2.imencode", return_value=(True, MagicMock(tobytes=lambda: b"x"))):
        grabber = FrameGrabber(device=0, width=640, height=480, fps=15, retry_interval=0.05,
                                recorder=_Recorder(), on_demand=True)
        grabber.start()
        assert _wait_until(lambda: grabber.latest_jpeg() is not None), \
            "armed recording at 0 viewers should still open the device"
        grabber._running = False


def test_on_demand_false_preserves_previous_always_on_behavior():
    """CAMERA_ON_DEMAND=false escape hatch: device opens with zero viewers
    and no recording, exactly like before this feature existed."""
    frame = MagicMock()
    frame.shape = (480, 640, 3)

    with patch("camera.stream_server.cv2.VideoCapture",
               return_value=_FakeCapture(opened=True, frames=[(True, frame)])), \
         patch("camera.stream_server.cv2.imencode", return_value=(True, MagicMock(tobytes=lambda: b"x"))):
        grabber = FrameGrabber(device=0, width=640, height=480, fps=15, retry_interval=0.05,
                                on_demand=False)
        grabber.start()
        assert _wait_until(lambda: grabber.latest_jpeg() is not None)
        grabber._running = False


# --- StreamHandler: /rec/start, /rec/stop (2026-09-18) ----------------------

def _start_test_server():
    server = http.server.HTTPServer(("127.0.0.1", 0), StreamHandler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    return server, thread


def test_rec_start_arms_the_recorder_when_a_frame_is_already_available(monkeypatch):
    fake_grabber = MagicMock()
    # _handle_rec_start() calls wait_for_frame() (2026-10-03), not a bare
    # latest_jpeg() -- see that method's own tests further down for the
    # real wake-up mechanism against an actual FrameGrabber; this one is
    # only about what _handle_rec_start() does with whatever frame (or
    # None) it gets back.
    fake_grabber.wait_for_frame.return_value = b"jpeg-bytes"
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
    fake_grabber.wait_for_frame.return_value = None  # see the comment in the test above
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


# --- /snap, /rec/start wake a released on-demand camera (2026-10-03) -------
# End-to-end against a REAL FrameGrabber (not a mocked one, unlike the
# tests just above) -- these are the ones that actually exercise
# wait_for_frame()'s wake-up mechanism, covering the gamepad's normal use
# case: SNAP/REC_START with nobody watching /stream.mjpg at all.

def test_snap_wakes_a_released_on_demand_camera(monkeypatch):
    frame = MagicMock()
    frame.shape = (480, 640, 3)

    with patch("camera.stream_server.cv2.VideoCapture",
               return_value=_FakeCapture(opened=True, frames=[(True, frame)])), \
         patch("camera.stream_server.cv2.imencode",
               return_value=(True, MagicMock(tobytes=lambda: b"jpeg-bytes"))):
        real_grabber = FrameGrabber(device=0, width=640, height=480, fps=15, retry_interval=0.05,
                                     on_demand=True)
        real_grabber.start()
        time.sleep(0.2)
        # Confirms the regression this feature could otherwise reintroduce:
        # the camera must genuinely be released before the request below.
        assert real_grabber.latest_jpeg() is None

        fake_store = MagicMock()
        fake_store.save.return_value = "snap_20261003_120000_000001.jpg"
        fake_store.count.return_value = 1
        monkeypatch.setattr(stream_server, "grabber", real_grabber)
        monkeypatch.setattr(stream_server, "snapshot_store", fake_store)

        server, thread = _start_test_server()
        try:
            port = server.server_address[1]
            with urllib.request.urlopen(f"http://127.0.0.1:{port}/snap", timeout=2) as resp:
                assert resp.status == 200
                body = json.loads(resp.read())
        finally:
            server.shutdown()
            server.server_close()
            real_grabber._running = False

    assert body == {"ok": True, "file": "snap_20261003_120000_000001.jpg", "count": 1}


def test_rec_start_wakes_a_released_on_demand_camera_and_keeps_it_open(monkeypatch):
    frame = MagicMock()
    frame.shape = (480, 640, 3)

    class _Recorder:
        def __init__(self):
            self.is_recording = False
            self.started = False

        def start(self):
            self.started = True
            self.is_recording = True

        def write(self, frame):
            pass

    recorder_obj = _Recorder()

    with patch("camera.stream_server.cv2.VideoCapture",
               return_value=_FakeCapture(opened=True, frames=[(True, frame)])), \
         patch("camera.stream_server.cv2.imencode",
               return_value=(True, MagicMock(tobytes=lambda: b"jpeg-bytes"))):
        real_grabber = FrameGrabber(device=0, width=640, height=480, fps=15, retry_interval=0.05,
                                     recorder=recorder_obj, on_demand=True)
        real_grabber.start()
        time.sleep(0.2)
        assert real_grabber.latest_jpeg() is None

        monkeypatch.setattr(stream_server, "grabber", real_grabber)
        monkeypatch.setattr(stream_server, "recorder", recorder_obj)

        server, thread = _start_test_server()
        try:
            port = server.server_address[1]
            with urllib.request.urlopen(f"http://127.0.0.1:{port}/rec/start", timeout=2) as resp:
                assert resp.status == 200
                body = json.loads(resp.read())
            # The device must stay open on its own now that recording is
            # armed -- no transient viewer left over from the request above.
            assert _wait_until(lambda: real_grabber.latest_jpeg() is not None)
        finally:
            server.shutdown()
            server.server_close()
            real_grabber._running = False

    assert body == {"ok": True, "recording": True}
    assert recorder_obj.started is True


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
