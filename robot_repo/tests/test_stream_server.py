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
import time
from unittest.mock import MagicMock, patch

from camera.stream_server import FrameGrabber


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
