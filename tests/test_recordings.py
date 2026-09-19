"""Tests for camera.recordings.VideoRecorder -- no real camera/codec
needed, cv2.VideoWriter is mocked out (same "mock cv2 directly, cv2 IS
installed in this sandbox unlike evdev" approach as
tests/test_stream_server.py uses for cv2.VideoCapture).

Run with:
    pytest
"""
import os
from unittest.mock import patch

from camera.recordings import VideoRecorder


class _FakeFrame:
    """Stands in for a numpy frame from cv2.VideoCapture.read() -- only
    .shape is ever read by VideoRecorder.write()."""
    def __init__(self, height=480, width=640):
        self.shape = (height, width, 3)


class _FakeWriter:
    def __init__(self, path, fourcc, fps, size, opened=True):
        self.path = path
        self.fourcc = fourcc
        self.fps = fps
        self.size = size
        self._opened = opened
        self.written_frames = []
        self.released = False
        if opened:
            # Real cv2.VideoWriter creates the file on disk as soon as
            # it's successfully opened -- touch an empty one here too, so
            # VideoRecorder._existing_files()/count()/_prune() (which
            # list the real directory) see it, same as they would for a
            # real recording.
            open(path, "wb").close()

    def isOpened(self):
        return self._opened

    def write(self, frame):
        self.written_frames.append(frame)

    def release(self):
        self.released = True


def test_write_before_start_is_a_no_op(tmp_path):
    recorder = VideoRecorder(directory=str(tmp_path))
    with patch("camera.recordings.cv2.VideoWriter") as mock_writer_cls:
        recorder.write(_FakeFrame())
    mock_writer_cls.assert_not_called()
    assert recorder.is_recording is False


def test_start_then_write_opens_a_writer_sized_to_the_frame(tmp_path):
    recorder = VideoRecorder(directory=str(tmp_path), fps=15.0)
    fake_writer = _FakeWriter(str(tmp_path / "fake.mp4"), "fourcc", 15.0, (640, 480))
    frame = _FakeFrame(height=480, width=640)
    with patch("camera.recordings.cv2.VideoWriter", return_value=fake_writer) as mock_writer_cls, \
         patch("camera.recordings.cv2.VideoWriter_fourcc", return_value="fourcc-code"):
        recorder.start()
        assert recorder.is_recording is True
        recorder.write(frame)

    # (width, height) order -- cv2.VideoWriter expects (width, height),
    # and frame.shape is (height, width, channels).
    _, _, _, size = mock_writer_cls.call_args[0]
    assert size == (640, 480)
    assert fake_writer.written_frames == [frame]


def test_write_reuses_the_same_writer_for_subsequent_frames(tmp_path):
    recorder = VideoRecorder(directory=str(tmp_path))
    fake_writer = _FakeWriter(str(tmp_path / "fake.mp4"), "fourcc", 15.0, (640, 480))
    with patch("camera.recordings.cv2.VideoWriter", return_value=fake_writer) as mock_writer_cls, \
         patch("camera.recordings.cv2.VideoWriter_fourcc", return_value="fourcc-code"):
        recorder.start()
        recorder.write(_FakeFrame())
        recorder.write(_FakeFrame())
        recorder.write(_FakeFrame())

    assert mock_writer_cls.call_count == 1  # opened once, not once per frame
    assert len(fake_writer.written_frames) == 3


def test_stop_releases_the_writer_and_returns_the_filename(tmp_path):
    recorder = VideoRecorder(directory=str(tmp_path))
    writers = []
    with patch("camera.recordings.cv2.VideoWriter",
               side_effect=lambda *a: writers.append(_FakeWriter(*a)) or writers[-1]), \
         patch("camera.recordings.cv2.VideoWriter_fourcc", return_value="fourcc-code"):
        recorder.start()
        recorder.write(_FakeFrame())
        filename = recorder.stop()

    assert writers[0].released is True
    assert filename is not None
    assert filename.startswith("rec_")
    assert recorder.is_recording is False
    assert recorder.count() == 1


def test_stop_before_any_frame_arrived_writes_nothing(tmp_path):
    # start() + stop() with no camera frame ever arriving in between --
    # e.g. no camera plugged in at all while "recording" was armed.
    recorder = VideoRecorder(directory=str(tmp_path))
    recorder.start()
    filename = recorder.stop()

    assert filename is None
    assert recorder.count() == 0
    assert recorder.is_recording is False


def test_write_disarms_cleanly_when_the_writer_fails_to_open(tmp_path):
    recorder = VideoRecorder(directory=str(tmp_path))
    fake_writer = _FakeWriter("path", "fourcc", 15.0, (640, 480), opened=False)
    with patch("camera.recordings.cv2.VideoWriter", return_value=fake_writer), \
         patch("camera.recordings.cv2.VideoWriter_fourcc", return_value="fourcc-code"):
        recorder.start()
        recorder.write(_FakeFrame())  # writer.isOpened() is False -- must not raise

    assert recorder.is_recording is False  # disarmed, not left in a broken "recording" state
    assert fake_writer.written_frames == []


def test_never_keeps_more_than_max_recordings(tmp_path):
    recorder = VideoRecorder(directory=str(tmp_path), max_recordings=3)
    with patch("camera.recordings.cv2.VideoWriter",
               side_effect=lambda *a: _FakeWriter(*a)), \
         patch("camera.recordings.cv2.VideoWriter_fourcc", return_value="fourcc-code"):
        for _ in range(5):
            recorder.start()
            recorder.write(_FakeFrame())
            recorder.stop()

    assert recorder.count() == 3


def test_directory_is_created_if_missing(tmp_path):
    target = str(tmp_path / "does" / "not" / "exist" / "yet")
    recorder = VideoRecorder(directory=target)
    assert os.path.isdir(target)


def test_start_is_idempotent_while_already_recording(tmp_path):
    recorder = VideoRecorder(directory=str(tmp_path))
    fake_writer = _FakeWriter(str(tmp_path / "fake.mp4"), "fourcc", 15.0, (640, 480))
    with patch("camera.recordings.cv2.VideoWriter", return_value=fake_writer) as mock_writer_cls, \
         patch("camera.recordings.cv2.VideoWriter_fourcc", return_value="fourcc-code"):
        recorder.start()
        recorder.write(_FakeFrame())
        recorder.start()  # already recording -- must not open a second file
        recorder.write(_FakeFrame())

    assert mock_writer_cls.call_count == 1
    assert len(fake_writer.written_frames) == 2


def test_list_files_returns_newest_first(tmp_path):
    recorder = VideoRecorder(directory=str(tmp_path), max_recordings=5)
    filenames = []
    with patch("camera.recordings.cv2.VideoWriter",
               side_effect=lambda *a: _FakeWriter(*a)), \
         patch("camera.recordings.cv2.VideoWriter_fourcc", return_value="fourcc-code"):
        for _ in range(3):
            recorder.start()
            recorder.write(_FakeFrame())
            filenames.append(recorder.stop())

    assert recorder.list_files() == list(reversed(filenames))


def test_list_files_empty_when_nothing_recorded(tmp_path):
    recorder = VideoRecorder(directory=str(tmp_path))
    assert recorder.list_files() == []
