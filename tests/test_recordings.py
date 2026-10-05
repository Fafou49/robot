"""Tests for camera.recordings.VideoRecorder -- no real camera/codec
needed, cv2.VideoWriter is mocked out (same "mock cv2 directly, cv2 IS
installed in this sandbox unlike evdev" approach as
tests/test_stream_server.py uses for cv2.VideoCapture).

Run with:
    pytest
"""
import os
from types import SimpleNamespace
from unittest.mock import patch

from camera.recordings import PREFERRED_FOURCCS, VideoRecorder


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


# --- FourCC fallback (2026-09-21, field report "recordings don't play in
# the browser") -- see PREFERRED_FOURCCS/this module's own docstring. --

def _selective_video_writer(working_fourccs, calls):
    """cv2.VideoWriter side_effect: opens only for a fourcc_code in
    `working_fourccs`, and records every fourcc_code it was called with
    (in order) into `calls` -- lets a test assert both which candidates
    were tried and in what order."""
    def _make(path, fourcc_code, fps, size):
        calls.append(fourcc_code)
        return _FakeWriter(path, fourcc_code, fps, size, opened=(fourcc_code in working_fourccs))
    return _make


def test_write_falls_back_through_fourcc_candidates_to_mp4v(tmp_path):
    # Simulates a Pi whose OpenCV/FFmpeg build has no H.264 encoder at
    # all -- only the last candidate, "mp4v", actually opens.
    recorder = VideoRecorder(directory=str(tmp_path))
    calls = []
    with patch("camera.recordings.cv2.VideoWriter",
               side_effect=_selective_video_writer({"mp4v"}, calls)), \
         patch("camera.recordings.cv2.VideoWriter_fourcc", side_effect=lambda *chars: "".join(chars)):
        recorder.start()
        recorder.write(_FakeFrame())

    assert calls == list(PREFERRED_FOURCCS), calls  # tried every candidate, in order
    assert recorder._opened_fourcc == "mp4v"
    assert recorder.is_recording is True  # armed and actually recording, not disarmed


def test_write_stops_at_the_first_working_fourcc(tmp_path):
    # H.264 works on the first try -- h264/mp4v must never even be
    # attempted (no point, and no reason to risk a second file getting
    # created on disk for the same recording).
    recorder = VideoRecorder(directory=str(tmp_path))
    calls = []
    with patch("camera.recordings.cv2.VideoWriter",
               side_effect=_selective_video_writer({"avc1"}, calls)), \
         patch("camera.recordings.cv2.VideoWriter_fourcc", side_effect=lambda *chars: "".join(chars)):
        recorder.start()
        recorder.write(_FakeFrame())

    assert calls == ["avc1"], calls
    assert recorder._opened_fourcc == "avc1"


def test_write_disarms_when_no_fourcc_candidate_opens(tmp_path):
    # None of the candidates open (e.g. a completely broken FFmpeg build)
    # -- same clean "disarm, no file, no raise" outcome as the pre-existing
    # single-fourcc failure test above, just reached after trying all of
    # them instead of one.
    recorder = VideoRecorder(directory=str(tmp_path))
    calls = []
    with patch("camera.recordings.cv2.VideoWriter",
               side_effect=_selective_video_writer(set(), calls)), \
         patch("camera.recordings.cv2.VideoWriter_fourcc", side_effect=lambda *chars: "".join(chars)):
        recorder.start()
        recorder.write(_FakeFrame())

    assert calls == list(PREFERRED_FOURCCS)
    assert recorder.is_recording is False
    assert recorder._writer is None


def test_explicit_fourcc_is_a_single_candidate_with_no_fallback(tmp_path):
    # An explicit fourcc= (this module's own troubleshooting suggestion,
    # e.g. fourcc="MJPG") must be tried exactly once -- no silent
    # fallback to mp4v if it fails to open, since the caller asked for
    # that specific codec.
    recorder = VideoRecorder(directory=str(tmp_path), fourcc="MJPG")
    calls = []
    with patch("camera.recordings.cv2.VideoWriter",
               side_effect=_selective_video_writer(set(), calls)), \
         patch("camera.recordings.cv2.VideoWriter_fourcc", side_effect=lambda *chars: "".join(chars)):
        recorder.start()
        recorder.write(_FakeFrame())

    assert calls == ["MJPG"]
    assert recorder.is_recording is False


# --- ffmpeg transcode-to-H.264 safety net (2026-09-21) --------------------

def test_stop_transcodes_to_h264_when_opened_codec_is_not_browser_safe(tmp_path):
    recorder = VideoRecorder(directory=str(tmp_path))
    calls = []
    with patch("camera.recordings.cv2.VideoWriter",
               side_effect=_selective_video_writer({"mp4v"}, calls)), \
         patch("camera.recordings.cv2.VideoWriter_fourcc", side_effect=lambda *chars: "".join(chars)):
        recorder.start()
        recorder.write(_FakeFrame())

    def fake_run(cmd, capture_output, timeout):
        # Simulates ffmpeg actually producing the transcoded temp file --
        # real content doesn't matter, only that stop() picks it up and
        # replaces the original with it.
        out_path = cmd[-1]
        with open(out_path, "wb") as f:
            f.write(b"fake-h264-bytes")
        return SimpleNamespace(returncode=0, stdout=b"", stderr=b"")

    with patch("camera.recordings.shutil.which", return_value="/usr/bin/ffmpeg"), \
         patch("camera.recordings.subprocess.run", side_effect=fake_run) as mock_run:
        filename = recorder.stop()

    mock_run.assert_called_once()
    cmd = mock_run.call_args[0][0]
    assert cmd[0] == "/usr/bin/ffmpeg"
    assert "-c:v" in cmd and "libx264" in cmd
    assert "-movflags" in cmd and "+faststart" in cmd
    final_path = os.path.join(str(tmp_path), filename)
    assert os.path.isfile(final_path)
    with open(final_path, "rb") as f:
        assert f.read() == b"fake-h264-bytes"  # original mp4v bytes replaced by the transcode
    # The temp file ffmpeg wrote to must not be left lying around.
    assert not os.path.isfile(final_path + ".h264.tmp")


def test_stop_skips_transcode_when_codec_already_h264(tmp_path):
    recorder = VideoRecorder(directory=str(tmp_path))
    calls = []
    with patch("camera.recordings.cv2.VideoWriter",
               side_effect=_selective_video_writer({"avc1"}, calls)), \
         patch("camera.recordings.cv2.VideoWriter_fourcc", side_effect=lambda *chars: "".join(chars)):
        recorder.start()
        recorder.write(_FakeFrame())

    with patch("camera.recordings.shutil.which") as mock_which, \
         patch("camera.recordings.subprocess.run") as mock_run:
        filename = recorder.stop()

    mock_which.assert_not_called()  # already browser-safe -- nothing to check for, even
    mock_run.assert_not_called()
    assert filename is not None


def test_stop_skips_transcode_when_ffmpeg_not_installed(tmp_path):
    recorder = VideoRecorder(directory=str(tmp_path))
    calls = []
    with patch("camera.recordings.cv2.VideoWriter",
               side_effect=_selective_video_writer({"mp4v"}, calls)), \
         patch("camera.recordings.cv2.VideoWriter_fourcc", side_effect=lambda *chars: "".join(chars)):
        recorder.start()
        recorder.write(_FakeFrame())

    with patch("camera.recordings.shutil.which", return_value=None), \
         patch("camera.recordings.subprocess.run") as mock_run:
        filename = recorder.stop()

    mock_run.assert_not_called()
    # Original mp4v file is left exactly as VideoWriter wrote it -- still
    # a valid recording (just not browser-playable), never lost.
    assert os.path.isfile(os.path.join(str(tmp_path), filename))


def test_stop_leaves_original_file_untouched_when_ffmpeg_fails(tmp_path):
    recorder = VideoRecorder(directory=str(tmp_path))
    calls = []
    with patch("camera.recordings.cv2.VideoWriter",
               side_effect=_selective_video_writer({"mp4v"}, calls)), \
         patch("camera.recordings.cv2.VideoWriter_fourcc", side_effect=lambda *chars: "".join(chars)):
        recorder.start()
        recorder.write(_FakeFrame())

    def failing_run(cmd, capture_output, timeout):
        return SimpleNamespace(returncode=1, stdout=b"", stderr=b"ffmpeg: error")

    with patch("camera.recordings.shutil.which", return_value="/usr/bin/ffmpeg"), \
         patch("camera.recordings.subprocess.run", side_effect=failing_run):
        filename = recorder.stop()  # must not raise despite the failed transcode

    final_path = os.path.join(str(tmp_path), filename)
    assert os.path.isfile(final_path)  # original file (from _FakeWriter's own creation) still there
    assert not os.path.isfile(final_path + ".h264.tmp")


def test_stop_survives_ffmpeg_raising(tmp_path):
    # A crashed/timed-out ffmpeg (subprocess.run raising) must degrade the
    # same way a failed/missing ffmpeg does -- never let stop() itself
    # raise and lose track of an otherwise-successful recording.
    recorder = VideoRecorder(directory=str(tmp_path))
    calls = []
    with patch("camera.recordings.cv2.VideoWriter",
               side_effect=_selective_video_writer({"mp4v"}, calls)), \
         patch("camera.recordings.cv2.VideoWriter_fourcc", side_effect=lambda *chars: "".join(chars)):
        recorder.start()
        recorder.write(_FakeFrame())

    with patch("camera.recordings.shutil.which", return_value="/usr/bin/ffmpeg"), \
         patch("camera.recordings.subprocess.run", side_effect=OSError("boom")):
        filename = recorder.stop()

    assert filename is not None
    assert os.path.isfile(os.path.join(str(tmp_path), filename))
