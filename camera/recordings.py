"""Capped-size storage for camera video recordings (the CAM,REC_START /
CAM,REC_STOP commands -- see link/robot_state.py's camera_command() and
pages/protocole_controle.html in the robot-webserver repo).

UPDATE (2026-09-18): REC_START/REC_STOP used to always raise
CommandError("09", "CAM_NOT_IMPLEMENTED:...") -- no recording code existed
in this project at all. VideoRecorder below is the actual implementation,
built to mirror camera/snapshots.py's SnapshotStore as closely as
recording allows: same rolling-FIFO cap (a handheld field robot's SD card
is small, and recordings are much bigger than a single JPEG snapshot, so
keeping only the most recent few matters even more here), same "never
raise out of a caller that isn't the one holding the camera open" spirit.

Honesty note (same caveat as the rest of this project's hardware-facing
code): written against cv2.VideoWriter's documented, stable API (already
used successfully elsewhere for reading -- cv2.VideoCapture, in
stream_server.py's FrameGrabber -- but this is this project's first use
of VideoWriter) but not run against a real camera/codec combination.

UPDATE (2026-09-21) -- real bug found and fixed, field report "the 2
videos I recorded don't play": the FOURCC this module used to hardcode,
"mp4v" (MPEG-4 Part 2), is one of the few codecs that reliably opens with
OpenCV's own bundled FFmpeg build with no extra system codec package --
which is exactly why recording itself worked (files were written, showed
up in the Recordings panel's list, non-empty) -- but it is NOT one of the
codecs any mainstream browser's native <video> element can decode
(Chrome/Firefox/Safari all need H.264/H.265, VP8/VP9, or AV1 -- MPEG-4
Part 2 is none of those). The file was perfectly valid, just not
browser-playable, which reads exactly like "the video won't launch" in
robot-webserver's Media page. Two changes fix this:

1. write() below now tries a short list of FourCC codes in order
   (PREFERRED_FOURCCS), H.264 aliases first, "mp4v" last as a final
   fallback so recording still produces SOME file rather than nothing if
   the Pi's OpenCV/FFmpeg build has no H.264 encoder available (pip's
   opencv-python wheels usually don't, for libx264 licensing reasons; a
   system apt-installed python3-opencv often does).
2. stop() now calls _maybe_transcode_to_h264() as a safety net: if the
   codec that actually ended up opening wasn't already H.264, and the
   `ffmpeg` command-line tool is available on the Pi's PATH (a much more
   commonly available H.264 encoder than OpenCV's own bundled one --
   Raspberry Pi OS "with desktop" typically has it, or `sudo apt install
   ffmpeg` for one line), it re-encodes the file in place to real H.264
   before it's ever exposed as a "finished" recording. This is what
   actually guarantees browser playback regardless of which FourCC
   OpenCV itself could open -- if ffmpeg isn't installed, the original
   file is kept as-is (unplayable in-browser, but still a valid file a
   desktop player like VLC can open, same as before this fix).
"""
import itertools
import os
import shutil
import subprocess
import time

import cv2

MAX_RECORDINGS = 5  # same rolling-FIFO idea as camera/snapshots.py's SnapshotStore

# A per-process, ever-increasing counter appended to every filename -- same
# reasoning as camera/snapshots.py's own _sequence: two recordings
# started/stopped within the same wall-clock second (a quick double-tap of
# the gamepad's record button) must never collide on one filename.
_sequence = itertools.count()

# Relative to this file (camera/) by default, same "works the same way
# whether run from the repo root or installed elsewhere" reasoning as
# camera/snapshots.py's DEFAULT_SNAPSHOT_DIR -- override with
# CAMERA_RECORDING_DIR if a different location is wanted (e.g. a larger
# external drive, since video files are much bigger than JPEG snapshots).
DEFAULT_RECORDING_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "recordings")

DEFAULT_FOURCC = "mp4v"
DEFAULT_EXTENSION = "mp4"
DEFAULT_FPS = 15.0  # matches this project's own CAMERA_FPS default

# FourCC codes tried in order when opening the VideoWriter (2026-09-21,
# see this module's own docstring for the full story): H.264 aliases
# first -- genuinely browser-playable if OpenCV's FFmpeg build supports
# them -- "mp4v" (DEFAULT_FOURCC) last, as a fallback that's virtually
# guaranteed to open but needs _maybe_transcode_to_h264() below to become
# browser-playable after the fact. A caller passing an explicit fourcc=
# to VideoRecorder() (e.g. this module's own earlier troubleshooting
# suggestion, fourcc="MJPG") still gets exactly that single codec tried,
# no fallback list -- see __init__ below.
PREFERRED_FOURCCS = ("avc1", "h264", DEFAULT_FOURCC)

# FourCC codes considered ALREADY browser-playable -- write() recording
# with one of these skips _maybe_transcode_to_h264() entirely on stop(),
# since there's nothing to fix.
_BROWSER_SAFE_FOURCCS = {"avc1", "h264", "x264", "avc3"}


def _maybe_transcode_to_h264(path, opened_fourcc, timeout=120):
    """Best-effort, in-place re-encode of `path` to real H.264 via the
    `ffmpeg` command-line tool, if `opened_fourcc` (whichever FourCC
    write() actually managed to open the file with, see PREFERRED_FOURCCS
    above) isn't already one browsers can decode natively. Called from
    stop() below, once, after the file is fully written and closed.

    Deliberately shells out to the `ffmpeg` BINARY rather than trying to
    get OpenCV itself to write H.264 in the first place: whether
    cv2.VideoWriter can open an H.264 FourCC depends on how OpenCV's own
    bundled FFmpeg was built (pip's opencv-python wheels are usually built
    WITHOUT libx264, a GPL-licensed encoder, so "avc1"/"h264" often fail
    to open even when the FourCC is spelled correctly) -- whereas a
    system `ffmpeg` install (apt, or already present on Raspberry Pi OS
    "with desktop") almost always DOES ship libx264, since Debian's ffmpeg
    package isn't subject to the same distribution constraint OpenCV's
    upstream project holds itself to.

    Silent no-op (returns without touching the file) if: the FourCC that
    was actually used is already browser-safe (nothing to do), or `ffmpeg`
    isn't on PATH (nothing this process can do about it -- same "degrade
    to whatever's actually available" spirit as the rest of this
    project's hardware-facing code; the file stays exactly as
    VideoWriter left it, playable in a desktop player like VLC even if
    not in a browser). Never raises: a transcode failure (bad input,
    ffmpeg crash, timeout) leaves the original file untouched rather than
    losing a recording the operator may have no way to re-shoot."""
    if opened_fourcc in _BROWSER_SAFE_FOURCCS:
        return
    ffmpeg_bin = shutil.which("ffmpeg")
    if ffmpeg_bin is None:
        return

    transcoded_path = path + ".h264.tmp"
    try:
        result = subprocess.run(
            [
                ffmpeg_bin, "-y", "-i", path,
                "-c:v", "libx264", "-pix_fmt", "yuv420p",
                # Moves the moov atom to the front of the file (rather
                # than FFmpeg's default of writing it last, once the full
                # length is known) so a browser can start playing before
                # the whole file has downloaded -- standard practice for
                # any MP4 served progressively, which is exactly how
                # robot-webserver's media_recording_file() proxy serves
                # these.
                "-movflags", "+faststart",
                # No audio track exists on this project's camera pipeline
                # (frames only, see FrameGrabber) -- explicit -an rather
                # than relying on ffmpeg to notice there's nothing to
                # copy, so a future frame source that DOES carry audio
                # doesn't silently start including it here unnoticed.
                "-an",
                transcoded_path,
            ],
            capture_output=True,
            timeout=timeout,
        )
        if result.returncode != 0 or not os.path.isfile(transcoded_path):
            return  # leave the original file as-is -- still a valid, if not browser-playable, recording
        os.replace(transcoded_path, path)
    except (OSError, subprocess.SubprocessError):
        return
    finally:
        # Cleans up a half-written temp file on failure/timeout -- os.replace()
        # above already consumed it on the success path, so this is a no-op then.
        if os.path.isfile(transcoded_path):
            try:
                os.remove(transcoded_path)
            except OSError:
                pass


class VideoRecorder:
    """Records frames handed to it one at a time -- by FrameGrabber's own
    capture loop, see stream_server.py's FrameGrabber(recorder=...) -- into
    a timestamped video file, while start()/stop() are called from a
    completely different thread (StreamHandler's /rec/start and /rec/stop,
    themselves called by link/robot_state.py's camera_command() over HTTP
    -- same cross-process split CAM,SNAP already uses). Not built to
    tolerate arbitrary concurrent callers -- only this specific pairing
    (one background capture thread calling write(), one HTTP handler
    thread calling start()/stop()), same as this project's other
    hardware-facing classes (e.g. GamepadReader's rumble state).

    The actual cv2.VideoWriter can only be opened once a frame's real
    width/height is known, so start() just arms recording (remembers
    nothing about frame size yet) and the VideoWriter itself is lazily
    created on the FIRST write() call after that -- mirroring
    camera/stream_server.py's FrameGrabber, which already opens its
    capture device lazily for the same "don't assume, use what's actually
    there" reason. If the camera never delivers a single frame while
    "recording" is armed (unplugged mid-recording, say), stop() simply
    reports no file was written -- there's nothing to encode."""

    def __init__(self, directory=DEFAULT_RECORDING_DIR, max_recordings=MAX_RECORDINGS,
                 fourcc=None, fps=DEFAULT_FPS):
        self.directory = directory
        self.max_recordings = max_recordings
        # fourcc=None (2026-09-21, new default -- was DEFAULT_FOURCC):
        # tries PREFERRED_FOURCCS in order (H.264 first, "mp4v" as a last
        # resort), see this module's own docstring. An explicit fourcc=
        # (this module's own earlier troubleshooting suggestion, e.g.
        # fourcc="MJPG") is still honored as-is -- a single candidate, no
        # fallback list, same as before this change.
        self._fourcc_candidates = PREFERRED_FOURCCS if fourcc is None else (fourcc,)
        self.fps = fps
        os.makedirs(self.directory, exist_ok=True)
        self._armed = False
        self._writer = None
        self._filename = None
        self._opened_fourcc = None

    @property
    def is_recording(self):
        return self._armed

    def start(self):
        """Arms recording. Safe to call again while already recording --
        a no-op, the same file already open just keeps being written to
        (link/robot_state.py's camera_command() only ever calls this once
        per REC_START anyway, guarded by RobotState.is_recording, but
        idempotency is cheap insurance against calling this directly)."""
        self._armed = True

    def write(self, frame):
        """Called by FrameGrabber's capture loop for EVERY frame it reads,
        recording or not -- a no-op the vast majority of the time (not
        armed). Opens the VideoWriter lazily on the first frame after
        start(), sized to THAT frame's real dimensions, since encoding at
        the wrong size silently produces a corrupt/empty file with most
        codecs rather than a clean error."""
        if not self._armed:
            return
        if self._writer is None:
            height, width = frame.shape[:2]
            filename = f"rec_{time.strftime('%Y%m%d_%H%M%S')}_{next(_sequence):06d}.{DEFAULT_EXTENSION}"
            path = os.path.join(self.directory, filename)
            # Tries each candidate FourCC in turn (2026-09-21, was a
            # single hardcoded attempt) -- see PREFERRED_FOURCCS/this
            # module's own docstring for why H.264 is tried first and
            # "mp4v" last. Each failed candidate is release()d before
            # trying the next (a VideoWriter that failed to open still
            # holds no real resource worth keeping around, but calling
            # release() on it is cheap and avoids relying on that being
            # true for every OpenCV/FFmpeg build).
            writer = None
            opened_fourcc = None
            for candidate in self._fourcc_candidates:
                fourcc_code = cv2.VideoWriter_fourcc(*candidate)
                candidate_writer = cv2.VideoWriter(path, fourcc_code, self.fps, (width, height))
                if candidate_writer.isOpened():
                    writer = candidate_writer
                    opened_fourcc = candidate
                    break
                candidate_writer.release()
            if writer is None:
                # Same "never crash the capture thread over a hardware/
                # codec quirk" reasoning as everywhere else in this
                # project -- stop() below reports this cleanly (no file
                # was actually written) rather than write() raising here,
                # which would kill FrameGrabber's whole capture loop.
                self._armed = False
                return
            self._writer = writer
            self._filename = filename
            self._opened_fourcc = opened_fourcc
        self._writer.write(frame)

    def stop(self):
        """Disarms recording and closes the file, if one was actually
        opened (see write() above -- start()+stop() with no camera frame
        ever arriving in between, e.g. no camera plugged in at all, is a
        real possibility this handles cleanly). Returns the filename that
        was written, or None if nothing was. Prunes down to
        max_recordings the same way camera/snapshots.py's SnapshotStore
        does for its own files.

        2026-09-21: also runs _maybe_transcode_to_h264() on the just-closed
        file before pruning -- see that function's own docstring. This
        makes stop() block for as long as the transcode takes (a no-op,
        instant check if the FourCC that actually opened was already
        H.264, or if `ffmpeg` isn't installed) -- acceptable for this
        project's short field-test clips, called once per REC_STOP, not
        per frame."""
        self._armed = False
        filename = self._filename
        if self._writer is not None:
            self._writer.release()
            self._writer = None
            self._filename = None
            _maybe_transcode_to_h264(os.path.join(self.directory, filename), self._opened_fourcc)
            self._opened_fourcc = None
            self._prune()
        return filename

    def _existing_files(self):
        """Recording files in `directory`, oldest first."""
        names = [f for f in os.listdir(self.directory)
                 if f.startswith("rec_") and f.endswith(f".{DEFAULT_EXTENSION}")]
        return sorted(names)  # timestamped names sort chronologically

    def _prune(self):
        existing = self._existing_files()
        overflow = len(existing) - self.max_recordings
        for old_name in existing[:max(overflow, 0)]:
            try:
                os.remove(os.path.join(self.directory, old_name))
            except OSError:
                pass  # already gone -- fine, not worth failing over

    def count(self) -> int:
        return len(self._existing_files())

    def list_files(self):
        """Recording filenames currently on disk, newest first -- mirrors
        SnapshotStore.list_files() (same reasoning: a UI listing wants
        newest first, and the stream server's file-serving route uses this
        to validate a requested filename against what's actually still
        present, since a name can go stale between one listing and the
        next request -- this store keeps at most max_recordings files)."""
        return list(reversed(self._existing_files()))
