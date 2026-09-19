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
of VideoWriter) but not run against a real camera/codec combination. The
FOURCC codec (default "mp4v") is deliberately picked for being one of the
very few that ship working with OpenCV's own bundled FFmpeg build on most
Linux distros with no extra system codec package -- if recordings come
out empty, or VideoWriter.isOpened() is False on the Pi, that's the first
thing to check (try fourcc="MJPG" with a .avi extension instead, which
needs no external codec at all, before assuming the rest of this code is
at fault).
"""
import itertools
import os
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
                 fourcc=DEFAULT_FOURCC, fps=DEFAULT_FPS):
        self.directory = directory
        self.max_recordings = max_recordings
        self.fourcc = fourcc
        self.fps = fps
        os.makedirs(self.directory, exist_ok=True)
        self._armed = False
        self._writer = None
        self._filename = None

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
            fourcc_code = cv2.VideoWriter_fourcc(*self.fourcc)
            writer = cv2.VideoWriter(path, fourcc_code, self.fps, (width, height))
            if not writer.isOpened():
                # Same "never crash the capture thread over a hardware/
                # codec quirk" reasoning as everywhere else in this
                # project -- stop() below reports this cleanly (no file
                # was actually written) rather than write() raising here,
                # which would kill FrameGrabber's whole capture loop.
                self._armed = False
                return
            self._writer = writer
            self._filename = filename
        self._writer.write(frame)

    def stop(self):
        """Disarms recording and closes the file, if one was actually
        opened (see write() above -- start()+stop() with no camera frame
        ever arriving in between, e.g. no camera plugged in at all, is a
        real possibility this handles cleanly). Returns the filename that
        was written, or None if nothing was. Prunes down to
        max_recordings the same way camera/snapshots.py's SnapshotStore
        does for its own files."""
        self._armed = False
        filename = self._filename
        if self._writer is not None:
            self._writer.release()
            self._writer = None
            self._filename = None
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
