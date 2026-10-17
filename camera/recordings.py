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

UPDATE (2026-10-17): the videos recorded by the first version of this
module (default fourcc "mp4v") never actually played in a browser -- they
opened fine with cv2.VideoWriter and produced non-empty .mp4 files, so the
2026-09-18 honesty note's "if recordings come out empty" diagnostic never
fired, but "mp4v" is MPEG-4 Part 2 ("simple profile"), which is NOT the
same thing as H.264/AVC ("avc1") despite both living happily inside an
.mp4 container -- and no mainstream browser's <video> element decodes
MPEG-4 Part 2 natively. That silent mismatch (file opens, bytes get
written, the web page's own <video onerror> handler just quietly treats
it as "camera unavailable" -- see robot-webserver's _media_page()) was the
actual root cause.

Asking cv2.VideoWriter for "avc1"/"h264" directly was tried and rejected:
OpenCV's bundled FFmpeg build (whether from opencv-python-headless's own
wheel or most Linux distro packages) ships with NO libx264 support at all,
for licensing reasons -- VideoWriter.isOpened() simply comes back False
for an H.264 fourcc on most installs, which would turn today's "plays the
wrong codec" bug into tomorrow's "recording silently does nothing" bug.

The fix keeps capturing with "MJPG" (CAPTURE_FOURCC below) into a
temporary .avi file -- Motion JPEG is the one codec this project's own
2026-09-18 docstring already confirmed needs no extra system codec
package and reliably opens -- and then shells out to the system `ffmpeg`
binary (a real, separate install: `sudo apt install ffmpeg` once on the
Pi; most Raspberry Pi OS / Debian ffmpeg packages DO ship with libx264,
unlike OpenCV's own bundled copy) to transcode that into the final,
browser-playable H.264/yuv420p .mp4 that is actually exposed under the
public filename. See VideoRecorder._finalize() below.

If `ffmpeg` isn't installed (or the transcode fails for any reason), this
falls back to keeping the raw MJPG/.avi capture under the public .mp4
name -- no worse than this module's pre-2026-10-17 behaviour (still not
guaranteed to play in a browser), never silently losing the recording,
and a clear log line tells the operator exactly how to fix it for real.

Honesty note (same caveat as the rest of this project's hardware-facing
code): the MJPG capture step is written against cv2.VideoWriter's
documented, stable API (already used successfully elsewhere for reading
-- cv2.VideoCapture, in stream_server.py's FrameGrabber) and the ffmpeg
transcode step against ffmpeg's own long-stable CLI flags, but neither
has been run against a real camera/ffmpeg-on-a-Pi combination. Also worth
knowing: stop() now blocks for roughly as long as the transcode takes
(typically well under real time for a short clip on a Pi 4/5, since x264
"veryfast" encodes faster than it decodes at this resolution) -- this is
synchronous for simplicity, matching this project's other HTTP handlers
(e.g. /snap's _wait_for_frame()); if that's noticeably slow on your Pi,
moving the transcode to a background thread would be the next step, not
a sign the rest of this code is wrong.
"""
import itertools
import logging
import os
import shutil
import subprocess
import time

import cv2

log = logging.getLogger(__name__)

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

# What FrameGrabber's raw frames are actually captured with (see this
# module's 2026-10-17 update note): MJPG/.avi needs no system codec
# package and reliably opens with OpenCV's own bundled FFmpeg build, unlike
# "mp4v" (plays in nothing) or "avc1"/"h264" (usually fails to open at all).
CAPTURE_FOURCC = "MJPG"
CAPTURE_EXTENSION = "avi"

# The PUBLIC filename extension every recording is exposed under --
# unchanged since 2026-09-18, so stream_server.py's /recordings route,
# robot-webserver's media routes, and tests/test_recordings.py all keep
# working exactly as before; what changed is what bytes actually end up
# there (see _finalize() below).
DEFAULT_EXTENSION = "mp4"
DEFAULT_FPS = 15.0  # matches this project's own CAMERA_FPS default

FFMPEG_BIN = "ffmpeg"


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
    reports no file was written -- there's nothing to encode.

    Since 2026-10-17 (see this module's docstring), frames are captured
    into a temporary MJPG/.avi file and stop() transcodes that into the
    final, browser-playable .mp4 exposed under the public filename."""

    def __init__(self, directory=DEFAULT_RECORDING_DIR, max_recordings=MAX_RECORDINGS,
                 fourcc=CAPTURE_FOURCC, fps=DEFAULT_FPS,
                 transcode_to_h264=True, ffmpeg_bin=FFMPEG_BIN):
        self.directory = directory
        self.max_recordings = max_recordings
        self.fourcc = fourcc
        self.fps = fps
        self.transcode_to_h264 = transcode_to_h264
        self.ffmpeg_bin = ffmpeg_bin
        os.makedirs(self.directory, exist_ok=True)
        self._armed = False
        self._writer = None
        self._public_filename = None
        self._raw_filename = None

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
        codecs rather than a clean error. Writes to a temporary
        CAPTURE_EXTENSION file -- see this module's docstring -- stop()
        below produces the final public .mp4 from it."""
        if not self._armed:
            return
        if self._writer is None:
            height, width = frame.shape[:2]
            timestamp_tag = f"{time.strftime('%Y%m%d_%H%M%S')}_{next(_sequence):06d}"
            public_filename = f"rec_{timestamp_tag}.{DEFAULT_EXTENSION}"
            raw_filename = f"rec_{timestamp_tag}.{CAPTURE_EXTENSION}"
            raw_path = os.path.join(self.directory, raw_filename)
            fourcc_code = cv2.VideoWriter_fourcc(*self.fourcc)
            writer = cv2.VideoWriter(raw_path, fourcc_code, self.fps, (width, height))
            if not writer.isOpened():
                # Same "never crash the capture thread over a hardware/
                # codec quirk" reasoning as everywhere else in this
                # project -- stop() below reports this cleanly (no file
                # was actually written) rather than write() raising here,
                # which would kill FrameGrabber's whole capture loop.
                self._armed = False
                return
            self._writer = writer
            self._public_filename = public_filename
            self._raw_filename = raw_filename
        self._writer.write(frame)

    def stop(self):
        """Disarms recording, closes the raw capture file (if one was
        actually opened -- see write() above: start()+stop() with no
        camera frame ever arriving in between, e.g. no camera plugged in
        at all, is a real possibility this handles cleanly), and produces
        the final public .mp4 from it (transcoded to H.264 via ffmpeg when
        possible -- see _finalize() below, and this module's docstring for
        why that step exists). Returns the public filename that was
        written, or None if nothing was. Prunes down to max_recordings the
        same way camera/snapshots.py's SnapshotStore does for its own
        files."""
        self._armed = False
        public_filename = self._public_filename
        raw_filename = self._raw_filename
        writer = self._writer
        self._writer = None
        self._public_filename = None
        self._raw_filename = None

        if writer is None:
            return None
        writer.release()

        raw_path = os.path.join(self.directory, raw_filename)
        public_path = os.path.join(self.directory, public_filename)
        produced = self.transcode_to_h264 and self._finalize(raw_path, public_path)
        if not produced:
            try:
                os.replace(raw_path, public_path)
            except OSError:
                log.exception(
                    "VideoRecorder: failed to keep the raw recording %s after "
                    "a failed/skipped transcode", raw_filename,
                )
                self._prune()
                return None

        self._prune()
        return public_filename

    def _finalize(self, raw_path, public_path):
        """Transcodes the just-recorded raw MJPG/.avi capture at
        `raw_path` into a browser-playable H.264/yuv420p .mp4 at
        `public_path`, using the system `ffmpeg` binary -- NOT
        cv2.VideoWriter, which has no libx264 support in its bundled
        FFmpeg build on most installs (see this module's 2026-10-17
        update note). `-movflags +faststart` moves the moov atom to the
        front so a browser can start playing before the whole file has
        downloaded -- matters here since robot-webserver proxies/caches
        these over what may be a slow mobile hotspot link.

        Returns True if `public_path` was actually produced, False
        otherwise (ffmpeg missing, or the transcode failed) -- never
        raises; `raw_path` is left untouched on failure so stop() can fall
        back to keeping it."""
        if shutil.which(self.ffmpeg_bin) is None:
            log.warning(
                "VideoRecorder: '%s' introuvable -- les enregistrements seront "
                "gardes tels quels (MJPG/.avi) et ne joueront probablement pas "
                "dans un navigateur. Installer une fois avec : sudo apt install ffmpeg",
                self.ffmpeg_bin,
            )
            return False

        try:
            result = subprocess.run(
                [
                    self.ffmpeg_bin, "-y", "-i", raw_path,
                    "-an",  # no audio track is ever captured -- nothing to carry over
                    "-c:v", "libx264", "-preset", "veryfast", "-crf", "23",
                    "-pix_fmt", "yuv420p", "-movflags", "+faststart",
                    public_path,
                ],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
            )
        except OSError:
            log.exception("VideoRecorder: echec du lancement de ffmpeg")
            return False

        if result.returncode != 0 or not os.path.isfile(public_path):
            log.warning(
                "VideoRecorder: transcodage ffmpeg echoue (code %s): %s",
                result.returncode,
                result.stderr.decode("utf-8", "replace")[-500:],
            )
            return False

        try:
            os.remove(raw_path)
        except OSError:
            pass  # transcode already succeeded -- a leftover temp file is harmless
        return True

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

    def delete(self, filename) -> bool:
        """Deletes one recording by name ahead of its natural FIFO
        rotation -- same reasoning and same validation-against-
        list_files() as camera/snapshots.py's SnapshotStore.delete()
        above; backs camera/stream_server.py's DELETE
        /recordings/<filename>. Returns True if a file was actually
        removed, False otherwise (name not currently present)."""
        if filename not in self.list_files():
            return False
        try:
            os.remove(os.path.join(self.directory, filename))
            return True
        except OSError:
            return False

    def list_files(self):
        """Recording filenames currently on disk, newest first -- mirrors
        SnapshotStore.list_files() (same reasoning: a UI listing wants
        newest first, and the stream server's file-serving route uses this
        to validate a requested filename against what's actually still
        present, since a name can go stale between one listing and the
        next request -- this store keeps at most max_recordings files)."""
        return list(reversed(self._existing_files()))
