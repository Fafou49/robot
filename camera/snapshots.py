"""Capped-size storage for camera snapshots (the CAM,SNAP command -- see
pages/protocole_controle.html in the robot-webserver repo).

Snapshots are meant for short-lived downstream processing (e.g. an image
pipeline reading the most recent frames), not as a permanent photo
library, so the store never keeps more than MAX_SNAPSHOTS files: saving a
new one past that limit deletes the oldest first (a small rolling FIFO
buffer, not a growing archive).
"""
import itertools
import os
import time

MAX_SNAPSHOTS = 5

# A per-process, ever-increasing counter appended to every filename --
# real-world snapshots (triggered by CAM,SNAP over the network) are
# seconds apart at most, but rapid successive saves (e.g. this module's
# own tests) can land within the same wall-clock millisecond, and two
# snapshots must never collide on the same filename and silently
# overwrite one another.
_sequence = itertools.count()

# Relative to this file (camera/) by default, so it works the same way
# whether the camera package is run from the repo root or installed
# elsewhere -- override with CAMERA_SNAPSHOT_DIR if a different location
# is wanted (e.g. a tmpfs mount to spare the SD card).
DEFAULT_SNAPSHOT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "tmp")


class SnapshotStore:
    """Saves JPEG bytes as timestamped files under `directory`, pruning
    the oldest ones so at most `max_snapshots` remain. Not thread-safe by
    itself -- callers (the stream server's request handler) are expected
    to serialize access, same as FrameGrabber's own lock does for reads."""

    def __init__(self, directory=DEFAULT_SNAPSHOT_DIR, max_snapshots=MAX_SNAPSHOTS):
        self.directory = directory
        self.max_snapshots = max_snapshots
        os.makedirs(self.directory, exist_ok=True)

    def _existing_files(self):
        """Snapshot files in `directory`, oldest first."""
        names = [f for f in os.listdir(self.directory) if f.startswith("snap_") and f.endswith(".jpg")]
        return sorted(names)  # timestamped names sort chronologically

    def save(self, jpeg_bytes: bytes) -> str:
        """Writes one snapshot and prunes down to max_snapshots. Returns
        the filename (not the full path) that was written."""
        # Timestamp plus a strictly-increasing sequence number, so two
        # snapshots taken within the same wall-clock millisecond still get
        # distinct, still-chronologically-sortable filenames (see
        # _sequence above -- relying on the clock alone isn't enough).
        filename = f"snap_{time.strftime('%Y%m%d_%H%M%S')}_{next(_sequence):06d}.jpg"
        path = os.path.join(self.directory, filename)
        with open(path, "wb") as f:
            f.write(jpeg_bytes)

        existing = self._existing_files()
        overflow = len(existing) - self.max_snapshots
        for old_name in existing[:max(overflow, 0)]:
            try:
                os.remove(os.path.join(self.directory, old_name))
            except OSError:
                pass  # already gone -- fine, not worth failing the snapshot over

        return filename

    def count(self) -> int:
        return len(self._existing_files())
