"""Tests for camera.snapshots.SnapshotStore -- no camera/OpenCV needed,
this only exercises the file-storage/pruning logic in isolation.

Run with:
    pytest
"""
import os

from camera.snapshots import SnapshotStore


def test_save_writes_a_file_under_the_given_directory(tmp_path):
    store = SnapshotStore(directory=str(tmp_path), max_snapshots=5)
    filename = store.save(b"jpeg-bytes")
    assert os.path.isfile(os.path.join(str(tmp_path), filename))
    assert store.count() == 1


def test_content_is_preserved(tmp_path):
    store = SnapshotStore(directory=str(tmp_path), max_snapshots=5)
    filename = store.save(b"hello-jpeg")
    with open(os.path.join(str(tmp_path), filename), "rb") as f:
        assert f.read() == b"hello-jpeg"


def test_never_keeps_more_than_max_snapshots(tmp_path):
    store = SnapshotStore(directory=str(tmp_path), max_snapshots=5)
    for i in range(8):
        store.save(f"frame{i}".encode())
    assert store.count() == 5


def test_oldest_snapshots_are_pruned_first(tmp_path):
    store = SnapshotStore(directory=str(tmp_path), max_snapshots=5)
    written = [store.save(f"frame{i}".encode()) for i in range(8)]
    remaining = sorted(os.listdir(str(tmp_path)))
    # The 5 most recently written filenames should be exactly what's left
    # -- frame0..frame2 (the 3 oldest) were pruned.
    assert remaining == sorted(written[3:])


def test_directory_is_created_if_missing(tmp_path):
    target = str(tmp_path / "does" / "not" / "exist" / "yet")
    store = SnapshotStore(directory=target, max_snapshots=5)
    assert os.path.isdir(target)
    store.save(b"x")
    assert store.count() == 1
