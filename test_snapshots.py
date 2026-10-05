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


def test_list_files_returns_newest_first(tmp_path):
    store = SnapshotStore(directory=str(tmp_path), max_snapshots=5)
    written = [store.save(f"frame{i}".encode()) for i in range(3)]
    assert store.list_files() == list(reversed(written))


def test_list_files_reflects_pruning(tmp_path):
    store = SnapshotStore(directory=str(tmp_path), max_snapshots=5)
    written = [store.save(f"frame{i}".encode()) for i in range(8)]
    # Only the 5 most recent remain, still newest first.
    assert store.list_files() == list(reversed(written[3:]))


def test_list_files_empty_when_nothing_saved(tmp_path):
    store = SnapshotStore(directory=str(tmp_path), max_snapshots=5)
    assert store.list_files() == []


# --- delete() (2026-10-05, backs the /control map's right-click delete) ----

def test_delete_removes_the_file_and_returns_true(tmp_path):
    store = SnapshotStore(directory=str(tmp_path), max_snapshots=5)
    filename = store.save(b"jpeg-bytes")
    assert store.delete(filename) is True
    assert not os.path.isfile(os.path.join(str(tmp_path), filename))
    assert filename not in store.list_files()


def test_delete_an_already_gone_file_returns_false(tmp_path):
    store = SnapshotStore(directory=str(tmp_path), max_snapshots=5)
    filename = store.save(b"jpeg-bytes")
    store.delete(filename)
    assert store.delete(filename) is False


def test_delete_a_name_that_was_never_saved_returns_false(tmp_path):
    store = SnapshotStore(directory=str(tmp_path), max_snapshots=5)
    assert store.delete("snap_never_existed.jpg") is False


def test_delete_leaves_other_files_untouched(tmp_path):
    store = SnapshotStore(directory=str(tmp_path), max_snapshots=5)
    keep = store.save(b"keep-me")
    drop = store.save(b"drop-me")
    store.delete(drop)
    assert store.list_files() == [keep]
