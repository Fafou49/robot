#!/usr/bin/env bash
# Launches link/server.py (`python3 -m link`, NMEA control server, default
# 0.0.0.0:5050) and camera/stream_server.py (`python3 -m camera`, MJPEG
# stream, default 0.0.0.0:8000) side by side, and stops both cleanly on a
# single Ctrl+C.
#
# `python3 -m link && python3 -m camera` does NOT work for this: `&&` runs
# the second command only after the first exits, but link/server.py runs
# forever (it serves connections until interrupted), so camera/stream_
# server.py never even starts. This script backgrounds both instead.
#
# The camera is OPTIONAL: link/server.py is what actually drives/logs GPS
# and answers the control protocol, so it stays the one critical process.
# If the camera fails to start (no webcam attached, CAMERA_DEVICE wrong,
# device busy...) or crashes later, this script prints a warning and keeps
# link/server.py running rather than tearing everything down -- previously
# either process dying killed both, which meant an unrelated/absent camera
# could take down GPS/driving too. A link/server.py crash is still fatal
# (it's the process that matters) and still stops the camera with it.
# Set CAMERA_ENABLED=0 to skip starting the camera altogether (e.g. no
# webcam on this robot / this run).
#
# Run from the repo root:
#   ./run_robot.sh
#   CAMERA_ENABLED=0 ./run_robot.sh   # link only, no camera at all
#
# Any environment variable the two scripts read (CAMERA_DEVICE, GPS_DEVICE,
# ROBOT_GPIOCHIP, CONTROL_PORT, CAMERA_PORT, ... -- see README.md) can be
# exported before calling this script, or set once in .env: both scripts
# already load it themselves.

set -u
cd "$(dirname "$0")"

CAMERA_ENABLED="${CAMERA_ENABLED:-1}"

python3 -m link &
LINK_PID=$!

CAMERA_PID=""
if [ "$CAMERA_ENABLED" != "0" ] && [ "$CAMERA_ENABLED" != "false" ]; then
    python3 -m camera &
    CAMERA_PID=$!
    echo "link running (pid $LINK_PID), camera running (pid $CAMERA_PID). Press Ctrl+C to stop."
else
    echo "Camera disabled (CAMERA_ENABLED=$CAMERA_ENABLED)."
    echo "link running (pid $LINK_PID). Press Ctrl+C to stop."
fi

CLEANED=0

# Stops whatever is still running. Runs on Ctrl+C/SIGTERM, and also once
# link/server.py exits on its own (startup crash or otherwise) -- the
# camera, if any, is stopped alongside it. A camera-only exit is handled
# separately below (see the wait loop) and never reaches this function.
cleanup() {
    if [ "$CLEANED" -eq 1 ]; then
        return
    fi
    CLEANED=1
    echo ""
    if [ -n "$CAMERA_PID" ]; then
        echo "Stopping link (pid $LINK_PID) and camera (pid $CAMERA_PID)..."
        kill "$LINK_PID" "$CAMERA_PID" 2>/dev/null
        wait "$LINK_PID" "$CAMERA_PID" 2>/dev/null
    else
        echo "Stopping link (pid $LINK_PID)..."
        kill "$LINK_PID" 2>/dev/null
        wait "$LINK_PID" 2>/dev/null
    fi
    echo "Stopped."
}
trap cleanup INT TERM

# Waits for link/server.py, the one process that actually matters. While
# the camera is still up, a single "wait -n" wakes up on EITHER exiting;
# if it turns out to be the camera (link/server.py still alive per
# `kill -0`), that's just a warning -- forget CAMERA_PID and loop back to
# waiting on link/server.py alone. If it's link/server.py, fall through to
# cleanup, which also stops the camera (if it's still around).
while true; do
    if [ -n "$CAMERA_PID" ]; then
        wait -n "$LINK_PID" "$CAMERA_PID"
    else
        wait "$LINK_PID"
    fi

    if ! kill -0 "$LINK_PID" 2>/dev/null; then
        break
    fi

    if [ -n "$CAMERA_PID" ] && ! kill -0 "$CAMERA_PID" 2>/dev/null; then
        echo ""
        echo "[run_robot] Camera process (pid $CAMERA_PID) stopped -- continuing WITHOUT camera (link/server.py keeps running)." >&2
        echo "[run_robot] Check CAMERA_DEVICE / the webcam is plugged in, or set CAMERA_ENABLED=0 to skip it next time." >&2
        CAMERA_PID=""
    fi
done

cleanup
