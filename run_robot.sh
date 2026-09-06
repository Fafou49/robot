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
# Run from the repo root:
#   ./run_robot.sh
#
# Any environment variable the two scripts read (CAMERA_DEVICE, GPS_DEVICE,
# ROBOT_GPIOCHIP, CONTROL_PORT, CAMERA_PORT, ... -- see README.md) can be
# exported before calling this script, or set once in .env: both scripts
# already load it themselves.

set -u
cd "$(dirname "$0")"

python3 -m link &
LINK_PID=$!

python3 -m camera &
CAMERA_PID=$!

CLEANED=0

# Stops whichever of the two is still alive. Runs on Ctrl+C/SIGTERM, and
# also once either process exits on its own (e.g. a startup crash) -- so a
# camera crash doesn't silently leave link/server.py running by itself
# with no camera feed, unnoticed, same failure mode as the silent
# background-thread crashes already found and fixed elsewhere in this
# project (nothing stops a lone survivor from just running on forever).
cleanup() {
    if [ "$CLEANED" -eq 1 ]; then
        return
    fi
    CLEANED=1
    echo ""
    echo "Stopping link (pid $LINK_PID) and camera (pid $CAMERA_PID)..."
    kill "$LINK_PID" "$CAMERA_PID" 2>/dev/null
    wait "$LINK_PID" "$CAMERA_PID" 2>/dev/null
    echo "Both stopped."
}
trap cleanup INT TERM

echo "link running (pid $LINK_PID), camera running (pid $CAMERA_PID). Press Ctrl+C to stop both."

# Waits for whichever of the two exits first -- either the user's Ctrl+C
# (handled above by the trap, which runs cleanup and stops both before this
# wait returns) or an unexpected crash of one of them (falls through to the
# explicit cleanup call below, which then stops the other one too).
wait -n "$LINK_PID" "$CAMERA_PID"
cleanup
