#!/usr/bin/env bash
# Boot-time entry point: activates this project's virtual environment,
# then hands off to run_robot.sh (which starts link/server.py and
# camera/stream_server.py side by side -- see that script's own comments).
#
# Why this exists as a SEPARATE script rather than just pointing systemd
# straight at run_robot.sh: `source .../activate` is a shell builtin, not
# an executable -- it only works run FROM a shell (`source x && y`), which
# is exactly what this script's body is. systemd's ExecStart runs a single
# program directly (no shell involved unless you say so), so it can't
# `source` anything by itself. This script IS that shell step, kept in one
# small, separately readable/testable file instead of burying a one-line
# shell snippet inside the systemd unit itself (see systemd/robot.service).
#
# Run by hand (from any directory) to test it exactly like the boot path will:
#   /home/robot/Desktop/coderobot/start_robot.sh
#
# PROJECT_DIR/VENV_DIR below are the two paths to double-check/edit if this
# project ever lives somewhere else on this Pi.
set -eu

PROJECT_DIR="/home/robot/Desktop/coderobot"
VENV_DIR="$PROJECT_DIR/.venv"

# shellcheck disable=SC1091 -- this file only exists once the venv itself
# has actually been created (see README.md, "Installation"), not at lint
# time here.
source "$VENV_DIR/bin/activate"

# exec (not a plain call) replaces THIS script's process with
# run_robot.sh's, instead of keeping this one around as a useless parent
# -- systemd then tracks run_robot.sh directly, and a stop/restart signal
# (systemctl stop/restart, or a reboot) reaches it straight away instead
# of having to be forwarded down through an extra layer.
cd "$PROJECT_DIR"
exec ./run_robot.sh
