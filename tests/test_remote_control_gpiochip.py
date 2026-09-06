"""Tests for motor_control.remote_control.detect_rp1_gpiochip().

Only this one function is tested: it's pure logic (subprocess output
parsing, no actual hardware/GPIO access), so it can run in this sandbox.
The rest of remote_control.py (Remote, its gamepad/GPIO loop) needs
evdev/pygame/gpiod, none of which are installable here (no PyPI access)
-- same honesty note as the rest of this project's hardware-facing code.
evdev/pygame/gpiod are stubbed out below just enough to import the module
(remote_control.py imports them at module level, unlike link/gps_reader.py
which guards its own hardware imports) -- this does NOT mean the rest of
the module was exercised, only that detect_rp1_gpiochip() could be.

Context: this function exists because the Raspberry Pi 5's user-facing
GPIO chip (RP1) is NOT at a fixed /dev/gpiochipN number -- early Pi 5 OS
images exposed it as gpiochip4, while a kernel/device-tree change (mid-
2024 onward) moved it to gpiochip0 instead, pushing unrelated internal
chips to gpiochip10+. Hardcoding a chip number breaks across an OS
update; identifying it by its "pinctrl-rp1" driver label (as `gpiodetect`
reports it) does not. It returns a FULL device path (e.g. "/dev/
gpiochip0"), not a bare name: confirmed for real on the robot's Pi 5
(2026-09-06) that libgpiod v2's gpiod.Chip() does NOT resolve a bare
"gpiochip0" against /dev/ itself -- it raised FileNotFoundError until the
"/dev/" prefix was added here.
"""
import subprocess
import sys
import types
from unittest.mock import patch

import pytest

# Stub the hardware-only modules remote_control.py imports at module
# level, so importing it here doesn't require evdev/pygame/gpiod to
# actually be installed. Includes gpiod.line (Direction/Value), which
# remote_control.py imports directly since it uses libgpiod v2's API.
for _name in ("evdev", "pygame", "gpiod"):
    sys.modules.setdefault(_name, types.ModuleType(_name))
sys.modules["gpiod"].Chip = lambda *a, **kw: None
_line_stub = types.ModuleType("gpiod.line")
_line_stub.Direction = types.SimpleNamespace(OUTPUT="OUTPUT")
_line_stub.Value = types.SimpleNamespace(ACTIVE="ACTIVE", INACTIVE="INACTIVE")
sys.modules["gpiod.line"] = _line_stub
sys.modules["gpiod"].line = _line_stub

from motor_control.remote_control import detect_rp1_gpiochip  # noqa: E402


@pytest.fixture(autouse=True)
def _clear_env_override(monkeypatch):
    # ROBOT_GPIOCHIP would short-circuit every test below if left set
    # from the environment -- make sure each test starts without it.
    monkeypatch.delenv("ROBOT_GPIOCHIP", raising=False)


def _fake_gpiodetect(stdout):
    return subprocess.CompletedProcess(args=["gpiodetect"], returncode=0, stdout=stdout)


def test_finds_pinctrl_rp1_on_gpiochip0():
    # Current (mid-2024 onward) Raspberry Pi OS layout.
    output = "gpiochip0 [pinctrl-rp1] (54 lines)\ngpiochip10 [pinctrl-bcm2835] (2 lines)\n"
    with patch("subprocess.run", return_value=_fake_gpiodetect(output)):
        assert detect_rp1_gpiochip() == "/dev/gpiochip0"


def test_finds_pinctrl_rp1_on_gpiochip4():
    # Early Pi 5 OS images, before the kernel/device-tree reorder.
    output = "gpiochip0 [pinctrl-bcm2712] (8 lines)\ngpiochip4 [pinctrl-rp1] (54 lines)\n"
    with patch("subprocess.run", return_value=_fake_gpiodetect(output)):
        assert detect_rp1_gpiochip() == "/dev/gpiochip4"


def test_falls_back_when_gpiodetect_missing():
    with patch("subprocess.run", side_effect=FileNotFoundError):
        assert detect_rp1_gpiochip() == "/dev/gpiochip0"
        assert detect_rp1_gpiochip(fallback="/dev/gpiochip4") == "/dev/gpiochip4"


def test_falls_back_when_gpiodetect_errors():
    with patch("subprocess.run", side_effect=subprocess.CalledProcessError(1, ["gpiodetect"])):
        assert detect_rp1_gpiochip() == "/dev/gpiochip0"


def test_falls_back_when_gpiodetect_times_out():
    with patch("subprocess.run", side_effect=subprocess.TimeoutExpired(["gpiodetect"], 5)):
        assert detect_rp1_gpiochip() == "/dev/gpiochip0"


def test_falls_back_when_label_not_found():
    output = "gpiochip0 [some-other-chip] (4 lines)\n"
    with patch("subprocess.run", return_value=_fake_gpiodetect(output)):
        assert detect_rp1_gpiochip(fallback="/dev/gpiochipX") == "/dev/gpiochipX"


def test_detected_path_always_has_dev_prefix():
    # Regression test for the exact bug reported on the real robot:
    # gpiod v2's Chip() does not resolve a bare "gpiochip0" against
    # /dev/ itself, so a path without the prefix silently breaks.
    output = "gpiochip0 [pinctrl-rp1] (54 lines)\n"
    with patch("subprocess.run", return_value=_fake_gpiodetect(output)):
        assert detect_rp1_gpiochip().startswith("/dev/")


def test_env_override_short_circuits_gpiodetect(monkeypatch):
    monkeypatch.setenv("ROBOT_GPIOCHIP", "/dev/gpiochip7")
    with patch("subprocess.run") as mock_run:
        assert detect_rp1_gpiochip() == "/dev/gpiochip7"
        mock_run.assert_not_called()
