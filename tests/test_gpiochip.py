"""Tests for motor_control.gpiochip.detect_rp1_gpiochip().

Moved here (2026-09-07, was tests/test_remote_control_gpiochip.py) when
detect_rp1_gpiochip() itself moved out of motor_control/remote_control.py
into its own module (motor_control/gpiochip.py), now that both
motor_control/motor_driver.py and remote_control.py's Remote need it.
Test bodies are unchanged.

Only this one function is tested: it's pure logic (subprocess output
parsing, no actual hardware/GPIO access), so it can run in this sandbox.
gpiod is stubbed out below just enough to import motor_control.gpiochip's
dependency chain -- this does NOT mean any hardware-facing code was
exercised, only that detect_rp1_gpiochip() could be.

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
from unittest.mock import patch

import pytest

# motor_control.gpiochip has no hardware dependencies of its own (just
# os/re/subprocess) -- unlike motor_control.remote_control (its previous
# home), which needs evdev/gpiod stubbed just to be imported. No stubbing
# needed here any more.
from motor_control.gpiochip import detect_rp1_gpiochip


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
