"""Tests for motor_control.gps_condition_logger.

Only MultiConditionGPSLogger._writes_for_line() is tested here: it's the
one piece of pure logic in this module (no serial port, no real files,
no hardware) -- the per-line dispatch that decides which condition(s) a
line should be logged under and when a START/END marker is due. The rest
(the actual serial read loop, ConditionGPSLogger's backward-compatible
constructor) needs pyserial and a real Remote/gamepad, so it's not
exercised here -- same honesty note as the rest of this project's
hardware-facing code.
"""
from motor_control.gps_condition_logger import ConditionGPSLogger, MultiConditionGPSLogger


class FakeRemote:
    """No threading.Lock/gamepad needed for these tests -- only
    MultiConditionGPSLogger._read_duty_cycles() would touch `verrou`,
    and _writes_for_line() is called directly with explicit left/right
    values instead, so a plain object with just the two attributes is
    enough."""
    def __init__(self):
        self.dutyCycleLeft = 0
        self.dutyCycleRight = 0


def is_a(left, right):
    return left == 1


def is_b(left, right):
    return right == 2


def _logger_with_two_conditions():
    remote = FakeRemote()
    return MultiConditionGPSLogger(remote, [
        (is_a, "/tmp/does-not-matter-a.log", "COND_A"),
        (is_b, "/tmp/does-not-matter-b.log", "COND_B"),
    ])


def test_no_condition_triggered_produces_no_writes():
    logger = _logger_with_two_conditions()
    was_triggered = [False, False]
    writes = logger._writes_for_line("$GPRMC,...", left=0, right=0, was_triggered=was_triggered)
    assert writes == []
    assert was_triggered == [False, False]


def test_one_condition_triggered_writes_start_marker_then_data():
    logger = _logger_with_two_conditions()
    was_triggered = [False, False]
    line = "$GPRMC,DATA"
    writes = logger._writes_for_line(line, left=1, right=0, was_triggered=was_triggered)

    # Only condition A (index 0) fired -- exactly one marker + one data line for it.
    indices = [i for i, _ in writes]
    assert indices == [0, 0]
    assert "COND_A_START" in writes[0][1]
    assert line in writes[1][1]
    assert was_triggered == [True, False]


def test_both_conditions_triggered_independently():
    logger = _logger_with_two_conditions()
    was_triggered = [False, False]
    line = "$GPRMC,DATA"
    writes = logger._writes_for_line(line, left=1, right=2, was_triggered=was_triggered)

    indices = [i for i, _ in writes]
    # Both conditions start (2 markers) then both log the data line (2 data writes).
    assert indices.count(0) == 2
    assert indices.count(1) == 2
    assert was_triggered == [True, True]


def test_marker_written_only_on_transition_not_every_line():
    logger = _logger_with_two_conditions()
    was_triggered = [True, False]  # condition A already active from a previous line
    writes = logger._writes_for_line("$GPRMC,DATA", left=1, right=0, was_triggered=was_triggered)

    # No new START marker for A (already active) -- just the data line.
    assert len(writes) == 1
    assert writes[0][0] == 0
    assert "START" not in writes[0][1]
    assert "$GPRMC,DATA" in writes[0][1]


def test_end_marker_written_when_condition_stops():
    logger = _logger_with_two_conditions()
    was_triggered = [True, False]  # condition A was active
    writes = logger._writes_for_line("$GPRMC,DATA", left=0, right=0, was_triggered=was_triggered)

    # Condition A just stopped -- an END marker, no data line (it's no longer triggered).
    assert len(writes) == 1
    assert writes[0][0] == 0
    assert "COND_A_END" in writes[0][1]
    assert was_triggered == [False, False]


def test_no_write_for_empty_line_even_if_triggered():
    logger = _logger_with_two_conditions()
    was_triggered = [True, False]  # already active, so no START marker expected either
    writes = logger._writes_for_line("", left=1, right=0, was_triggered=was_triggered)
    assert writes == []


def test_condition_gps_logger_is_single_condition_multi_logger():
    remote = FakeRemote()
    logger = ConditionGPSLogger(remote, is_a, "/tmp/does-not-matter.log", trigger_name="COND_A")
    assert isinstance(logger, MultiConditionGPSLogger)
    assert logger.conditions == [(is_a, "/tmp/does-not-matter.log", "COND_A")]
    # Backward-compatible attributes still present.
    assert logger.trigger is is_a
    assert logger.log_path == "/tmp/does-not-matter.log"
    assert logger.trigger_name == "COND_A"
