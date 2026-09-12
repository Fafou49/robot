"""Tests for motor_control.gps_condition_logger.

Only MultiConditionGPSLogger._writes_for_line() is tested here: it's the
one piece of pure logic in this module (no serial port, no real files,
no hardware) -- the per-line dispatch that decides which condition(s) a
line should be logged under and when a START/END marker is due, and
(2026-09-11) when the on_transition callback fires. The rest (the actual
serial read loop, ConditionGPSLogger's backward-compatible constructor)
needs pyserial and a real Remote/gamepad, so it's not exercised here --
same honesty note as the rest of this project's hardware-facing code.
on_transition itself is just a plain function call from within
_writes_for_line, no I/O -- what a *real* caller does with it (e.g.
buzzing a gamepad, see link/gamepad_handler.py's start_rumble()) is that
caller's own concern and is tested over there instead.

is_dgps_quality() (2026-09-11) is tested further down -- plain
arithmetic, no dependencies. _gga_quality() itself (the actual pynmea2-
based sentence parsing) has its own test file, tests/test_gga_quality.py,
skipped via pytest.importorskip("pynmea2") if it isn't installed --
deliberately NOT in this file: pytest.importorskip() at module level
skips the ENTIRE module's collection on failure, not just the tests after
it, so keeping it out of this file means the plain tests here (including
is_dgps_quality()'s) still run even where pynmea2 isn't installed (e.g.
this project's dev sandbox), instead of silently going along for the ride
with a skip they don't actually need. on_gps_quality's "only while
something is being logged" gating lives in _loop() itself, which -- like
the rest of _loop() -- needs a real serial port and isn't exercised here.
"""
from motor_control.gps_condition_logger import (
    ConditionGPSLogger, DGPS_QUALITY, MultiConditionGPSLogger, _is_gps_sentence,
    is_dgps_quality,
)


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


def test_no_data_write_for_a_non_gps_line_even_if_triggered():
    # The bug this guards against (2026-09-11 field report): gps/
    # dgps_transfer.py writes RTCM/DGPS correction bytes to the very same
    # serial device these loggers read from -- before _is_gps_sentence()
    # existed, ANY non-empty line read while triggered was logged
    # verbatim, correction traffic included. A START marker is still
    # expected (the transition itself doesn't depend on what `line` is),
    # just no data line for content that isn't a recognized GPS sentence.
    logger = _logger_with_two_conditions()
    was_triggered = [False, False]
    writes = logger._writes_for_line(
        "not a gps sentence at all", left=1, right=0, was_triggered=was_triggered,
    )
    assert len(writes) == 1
    assert "COND_A_START" in writes[0][1]


def test_no_data_write_for_already_triggered_non_gps_line():
    logger = _logger_with_two_conditions()
    was_triggered = [True, False]  # already active -- no START marker expected either
    writes = logger._writes_for_line(
        "not a gps sentence at all", left=1, right=0, was_triggered=was_triggered,
    )
    assert writes == []


def test_on_transition_called_on_start_and_end_not_in_between():
    calls = []
    remote = FakeRemote()
    logger = MultiConditionGPSLogger(
        remote, [(is_a, "/tmp/does-not-matter-a.log", "COND_A")],
        on_transition=lambda trigger_name, triggered: calls.append((trigger_name, triggered)),
    )
    was_triggered = [False]

    logger._writes_for_line("$GPRMC,DATA", left=1, right=0, was_triggered=was_triggered)
    logger._writes_for_line("$GPRMC,DATA", left=1, right=0, was_triggered=was_triggered)  # still triggered
    logger._writes_for_line("$GPRMC,DATA", left=0, right=0, was_triggered=was_triggered)

    assert calls == [("COND_A", True), ("COND_A", False)]


def test_on_transition_defaults_to_a_silent_no_op():
    # No on_transition passed -- must not raise.
    remote = FakeRemote()
    logger = MultiConditionGPSLogger(remote, [(is_a, "/tmp/does-not-matter-a.log", "COND_A")])
    was_triggered = [False]
    logger._writes_for_line("$GPRMC,DATA", left=1, right=0, was_triggered=was_triggered)


def test_on_transition_fires_independently_per_condition():
    calls = []
    remote = FakeRemote()
    logger = MultiConditionGPSLogger(remote, [
        (is_a, "/tmp/does-not-matter-a.log", "COND_A"),
        (is_b, "/tmp/does-not-matter-b.log", "COND_B"),
    ], on_transition=lambda trigger_name, triggered: calls.append((trigger_name, triggered)))
    was_triggered = [False, False]

    logger._writes_for_line("$GPRMC,DATA", left=1, right=2, was_triggered=was_triggered)
    assert calls == [("COND_A", True), ("COND_B", True)]


def test_condition_gps_logger_forwards_on_transition():
    calls = []
    remote = FakeRemote()
    logger = ConditionGPSLogger(
        remote, is_a, "/tmp/does-not-matter.log", trigger_name="COND_A",
        on_transition=lambda trigger_name, triggered: calls.append((trigger_name, triggered)),
    )
    was_triggered = [False]
    logger._writes_for_line("$GPRMC,DATA", left=1, right=0, was_triggered=was_triggered)
    assert calls == [("COND_A", True)]


def test_condition_gps_logger_is_single_condition_multi_logger():
    remote = FakeRemote()
    logger = ConditionGPSLogger(remote, is_a, "/tmp/does-not-matter.log", trigger_name="COND_A")
    assert isinstance(logger, MultiConditionGPSLogger)
    assert logger.conditions == [(is_a, "/tmp/does-not-matter.log", "COND_A")]
    # Backward-compatible attributes still present.
    assert logger.trigger is is_a
    assert logger.log_path == "/tmp/does-not-matter.log"
    assert logger.trigger_name == "COND_A"


# --- last_is_dgps / is_dgps_quality (no hardware, no pynmea2 needed) --------

def test_last_is_dgps_defaults_to_false_before_any_gga_line():
    remote = FakeRemote()
    logger = MultiConditionGPSLogger(remote, [(is_a, "/tmp/does-not-matter-a.log", "COND_A")])
    assert logger.last_is_dgps is False


def test_is_dgps_quality_only_true_for_the_dgps_indicator():
    assert is_dgps_quality(DGPS_QUALITY) is True
    assert is_dgps_quality(1) is False   # plain autonomous GPS fix
    assert is_dgps_quality(0) is False   # no fix
    assert is_dgps_quality(None) is False


# --- _is_gps_sentence (2026-09-11, no pynmea2 needed -- pure prefix check) --

def test_is_gps_sentence_accepts_known_talker_and_type_combinations():
    assert _is_gps_sentence("$GPGGA,123519,...") is True
    assert _is_gps_sentence("$GNRMC,...") is True
    assert _is_gps_sentence("$GPRMC,DATA") is True  # same fixture style as the tests above
    assert _is_gps_sentence("$GNGSA,...") is True
    assert _is_gps_sentence("$GPVTG,...") is True


def test_is_gps_sentence_rejects_empty_and_missing_dollar():
    assert _is_gps_sentence("") is False
    assert _is_gps_sentence("GPGGA,123519,...") is False  # missing leading "$"


def test_is_gps_sentence_rejects_correction_looking_noise():
    # What this filter exists for: RTCM/DGPS correction bytes (or any
    # other non-NMEA noise) read off the same serial line as the GPS's
    # own output, decoded as ASCII -- doesn't start with a recognized
    # talker+type combination, so it's not mistaken for a GPS frame.
    assert _is_gps_sentence("not a gps sentence at all") is False
    assert _is_gps_sentence("�������") is False
    assert _is_gps_sentence("$PUBX,04,...") is False  # unrecognized talker/type


def test_is_gps_sentence_rejects_too_short_lines():
    assert _is_gps_sentence("$GP") is False
