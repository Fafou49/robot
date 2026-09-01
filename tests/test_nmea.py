"""
Unit tests for the NMEA-style sentence helpers (link/nmea.py). No hardware
dependency, run with:

    pytest
"""
import pytest

from link.nmea import SentenceError, build_sentence, compute_checksum, parse_sentence


def test_checksum_matches_hand_computed_example():
    # From pages/protocole_controle.html: $PROV,STP*60
    assert compute_checksum("PROV,STP") == "60"


def test_build_sentence_round_trips_through_parse():
    sentence = build_sentence("DRV", 120, 120)
    sentence_type, fields = parse_sentence(sentence)
    assert sentence_type == "DRV"
    assert fields == ["120", "120"]


def test_build_sentence_matches_documented_example():
    assert build_sentence("DRV", 120, 120) == "$PROV,DRV,120,120*77"


def test_parse_rejects_bad_checksum():
    with pytest.raises(SentenceError):
        parse_sentence("$PROV,STP*00")


def test_parse_rejects_missing_dollar():
    with pytest.raises(SentenceError):
        parse_sentence("PROV,STP*60")


def test_parse_rejects_missing_star():
    with pytest.raises(SentenceError):
        parse_sentence("$PROV,STP")


def test_parse_rejects_wrong_prefix():
    # Correct checksum for "OTHER,STP", but not our proprietary prefix.
    body = "OTHER,STP"
    sentence = f"${body}*{compute_checksum(body)}"
    with pytest.raises(SentenceError):
        parse_sentence(sentence)


def test_parse_is_case_insensitive_on_checksum_hex():
    sentence = build_sentence("STP")
    lowercase_checksum = sentence.replace(sentence[-2:], sentence[-2:].lower())
    sentence_type, fields = parse_sentence(lowercase_checksum)
    assert sentence_type == "STP"
    assert fields == []
