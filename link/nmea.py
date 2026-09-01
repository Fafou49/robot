"""Build and parse the project's NMEA-0183-inspired proprietary sentences.

Sentence shape: $PROV,<TYPE>,<field1>,<field2>,...*<checksum>\\r\\n
- "PROV" is the fixed proprietary-sentence prefix (P = NMEA proprietary
  sentence, ROV = this project's 3-letter code).
- <checksum> is the XOR of every byte between "$" and "*", as 2 uppercase
  hex characters -- identical to the standard NMEA 0183 checksum.

This module is deliberately dependency-free (stdlib only) since it's used
on both sides of the link: the robot (Pi #1, this repo) and the web server
(Pi #2, the separate robot-webserver project, which has its own copy of
this file).
"""

PREFIX = "PROV"


class SentenceError(ValueError):
    """Raised when a raw line isn't a well-formed / valid sentence."""


def compute_checksum(body: str) -> str:
    """XOR of every character in `body` (the part between "$" and "*",
    exclusive), formatted as 2 uppercase hex digits."""
    checksum = 0
    for ch in body:
        checksum ^= ord(ch)
    return f"{checksum:02X}"


def build_sentence(sentence_type: str, *fields) -> str:
    """Builds a full sentence, e.g. build_sentence("DRV", 120, 120) ->
    "$PROV,DRV,120,120*77" (no trailing \\r\\n -- callers append it when
    writing to a socket, since tests often compare the bare string)."""
    parts = [PREFIX, sentence_type, *[str(f) for f in fields]]
    body = ",".join(parts)
    return f"${body}*{compute_checksum(body)}"


def parse_sentence(raw: str):
    """Parses a raw line into (sentence_type, fields). Raises
    SentenceError with a human-readable reason if the line is malformed,
    doesn't use our proprietary prefix, or fails the checksum."""
    raw = raw.strip()
    if not raw.startswith("$"):
        raise SentenceError("missing leading '$'")
    if "*" not in raw:
        raise SentenceError("missing '*checksum'")

    body, _, checksum = raw[1:].partition("*")
    checksum = checksum.strip()
    if len(checksum) != 2:
        raise SentenceError("checksum must be exactly 2 hex characters")

    expected = compute_checksum(body)
    if checksum.upper() != expected:
        raise SentenceError(f"checksum mismatch (got {checksum.upper()}, expected {expected})")

    parts = body.split(",")
    if len(parts) < 2 or parts[0] != PREFIX:
        raise SentenceError(f"expected prefix '{PREFIX}', got '{parts[0]}'")

    sentence_type = parts[1]
    fields = parts[2:]
    return sentence_type, fields
