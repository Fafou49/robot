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


def decimal_to_nmea(value: float, is_longitude: bool):
    """Converts a signed decimal-degrees coordinate into this protocol's
    on-the-wire (ddmm.mmmm string, direction letter) pair -- the format
    already used by NAV's lat/lon fields, e.g. 47.391534 -> ("4723.492",
    "N"), -0.739006 -> ("00044.340", "W"). Longitude gets 3-digit degrees
    (000-179), latitude 2 (00-90), matching standard NMEA GGA/RMC fields."""
    direction = ("W" if value < 0 else "E") if is_longitude else ("S" if value < 0 else "N")
    magnitude = abs(value)
    degrees = int(magnitude)
    minutes = (magnitude - degrees) * 60
    deg_digits = 3 if is_longitude else 2
    return f"{degrees:0{deg_digits}d}{minutes:06.3f}", direction


def nmea_to_decimal(raw: str, direction: str):
    """The inverse of decimal_to_nmea: parses a ddmm.mmmm string plus its
    direction letter back into signed decimal degrees. Returns None if
    `raw` isn't a usable number."""
    try:
        value = float(raw)
    except (TypeError, ValueError):
        return None
    degrees = int(value // 100)
    minutes = value - degrees * 100
    decimal = degrees + minutes / 60
    if direction in ("S", "W"):
        decimal = -decimal
    return decimal
