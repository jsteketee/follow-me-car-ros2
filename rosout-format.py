#!/usr/bin/env python3
"""
rosout-format — collapse `ros2 topic echo /rosout` output into one line per log record.

`ros2 topic echo` prints each rcl_interfaces/msg/Log as an 8-line YAML block, which makes the
dev-tmux rosout quadrant unreadable. This filter reads that stream on stdin and prints one
formatted line per record instead:

    12:34:56.789 WARN  [serial_bridge] frame dropped: bad checksum

Severity is colorized (blue/amber/red, matching the web dashboard's rosout overlay) when stdout
is a TTY; NO_COLOR in the environment or --no-color disables it.

The YAML is parsed by hand rather than with PyYAML so this runs under any python3, and records
are emitted as soon as their `---` terminator arrives (run the producer with PYTHONUNBUFFERED=1
so it doesn't block-buffer into the pipe).

Usage:
    PYTHONUNBUFFERED=1 ros2 topic echo /rosout | python3 -u rosout-format.py [--no-color]
"""

import argparse
import os
import re
import sys
from datetime import datetime

# rcl_interfaces/msg/Log severity constants -> (display label, ANSI color).
# INFO is left uncolored on purpose: it is the bulk of the stream, so warnings stand out.
LEVELS = {
    10: ("DEBUG", "\033[90m"),
    20: ("INFO", ""),
    30: ("WARN", "\033[33m"),
    40: ("ERROR", "\033[31m"),
    50: ("FATAL", "\033[1;31m"),
}
UNKNOWN = ("LOG", "")
DIM = "\033[2m"
RESET = "\033[0m"

# Matches a `key: value` line, capturing indent so nested stamp fields can be told apart.
FIELD_RE = re.compile(r"^(\s*)([A-Za-z_]\w*):[ ]?(.*)$")
ESCAPES = {"n": "\n", "t": "\t", "r": "\r", '"': '"', "\\": "\\"}


def is_closed(value):
    """Report whether a YAML scalar's opening quote has been matched by a closing one."""
    quote = value[:1]
    if quote not in ("'", '"'):
        return True
    body = value[1:].rstrip()
    if not body.endswith(quote):
        return False
    if quote == "'":
        # A run of quotes ends the scalar only if odd-length ('' is an escaped quote).
        run = len(body) - len(body.rstrip("'"))
        return run % 2 == 1
    # Double-quoted: the closing quote must not itself be backslash-escaped.
    lead = body[:-1]
    return (len(lead) - len(lead.rstrip("\\"))) % 2 == 0


def unquote(value):
    """Strip YAML quoting from a scalar and resolve its escape sequences."""
    value = value.strip()
    if len(value) >= 2 and value[0] == value[-1] == "'":
        return value[1:-1].replace("''", "'")
    if len(value) >= 2 and value[0] == value[-1] == '"':
        return re.sub(r"\\(.)", lambda m: ESCAPES.get(m.group(1), m.group(0)), value[1:-1])
    return value


def records(stream):
    """Yield one field dict per `---`-terminated YAML block on the echo stream."""
    rec, in_stamp, pending = {}, False, None
    for raw in stream:
        line = raw.rstrip("\r\n")

        if line.strip() == "---":
            # A partial first block (echo attached mid-message) has neither field; drop it.
            if "msg" in rec or "level" in rec:
                yield rec
            rec, in_stamp, pending = {}, False, None
            continue

        # Continuation of a quoted scalar that spilled onto the next line. A blank line is
        # YAML's literal newline inside the scalar — the joining space already stands in for it.
        if pending is not None:
            if line.strip():
                rec[pending] += " " + line.strip()
            if is_closed(rec[pending]):
                rec[pending] = unquote(rec[pending])
                pending = None
            continue

        match = FIELD_RE.match(line)
        if not match:
            continue
        indent, key, value = match.groups()

        if indent:
            if in_stamp and key in ("sec", "nanosec"):
                rec[key] = int(value) if value.lstrip("-").isdigit() else 0
            continue

        in_stamp = key == "stamp" and not value
        if key == "level":
            rec["level"] = int(value) if value.isdigit() else -1
        elif key in ("name", "msg"):
            if value[:1] in ("'", '"') and not is_closed(value):
                rec[key], pending = value, key
            else:
                rec[key] = unquote(value)

    if "msg" in rec or "level" in rec:
        yield rec


def timestamp(rec):
    """Format a record's ROS stamp as local HH:MM:SS.mmm, falling back to arrival time."""
    sec = rec.get("sec")
    when = datetime.fromtimestamp(sec + rec.get("nanosec", 0) / 1e9) if sec else datetime.now()
    return when.strftime("%H:%M:%S") + ".%03d" % (when.microsecond // 1000)


def render(rec, color):
    """Render one parsed record as a single formatted (optionally colorized) line."""
    label, hue = LEVELS.get(rec.get("level"), UNKNOWN)
    stamp = timestamp(rec)
    head = "%-5s [%s]" % (label, rec.get("name", "?"))
    # Newlines inside a log message would break the one-line-per-record promise.
    msg = re.sub(r"\s*[\r\n]+\s*", " ", rec.get("msg", ""))

    if not color:
        return "%s %s %s" % (stamp, head, msg)
    if rec.get("level", 0) >= 30:
        msg = hue + msg + RESET
    return "%s%s%s %s%s%s %s" % (DIM, stamp, RESET, hue or DIM, head, RESET, msg)


def main():
    """Parse args, then stream formatted lines until stdin closes or the user interrupts."""
    parser = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    parser.add_argument("--no-color", action="store_true", help="disable ANSI colorization")
    args = parser.parse_args()

    color = not args.no_color and not os.environ.get("NO_COLOR") and sys.stdout.isatty()
    sys.stdin.reconfigure(errors="replace")

    try:
        for rec in records(sys.stdin):
            print(render(rec, color), flush=True)
    except KeyboardInterrupt:
        return 130
    except BrokenPipeError:
        os._exit(0)
    return 0


if __name__ == "__main__":
    sys.exit(main())
