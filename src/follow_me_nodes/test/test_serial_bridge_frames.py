#!/usr/bin/env python3
"""Unit tests for serial_bridge inbound frame dispatch: grouped telemetry vs typed frames.
No hardware; frames are passed straight to _publish (see test_serial_bridge_tx.py).
"""

import queue

import pytest
import rclpy

from follow_me_nodes.serial_bridge import SerialBridge


class FakeSerial:
    """Minimal serial stand-in so the reader thread never touches hardware."""

    def __init__(self):
        self.writes = []
        self._lines = queue.Queue()
        self._buf = bytearray()
        self.closed = False
        self.write_timeout = 0.1
        self.timeout = 1.0

    @property
    def in_waiting(self):
        """Bytes ready to read: drain any queued lines into the buffer, report its length."""
        while True:
            try:
                self._buf.extend(self._lines.get_nowait())
            except queue.Empty:
                break
        return len(self._buf)

    def read(self, n):
        """Return up to n buffered bytes, else b"" after a short wait."""
        if not self._buf:
            try:
                self._buf.extend(self._lines.get(timeout=0.02))
            except queue.Empty:
                return b""
        take = bytes(self._buf[:n])
        del self._buf[:n]
        return take

    def write(self, data):
        """Record one TX payload."""
        self.writes.append(data)
        return len(data)

    def close(self):
        """Mark closed."""
        self.closed = True


class FakePublisher:
    """Recording stand-in for a telemetry publisher."""

    def __init__(self):
        self.msgs = []

    def publish(self, msg):
        """Record one published message."""
        self.msgs.append(msg)


class FakeLogger:
    """Recording logger: (level, text) per call; ignores rclpy logging kwargs (throttle_*)."""

    def __init__(self):
        self.records = []

    def debug(self, m, **kw): self.records.append(("debug", m))
    def info(self, m, **kw): self.records.append(("info", m))
    def warning(self, m, **kw): self.records.append(("warning", m))
    def warn(self, m, **kw): self.records.append(("warning", m))
    def error(self, m, **kw): self.records.append(("error", m))
    def fatal(self, m, **kw): self.records.append(("fatal", m))


TELEMETRY_PUBS = ("pub_imu", "pub_wheel", "pub_command_status",
                  "pub_actuator_status", "pub_uwb_raw", "pub_joints")
EVENT_PUBS = ("pub_sensor_health",)


@pytest.fixture
def node():
    """SerialBridge with a FakeSerial port and all telemetry publishers recorded."""
    rclpy.init()
    fake = FakeSerial()
    orig_open = SerialBridge._open_port
    SerialBridge._open_port = lambda self: fake
    n = SerialBridge()
    n._ser = fake
    for name in TELEMETRY_PUBS + EVENT_PUBS:
        setattr(n, name, FakePublisher())
    try:
        yield n
    finally:
        SerialBridge._open_port = orig_open
        n.destroy_node()
        rclpy.shutdown()


def telemetry_counts(n):
    """Total messages recorded across all telemetry publishers."""
    return sum(len(getattr(n, name).msgs) for name in TELEMETRY_PUBS)


def test_grouped_frame_publishes_all_telemetry(node):
    """An untyped (telemetry) frame fans its groups out to every publisher; DIRECT echo included."""
    node._publish({
        "ts": 1000, "seq": 1,
        "imu": {"t": 998, "yaw": 12.0},
        "uwb": {"t": 995, "dist": 200.0, "bearing": 3.0},
        "cmd": {"t": 990, "speed": 2.0, "heading": 270.0, "throttle": 0.3, "steering": -0.1, "rejects": 0},
        "wheel": {"speed": 1.9, "odo": 100.0, "enc_fault": 1},
        "ctrl": {"throttle": 0.3, "steering": -0.1, "pan_angle": -5.0},
    })
    for name in TELEMETRY_PUBS:
        assert len(getattr(node, name).msgs) == 1, f"{name} did not publish"
    cmd = node.pub_command_status.msgs[0]
    assert cmd.cmd_throttle == pytest.approx(0.3)   # DIRECT echo carried through
    assert cmd.cmd_age_ms == 10                      # ts - cmd.t = 1000 - 990
    assert node.pub_wheel.msgs[0].enc_fault is True
    node._publish({"ts": 1020, "wheel": {"speed": 1.9, "odo": 100.0}})  # enc_fault absent -> healthy
    assert node.pub_wheel.msgs[1].enc_fault is False


def test_log_frame_publishes_no_telemetry(node):
    """A log frame is re-logged only: no telemetry topics, no clock/halt side effects."""
    node._publish({                            # establish a clock offset + the six telemetry msgs
        "ts": 1000, "seq": 1,
        "imu": {"t": 998, "yaw": 0.0},
        "uwb": {"t": 995, "dist": 200.0, "bearing": 0.0},
        "cmd": {"t": 990, "speed": 0.0, "heading": 0.0, "rejects": 0},
        "wheel": {"speed": 0.0, "odo": 0.0},
        "ctrl": {"throttle": 0.0, "steering": 0.0, "pan_angle": 0.0},
    })
    offset = node._clock_offset_ns
    node.get_logger = lambda: FakeLogger()    # silence; side effects are the assertion
    node._publish({"type": "log", "level": "error", "msg": "ESC overtemp"})
    assert telemetry_counts(node) == 6        # only the telemetry frame's six
    assert node._clock_offset_ns == offset    # ts-less frame didn't fake a reboot
    assert node._halt is False


def test_log_frame_level_mapping(node):
    """Wire levels map onto logger severities; unknown levels fall back to info."""
    logger = FakeLogger()
    node.get_logger = lambda: logger
    for wire, expected in [("debug", "debug"), ("info", "info"), ("warn", "warning"),
                           ("warning", "warning"), ("error", "error"), ("fatal", "fatal"),
                           ("LOUD", "info")]:
        node._publish({"type": "log", "level": wire, "msg": "x"})
        assert logger.records[-1] == (expected, "[esp32] x")
    node._publish({"type": "log", "msg": "no level"})   # level omitted -> info
    assert logger.records[-1] == ("info", "[esp32] no level")


def test_health_frame_publishes_sensor_health(node):
    """A health frame maps its sensors object to parallel arrays and no telemetry topics."""
    node._publish({"type": "health", "sensors": {"imu": 205.0, "uwb": 0, "loop": 2000}})
    assert telemetry_counts(node) == 0
    assert len(node.pub_sensor_health.msgs) == 1
    msg = node.pub_sensor_health.msgs[0]
    assert dict(zip(msg.names, msg.rates_hz)) == {"imu": 205.0, "uwb": 0.0, "loop": 2000.0}


def test_health_frame_skips_bad_values(node):
    """A non-numeric rate drops that pair only; a missing sensors object drops the frame."""
    logger = FakeLogger()
    node.get_logger = lambda: logger
    node._publish({"type": "health", "sensors": {"imu": 205.0, "uwb": "dead"}})
    msg = node.pub_sensor_health.msgs[0]
    assert dict(zip(msg.names, msg.rates_hz)) == {"imu": 205.0}
    node._publish({"type": "health"})
    assert len(node.pub_sensor_health.msgs) == 1
    assert any("sensors" in r[1] for r in logger.records if r[0] == "warning")


def test_unknown_type_warns_once_and_drops(node):
    """An unknown "type" publishes nothing and warns exactly once per type value."""
    logger = FakeLogger()
    node.get_logger = lambda: logger
    for _ in range(3):
        node._publish({"type": "caps", "max_speed": 2.5})
    assert telemetry_counts(node) == 0
    warns = [r for r in logger.records if r[0] == "warning" and "caps" in r[1]]
    assert len(warns) == 1


def test_telemetry_watchdog_warns_only_past_the_limit(node):
    """Quiet inside the 1 s limit, warns past it, from either baseline: node start (nothing ever
    received) or the last landed frame."""
    logger = FakeLogger()
    node.get_logger = lambda: logger

    # Fresh node: nothing is overdue yet.
    node._telem_watchdog_tick()
    assert logger.records == []

    # 1.5 s up with no frame ever received.
    node._node_start_ns -= 1_500_000_000
    node._telem_watchdog_tick()
    assert len(logger.records) == 1
    level, text = logger.records[0]
    assert level == "warning"
    assert "none received since startup" in text

    # A landed frame re-baselines the watchdog onto _last_frame_ns and clears the silence.
    node._publish({"ts": 1000, "seq": 1, "wheel": {"speed": 0.0, "odo": 0.0}})
    node._telem_watchdog_tick()
    assert len(logger.records) == 1

    # Link goes quiet after a good frame.
    node._last_frame_ns -= 1_500_000_000
    node._telem_watchdog_tick()
    assert len(logger.records) == 2
    assert "1.5 s" in logger.records[1][1]
    assert "none received since startup" not in logger.records[1][1]
