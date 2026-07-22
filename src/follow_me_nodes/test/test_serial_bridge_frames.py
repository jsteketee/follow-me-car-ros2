#!/usr/bin/env python3
"""Unit tests for serial_bridge inbound frame dispatch: flat telemetry vs typed frames.
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
        self.closed = False
        self.write_timeout = 0.1
        self.timeout = 1.0

    def readline(self):
        """Return a fed line if queued, else b"" after a short wait."""
        try:
            return self._lines.get(timeout=0.02)
        except queue.Empty:
            return b""

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
    """Recording logger: (level, text) per call, mimicking the rclpy logger surface."""

    def __init__(self):
        self.records = []

    def debug(self, m): self.records.append(("debug", m))
    def info(self, m): self.records.append(("info", m))
    def warning(self, m): self.records.append(("warning", m))
    def warn(self, m): self.records.append(("warning", m))
    def error(self, m): self.records.append(("error", m))
    def fatal(self, m): self.records.append(("fatal", m))


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


def test_untyped_frame_publishes_all_telemetry(node):
    """A frame without "type" takes the flat telemetry path and hits every publisher."""
    node._publish({"ts": 1000, "yaw": 12.0, "mode": "SETPOINT", "enc_fault": 1})
    for name in TELEMETRY_PUBS:
        assert len(getattr(node, name).msgs) == 1, f"{name} did not publish"
    assert node.pub_command_status.msgs[0].command_mode == "SETPOINT"
    assert node.pub_wheel.msgs[0].enc_fault is True
    node._publish({"ts": 1020, "yaw": 12.0})   # enc_fault absent (older firmware) -> healthy
    assert node.pub_wheel.msgs[1].enc_fault is False


def test_log_frame_publishes_no_telemetry(node):
    """A log frame is re-logged only: no telemetry topics, no clock/halt side effects."""
    node._publish({"ts": 1000, "yaw": 0.0})   # establish a clock offset first
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
