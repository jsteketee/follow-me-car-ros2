#!/usr/bin/env python3
"""Unit tests for the serial_bridge command-TX path (T2b). No hardware; a FakeSerial
records writes and _tx_tick is called directly for determinism (see NOTES.md).
"""

import json
import math
import queue

import pytest
import rclpy

from nav_msgs.msg import Odometry
from follow_me_interfaces.msg import DriveCommand
from follow_me_nodes.serial_bridge import SerialBridge, MPH_TO_MPS


class FakeSerial:
    """Minimal serial stand-in: records writes, serves fed lines, never blocks forever."""

    def __init__(self):
        self.writes = []                # list[bytes] — every write() payload, in order
        self._lines = queue.Queue()     # telemetry lines to hand back from readline()
        self.closed = False
        self.write_timeout = 0.1
        self.timeout = 1.0

    def readline(self):
        """Return a fed line if one is queued, else b"" after a short wait (no busy-spin)."""
        try:
            return self._lines.get(timeout=0.02)
        except queue.Empty:
            return b""

    def write(self, data):
        """Record one TX payload."""
        self.writes.append(data)
        return len(data)

    def feed(self, line):
        """Queue a telemetry line (bytes) for the reader thread to consume."""
        self._lines.put(line)

    def close(self):
        """Mark closed (the reader calls this on teardown/reconnect)."""
        self.closed = True


# --- message builders -------------------------------------------------------

def make_cmd(speed, heading, stamp_ns=None):
    """Build a DriveCommand; leave the stamp at zero unless stamp_ns is given."""
    c = DriveCommand()
    c.speed = float(speed)
    c.heading = float(heading)
    if stamp_ns is not None:
        c.header.stamp.sec = int(stamp_ns // 1_000_000_000)
        c.header.stamp.nanosec = int(stamp_ns % 1_000_000_000)
    return c


def make_odom(yaw_rad, stamp_ns):
    """Build an Odometry with a yaw-only orientation and the given stamp."""
    o = Odometry()
    o.header.stamp.sec = int(stamp_ns // 1_000_000_000)
    o.header.stamp.nanosec = int(stamp_ns % 1_000_000_000)
    o.pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
    o.pose.pose.orientation.w = math.cos(yaw_rad / 2.0)
    return o


def make_telem(ts, yaw=0.0):
    """Minimal telemetry dict for _publish (all other fields default via .get)."""
    return {"ts": ts, "yaw": yaw}


def establish_offset(node, device_deg, odom_yaw_rad, t_ns=None):
    """Drive the real offset path: pair a device-yaw sample with an odom yaw at the same time."""
    if t_ns is None:
        t_ns = node.get_clock().now().nanoseconds
    with node._tx_lock:
        node._last_device_yaw_deg = device_deg
        node._last_device_yaw_ns = t_ns
    node._on_odom(make_odom(odom_yaw_rad, t_ns))


def last_frame(fake):
    """Parse the most recent TX payload as JSON."""
    assert fake.writes, "expected at least one TX frame"
    return json.loads(fake.writes[-1].decode())


# --- fixture ----------------------------------------------------------------

@pytest.fixture
def node():
    """Construct a SerialBridge whose port is a FakeSerial; tear it down after the test."""
    rclpy.init()
    fake = FakeSerial()
    orig_open = SerialBridge._open_port
    SerialBridge._open_port = lambda self: fake  # every (re)open hands back the same fake
    n = SerialBridge()
    n._ser = fake  # deterministic: don't race the reader thread's own assignment
    try:
        yield n, fake
    finally:
        SerialBridge._open_port = orig_open
        n.destroy_node()
        rclpy.shutdown()


# --- tests ------------------------------------------------------------------

def test_timer_period_and_stream(node):
    """(1) Period is 50 ms, and a short real spin with a fresh command yields many frames."""
    n, fake = node
    assert n._tx_timer.timer_period_ns == 50_000_000

    establish_offset(n, 0.0, 0.0)          # offset 0
    n._on_cmd_drive(make_cmd(0.5, 0.0))    # zero stamp -> arrival time -> fresh

    end_ns = n.get_clock().now().nanoseconds + int(0.4e9)
    while n.get_clock().now().nanoseconds < end_ns:
        rclpy.spin_once(n, timeout_sec=0.02)

    assert len(fake.writes) >= 4           # ~8 expected at 20 Hz over 0.4 s; loose bound


def test_unit_and_sign_roundtrip(node):
    """(2) speed 1.0 m/s -> ~2.24 mph; heading == odom yaw of the pair -> device yaw back."""
    n, fake = node
    device_yaw, odom_yaw = 100.0, math.radians(30.0)
    establish_offset(n, device_yaw, odom_yaw)   # offset = wrap_pm180(100 - 30) = 70

    n._on_cmd_drive(make_cmd(1.0, odom_yaw))    # heading == the odom yaw used above
    n._tx_tick()

    frame = last_frame(fake)
    assert frame["target_speed"] == pytest.approx(1.0 / MPH_TO_MPS, abs=0.01)  # ~2.24
    assert frame["target_heading"] == pytest.approx(device_yaw, abs=0.2)       # round-trip


def test_wrap_seam(node):
    """(3) Heading + offset crossing 360 wraps into [0, 360) with no +/-360 error."""
    n, fake = node
    establish_offset(n, 350.0, math.radians(300.0))   # offset = 50

    n._on_cmd_drive(make_cmd(0.5, math.radians(340.0)))  # 340 + 50 = 390 -> 30
    n._tx_tick()

    heading = last_frame(fake)["target_heading"]
    assert 0.0 <= heading < 360.0
    assert heading == pytest.approx(30.0, abs=0.2)


def test_staleness_gate(node):
    """(4) A latched command older than 500 ms stops TX (silence, not a zero command)."""
    n, fake = node
    establish_offset(n, 0.0, 0.0)
    old_ns = n.get_clock().now().nanoseconds - int(0.6e9)  # 600 ms old
    n._on_cmd_drive(make_cmd(0.5, 0.0, stamp_ns=old_ns))
    n._tx_tick()
    assert fake.writes == []


def test_disconnected_drops_and_never_replays(node):
    """(5) Ticks with no port drop; reconnecting does not flush any queued frame."""
    n, fake = node
    establish_offset(n, 0.0, 0.0)
    n._on_cmd_drive(make_cmd(0.5, 0.0))   # fresh

    n._ser = None
    for _ in range(3):
        n._tx_tick()
    assert fake.writes == []               # disconnected -> every tick drops

    n._ser = fake                          # "reconnect"
    assert fake.writes == []               # nothing was queued, so nothing replays


def test_reboot_halts_until_fresh_command(node):
    """(6) ts going backwards clears the latch and halts TX until a new command is latched."""
    n, fake = node
    establish_offset(n, 0.0, 0.0)
    n._publish(make_telem(ts=1000))        # first frame captures the clock offset
    n._on_cmd_drive(make_cmd(0.5, 0.0))    # fresh latch

    n._publish(make_telem(ts=500))         # ts jumped backwards -> reboot
    assert n._latched is None
    assert n._halt is True

    n._tx_tick()
    assert fake.writes == []               # halted -> no TX

    establish_offset(n, 0.0, 0.0)          # reboot invalidated the offset; re-establish
    n._on_cmd_drive(make_cmd(0.5, 0.0))    # a fresh command re-arms
    assert n._halt is False
    n._tx_tick()
    assert len(fake.writes) == 1           # TX resumes only after the new command


def test_speed_passthrough_and_nan_reject(node):
    """(7) Speed is sent un-clamped (the ESP32 gates); a NaN heading is not latched."""
    n, fake = node
    establish_offset(n, 0.0, 0.0)

    n._on_cmd_drive(make_cmd(5.0, 0.0))    # 5 m/s -> ~11.18 mph, sent raw (no Pi cap)
    n._tx_tick()
    assert last_frame(fake)["target_speed"] == pytest.approx(5.0 / MPH_TO_MPS, abs=0.01)

    n._on_cmd_drive(make_cmd(-1.0, 0.0))   # negative sent raw (ESP32 rejects it, not the Pi)
    n._tx_tick()
    assert last_frame(fake)["target_speed"] == pytest.approx(-1.0 / MPH_TO_MPS, abs=0.01)

    n._on_cmd_drive(make_cmd(0.5, float("nan")))  # non-finite heading -> not latched (guard kept)
    assert n._latched[0] == -1.0           # prior latch (speed -1.0) intact, NaN not stored
