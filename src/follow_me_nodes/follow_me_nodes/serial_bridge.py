#!/usr/bin/env python3
"""Serial bridge: ESP32-S3 grouped JSON telemetry <-> ROS2 topics (SI/REP-103 conversion boundary).
Reads ~50 Hz producer-grouped frames; async groups (imu/uwb/cmd) stamp at their capture time t,
loop-sync groups (wheel/ctrl) at the frame ts. Writes drive commands back at 20 Hz. Schema: interface.md.
"""

import json
import math
import threading
from collections import deque

import rclpy
from rclpy.node import Node

import serial

from sensor_msgs.msg import Imu, JointState
from nav_msgs.msg import Odometry
from follow_me_interfaces.msg import (
    ActuatorStatus,
    CommandStatus,
    DriveCommand,
    SensorHealth,
    UwbRaw,
    WheelState,
)

DEFAULT_PORT = (
    "/dev/serial/by-id/"
    "usb-Espressif_USB_JTAG_serial_debug_unit_3C:DC:75:71:53:58-if00"
)

# ---------------------------------------------------------------------------
# Project-defined topics — RELATIVE names, namespaced per robot at launch.
#
# No leading "/". A relative name is resolved against the node's namespace, so
# `--ros-args -r __ns:=/fmbot` yields /fmbot/imu/data. An absolute name (leading
# "/") would ignore the namespace entirely and break multi-robot.
# ---------------------------------------------------------------------------
TOPIC_IMU = "imu/data"
# Co-sampled wheel readings bundle into one stamped message (see TOPIC LAYOUT in the docstring).
TOPIC_WHEEL_STATE = "wheel/state"
# ESP32 status/echo and the UWB tag fix stream.
TOPIC_COMMAND_STATUS = "command/status"
TOPIC_ACTUATOR_STATUS = "actuator/status"
TOPIC_UWB_RAW = "uwb/raw"
# Low-rate per-sensor update rates from {"type":"health"} event frames (~1-2 Hz).
TOPIC_SENSOR_HEALTH = "sensor_health"

# Max chars of an offending serial line to echo into a warning (bounds log spam).
LOG_SNIPPET_CHARS = 120

# Forward telemetry seq jump beyond this is treated as a glitch/reboot, not real loss (not counted).
SEQ_GAP_MAX = 10000

# Cap on the serial reassembly buffer; a partial line larger than this (no newline) is dropped.
SERIAL_BUF_MAX = 8192

# ---------------------------------------------------------------------------
# Telemetry-silence watchdog (Pi side) — why the period is sub-limit: see NOTES.md.
# ---------------------------------------------------------------------------
TELEM_SILENCE_LIMIT_NS = 1_000_000_000
TELEM_WATCHDOG_PERIOD_S = 0.25

# Subscribed (relative): drive setpoints in, odom for the outbound heading offset.
TOPIC_CMD_DRIVE = "cmd_drive"
TOPIC_ODOM = "odom"

# Joint states for robot_state_publisher — drives the base_link -> uwb_link pan edge so
# the anchor frame reflects the live servo angle, plus the two front steer joints so the
# render shows the wheels turning. Joint names must match the revolute joints in
# follow_me_car.urdf.
TOPIC_JOINT_STATES = "joint_states"
PAN_JOINT_NAME = "base_to_uwb"
STEER_JOINT_NAMES = ["base_to_front_left_wheel", "base_to_front_right_wheel"]
# Steering visual scaling — BOTH values are uncalibrated guesses (see NOTES.md: sign
# convention + lock angle deferred with the raw-actuator mode). Purely cosmetic: nothing
# downstream consumes these joints. MAX_STEER_RAD is an assumed +/-30 deg lock; STEER_SIGN
# assumes the wire's steering follows pan_angle's +right convention, so it is negated into
# TF's +z = +left (CCW). Flip STEER_SIGN to +1.0 if the render mirrors reality.
MAX_STEER_RAD = 0.5236  # 30 deg
STEER_SIGN = -1.0

# ---------------------------------------------------------------------------
# Unit conversion — REP-103 compliance.
#
# The wire is SI except angles (2026-07-25; the ESP32 previously spoke mph / cm):
# speed, distance and accel pass through, only degrees convert to radians. This
# node is that boundary, so everything downstream is SI by construction. Sign
# flips live here too — uwb bearing is +right on the wire, +left in REP-103.
# ---------------------------------------------------------------------------
DEG_TO_RAD = math.pi / 180.0  # angles only; speed/distance are SI on the wire (2026-07-25)

# ---------------------------------------------------------------------------
# Command TX (Pi -> ESP32) tuning.
# ---------------------------------------------------------------------------
CMD_TX_PERIOD_S = 0.05          # 20 Hz command stream (the ESP32's heartbeat)
CMD_STALE_NS = 500_000_000      # latched command older than this -> stop sending (500 ms)
OFFSET_SYNC_NS = 200_000_000    # device-yaw / odom-yaw must be this close in time to pair
OFFSET_EMA_ALPHA = 0.2          # EMA weight on the WRAPPED heading-offset delta


def euler_deg_to_quaternion(roll_deg, pitch_deg, yaw_deg):
    """Convert roll/pitch/yaw (degrees, ZYX intrinsic) to a quaternion (x, y, z, w)."""
    r = math.radians(roll_deg) * 0.5
    p = math.radians(pitch_deg) * 0.5
    y = math.radians(yaw_deg) * 0.5
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw


def yaw_from_quaternion(x, y, z, w):
    """Extract yaw (rotation about z, radians) from a quaternion."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_rad(a):
    """Wrap radians to (-pi, pi] — seam-safe form for angle differences."""
    return math.atan2(math.sin(a), math.cos(a))


def wrap_pm180(deg):
    """Wrap degrees to [-180, 180) — the seam-safe form for heading-offset deltas."""
    return (deg + 180.0) % 360.0 - 180.0


def wrap_0_360(deg):
    """Wrap degrees to [0, 360) — the ESP32 compass-heading convention on the wire."""
    return deg % 360.0


class SerialBridge(Node):
    def __init__(self):
        """Set up params, publishers/subscribers, TX timer, and the serial reader thread."""
        super().__init__("serial_bridge")

        self.port = self.declare_parameter("serial_port", DEFAULT_PORT).value
        self.baud = self.declare_parameter("baud", 115200).value
        # TF frame ids. Namespacing does NOT prefix these — frame ids live in the
        # global TF tree, so multi-robot needs them set explicitly per robot
        # (e.g. fmbot/base_link) via these parameters at launch.
        self.frame_id = self.declare_parameter("frame_id", "base_link").value
        self.imu_frame_id = self.declare_parameter("imu_frame_id", "imu_link").value
        self.pub_imu = self.create_publisher(Imu, TOPIC_IMU, 10)
        self.pub_wheel = self.create_publisher(WheelState, TOPIC_WHEEL_STATE, 10)
        self.pub_command_status = self.create_publisher(
            CommandStatus, TOPIC_COMMAND_STATUS, 10
        )
        self.pub_actuator_status = self.create_publisher(
            ActuatorStatus, TOPIC_ACTUATOR_STATUS, 10
        )
        self.pub_uwb_raw = self.create_publisher(UwbRaw, TOPIC_UWB_RAW, 10)
        self.pub_joints = self.create_publisher(JointState, TOPIC_JOINT_STATES, 10)
        self.pub_sensor_health = self.create_publisher(
            SensorHealth, TOPIC_SENSOR_HEALTH, 10
        )

        # Device-clock -> ROS-clock offset, captured on the first frame.
        self._clock_offset_ns = None
        self._last_ts_ms = None

        # Odometer stitching: the onboard odo resets to ~0 on ESP32 reboot; an accumulation
        # offset keeps wheel/state.distance continuous across reboots (see interface.md).
        # _reboot_pending is raised by the reboot detector, consumed on the next frame.
        self._odo_offset_m = 0.0
        self._last_cont_dist_m = None
        self._reboot_pending = False

        # Pi-side received-telemetry rate: ns stamps of successful telemetry frames, pruned
        # to a 1 s sliding window. Reader-thread-local -> no lock; reported on sensor_health.
        self._telem_stamps = deque()

        # Link-health counters, all reader-thread-local (mutated in the reader thread, emitted
        # from _handle_health_frame on that same thread) -> no lock. Cumulative since boot except
        # _max_gap_ns_window, which is windowed and reset on each health emit.
        self._last_seq = None          # last telemetry "seq" seen; None until the first frame
        self._seq_gaps = 0             # missing sequence numbers (dropped/lost telemetry frames)
        self._parse_fail_count = 0     # unparseable / field-coercion-dropped telemetry lines
        self._last_frame_ns = None     # arrival ns of the previous telemetry frame
        self._max_gap_ns_window = 0    # largest inter-frame gap since the last health emit

        # Async-group monotonic gate baselines: imu/data and uwb/raw publish only when their group's
        # capture time advances, so a frozen sensor stops republishing (its header stamp goes stale
        # rather than masquerading as fresh). Reset on reboot (t restarts near 0). Reader-thread-local.
        self._last_imu_t = None
        self._last_uwb_t = None

        # Frame "type" values seen but not understood — each warns once, then drops.
        self._unknown_frame_types = set()

        # The port handle is shared between the reader thread (owns open/close) and the
        # TX timer on the executor thread (writes command frames). The lock guards the
        # handle swap; it is None whenever disconnected, so a TX tick in a gap just drops.
        self._ser = None
        self._ser_lock = threading.Lock()

        # --- Command TX (Pi -> ESP32) ---
        # cmd_drive is latched (newest wins) by its callback; the timer does ALL writing.
        # odom feeds the heading offset that maps odom-frame headings to device compass deg.
        self.sub_cmd = self.create_subscription(
            DriveCommand, TOPIC_CMD_DRIVE, self._on_cmd_drive, 10
        )
        self.sub_odom = self.create_subscription(
            Odometry, TOPIC_ODOM, self._on_odom, 10
        )
        # Shared TX state, written from the executor (latch) AND the reader thread (reboot
        # halt) -> its own lock. Kept separate from _ser_lock: the timer snapshots this
        # state, releases, THEN takes _ser_lock to write, so the two locks never nest.
        self._tx_lock = threading.Lock()
        self._latched = None             # (speed_mps, heading_rad, stamp_ns) or None
        self._heading_offset_deg = None  # EMA of (device_yaw_deg - degrees(odom_yaw))
        self._halt = False               # set on ESP32 reboot; cleared by a fresh cmd_drive
        self._last_device_yaw_deg = None
        self._last_device_yaw_ns = None

        self._tx_timer = self.create_timer(CMD_TX_PERIOD_S, self._tx_tick)

        self._node_start_ns = self.get_clock().now().nanoseconds
        self._telem_watchdog_timer = self.create_timer(
            TELEM_WATCHDOG_PERIOD_S, self._telem_watchdog_tick
        )

        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def _open_port(self):
        """Open the serial port with read/write timeouts (test seam: monkeypatched)."""
        return serial.serial_for_url(
            self.port, baudrate=self.baud, timeout=0.05, write_timeout=0.1
        )

    def destroy_node(self):
        """Stop the reader thread and tear down the node."""
        self._stop.set()
        if self._thread.is_alive():
            self._thread.join(timeout=2.0)
        super().destroy_node()

    def _device_ts_to_ros_ns(self, ts_ms):
        """Map ESP32 uptime (ms) to ROS-clock ns via a fixed offset captured on the first frame."""
        ts_ns = int(ts_ms) * 1_000_000

        # First frame, or ESP32 rebooted (uptime jumped backwards) -> (re)capture offset.
        if self._clock_offset_ns is None or (
            self._last_ts_ms is not None and ts_ms < self._last_ts_ms
        ):
            if self._clock_offset_ns is not None:
                self.get_logger().error(
                    f"ESP32 REBOOTED (device clock {self._last_ts_ms} -> {ts_ms} ms); "
                    "stitching odometer continuous, holding odom pose, halting TX until a "
                    "fresh command."
                )
                # Reboot hygiene: drop the latch and halt TX so the car cannot auto-resume
                # into a stale command; a fresh cmd_drive re-arms it (the TX heading offset
                # re-derives from the imu/odom streams). The onboard odo resets to ~0, so flag
                # a stitch to keep wheel/state.distance continuous. Yaw is compass-absolute and
                # needs no re-baseline (interface.md). Reader thread -> guard shared TX state.
                self._reboot_pending = True
                self._last_seq = None  # seq restarts at 0 on reboot; don't count the restart as a gap
                self._last_imu_t = None  # group t restarts near 0 on reboot; clear the gate baselines
                self._last_uwb_t = None  #   or the monotonic gate would reject every post-reboot frame
                with self._tx_lock:
                    self._latched = None
                    self._heading_offset_deg = None
                    self._last_device_yaw_deg = None
                    self._last_device_yaw_ns = None
                    self._halt = True
            self._clock_offset_ns = self.get_clock().now().nanoseconds - ts_ns

        self._last_ts_ms = ts_ms
        return ts_ns + self._clock_offset_ns

    def _map_device_ns(self, ms):
        """Map an ESP32 millis() timestamp to ROS-clock ns via the current offset (no side effects)."""
        return int(ms) * 1_000_000 + self._clock_offset_ns

    def _set_stamp(self, msg_header, ros_ns):
        """Write a ROS-clock ns value into a header stamp (sec/nanosec)."""
        msg_header.stamp.sec = ros_ns // 1_000_000_000
        msg_header.stamp.nanosec = ros_ns % 1_000_000_000

    def _read_loop(self):
        """Open the port (retrying) and publish parsed frames until shutdown."""
        while not self._stop.is_set() and rclpy.ok():
            try:
                ser = self._open_port()
            except serial.SerialException as exc:
                self.get_logger().warn(
                    f"Cannot open {self.port}: {exc}. Retrying in 2s..."
                )
                self._stop.wait(2.0)
                continue

            with self._ser_lock:
                self._ser = ser
            self.get_logger().info(f"Serial open on {self.port} @ {self.baud} baud")
            buf = bytearray()
            try:
                while not self._stop.is_set() and rclpy.ok():
                    # Bulk-drain: block for at least one byte (bounded by the port timeout), then
                    # grab everything else already buffered in one shot. read(in_waiting) returns
                    # instantly, so this never blocks waiting for a byte count that won't arrive.
                    chunk = ser.read(1)
                    if not chunk:
                        continue  # read timeout, loop to re-check shutdown
                    n = ser.in_waiting
                    if n:
                        chunk += ser.read(n)
                    buf.extend(chunk)
                    while True:
                        nl = buf.find(b"\n")
                        if nl < 0:
                            break
                        line = bytes(buf[:nl])
                        del buf[:nl + 1]
                        self._process_serial_line(line)
                    if len(buf) > SERIAL_BUF_MAX:
                        buf.clear()  # partial line past the cap with no framing: drop, resync on next '\n'
            except serial.SerialException as exc:
                self.get_logger().warn(f"Serial error: {exc}. Reconnecting...")
            finally:
                with self._ser_lock:
                    self._ser = None
                try:
                    ser.close()
                except Exception:
                    pass

    def _process_serial_line(self, raw):
        """Decode, filter, parse, and publish one newline-stripped serial line."""
        line = raw.decode("utf-8", errors="replace").strip()
        if not line.startswith("{"):
            return  # skip interleaved ESP-IDF log lines
        try:
            frame = json.loads(line)
        except json.JSONDecodeError:
            self._parse_fail_count += 1
            self.get_logger().warning(
                f"Unparseable serial line: {line[:LOG_SNIPPET_CHARS]!r}",
                throttle_duration_sec=2.0,
            )
            return
        try:
            self._publish(frame)
        except Exception as exc:
            # A bad field value (e.g. null/non-numeric -> float()/int()) must not
            # kill the reader thread; surface it on /rosout and keep reading.
            self._parse_fail_count += 1
            self.get_logger().warning(
                f"Dropped telemetry frame ({type(exc).__name__}: {exc}); "
                f"line: {line[:LOG_SNIPPET_CHARS]!r}",
                throttle_duration_sec=2.0,
            )

    def _publish(self, f):
        """Dispatch one JSON frame: no "type" key = flat telemetry; typed frames branch."""
        # Branch BEFORE any ts handling: a ts-less typed frame must not look like an
        # ESP32 reboot to the clock-offset logic in _device_ts_to_ros_ns.
        ftype = f.get("type")
        if ftype is None:
            self._publish_telemetry(f)
        elif ftype == "log":
            self._handle_log_frame(f)
        elif ftype == "health":
            self._handle_health_frame(f)
        elif ftype not in self._unknown_frame_types:
            self._unknown_frame_types.add(ftype)
            self.get_logger().warning(
                f"Unknown serial frame type '{ftype}'; ignoring (warned once)")

    def _handle_log_frame(self, f):
        """Re-log an ESP32 log event frame at its mapped severity (reaches /rosout)."""
        # rclpy binds a severity to each logging call site (file/function/line) and raises
        # "Logger severity cannot be changed between calls." if one site logs at more than one
        # severity. Routing every level through a shared level_fn() makes line-N that single site,
        # so the second distinct severity throws — each level must dispatch from its own line.
        log = self.get_logger()
        level = str(f.get("level", "info")).lower()
        msg = f"[esp32] {str(f.get('msg', ''))}"
        if level == "fatal":
            log.fatal(msg)
        elif level == "error":
            log.error(msg)
        elif level in ("warn", "warning"):
            log.warning(msg)
        elif level == "debug":
            log.debug(msg)
        else:
            log.info(msg)

    def _prune_telem_window(self, now_ns):
        """Drop telemetry timestamps older than the 1 s received-rate window."""
        cutoff = now_ns - 1_000_000_000
        stamps = self._telem_stamps
        while stamps and stamps[0] < cutoff:
            stamps.popleft()

    def _telem_watchdog_tick(self):
        """Warn (at most 1/s) while no telemetry frame has landed for over TELEM_SILENCE_LIMIT_NS.

        Threading, baseline choice, and message wording: see NOTES.md.
        """
        last_ns = self._last_frame_ns
        silent_ns = self.get_clock().now().nanoseconds - (
            self._node_start_ns if last_ns is None else last_ns
        )
        if silent_ns <= TELEM_SILENCE_LIMIT_NS:
            return
        detail = " (none received since startup)" if last_ns is None else ""
        self.get_logger().warning(
            f"No ESP32 telemetry for {silent_ns / 1e9:.1f} s "
            f"(link silent or all frames dropping).{detail}",
            throttle_duration_sec=1.0,
        )

    def _handle_health_frame(self, f):
        """Publish a {"type":"health"} frame's per-sensor rates on sensor_health."""
        sensors = f.get("sensors")
        if not isinstance(sensors, dict):
            self.get_logger().warning("health frame without a 'sensors' object; dropped")
            return
        msg = SensorHealth()
        # Stamped with arrival time: health frames carry no device ts, and at ~1-2 Hz the
        # serial latency is irrelevant — staleness detection is the field that matters.
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        # Worst control-loop gap since the last health frame (us); top-level, not a sensor rate.
        try:
            msg.max_loop_us = int(f.get("max_loop_us", 0))
        except (TypeError, ValueError):
            msg.max_loop_us = 0
        # Pi-derived received-telemetry rate: successful telemetry frames parsed in the last 1 s.
        self._prune_telem_window(self.get_clock().now().nanoseconds)
        msg.telem_frames_1s = len(self._telem_stamps)
        # ESP-reported cumulative telemetry TX drops (buffer full); passthrough.
        try:
            msg.tx_drops = int(f.get("tx_drops", 0))
        except (TypeError, ValueError):
            msg.tx_drops = 0
        # Pi-derived link health: cumulative seq gaps + parse failures, and the worst inter-frame
        # gap since the last emit (windowed -> reset here). Reader-thread-local, so no lock.
        msg.seq_gaps = self._seq_gaps
        msg.parse_fails = self._parse_fail_count
        msg.max_inter_frame_gap_ms = self._max_gap_ns_window / 1e6
        self._max_gap_ns_window = 0
        for name, rate in sensors.items():
            try:
                msg.rates_hz.append(float(rate))
            except (TypeError, ValueError):
                continue  # skip the pair; a bad value must not drop the whole report
            msg.names.append(str(name))
        self.pub_sensor_health.publish(msg)

    def _publish_telemetry(self, f):
        """Dispatch one grouped telemetry frame: map the envelope clock, then publish per group."""
        ts = f.get("ts", 0)
        # Envelope clock: reboot detection + offset capture live here, keyed on the frame ts. Async
        # groups stamp at their own t (via _map_device_ns); loop-sync groups (wheel/ctrl) stamp at ts.
        ts_ns = self._device_ts_to_ros_ns(ts)

        # Sequence-gap accounting: a forward "seq" jump > 1 = frames lost; a backwards/oversized jump
        # is a reboot/wrap/glitch, so resync without counting (the reboot itself is handled above).
        seq = f.get("seq")
        if seq is not None:
            seq = int(seq)
            if self._last_seq is not None:
                delta = seq - self._last_seq
                if 1 <= delta <= SEQ_GAP_MAX:
                    self._seq_gaps += delta - 1
            self._last_seq = seq

        # Per-group publish; a missing group just skips its topic (each frame is a full snapshot,
        # but tolerate absence). Async groups carry their own t; loop-sync groups ride the envelope.
        # wheel/state goes before imu/data: pose_estimator caches the odometer on wheel/state and
        # integrates it on imu/data, so the co-sampled odo must land first (see NOTES.md).
        wheel = f.get("wheel")
        if wheel is not None:
            self._publish_wheel(wheel, ts_ns)
        imu = f.get("imu")
        if imu is not None:
            self._publish_imu(imu)
        cmd = f.get("cmd")
        if cmd is not None:
            self._publish_cmd(cmd, ts)
        ctrl = f.get("ctrl")
        if ctrl is not None:
            self._publish_ctrl(ctrl, ts_ns)
        uwb = f.get("uwb")
        if uwb is not None:
            self._publish_uwb(uwb, ts)

        # Received-telemetry-rate window + worst inter-frame gap (jitter), reported on sensor_health.
        now_ns = self.get_clock().now().nanoseconds
        if self._last_frame_ns is not None:
            self._max_gap_ns_window = max(self._max_gap_ns_window, now_ns - self._last_frame_ns)
        self._last_frame_ns = now_ns
        self._telem_stamps.append(now_ns)
        self._prune_telem_window(now_ns)

    def _publish_imu(self, imu):
        """Publish imu/data from the async IMU group; gated + stamped on its capture time imu.t."""
        t = int(imu.get("t", -1))
        if t < 0 or (self._last_imu_t is not None and t <= self._last_imu_t):
            return  # never produced, or no new capture since the last publish (stale repeat)
        self._last_imu_t = t
        stamp_ns = self._map_device_ns(t)

        msg = Imu()
        self._set_stamp(msg.header, stamp_ns)
        msg.header.frame_id = self.imu_frame_id
        yaw_deg = float(imu.get("yaw", 0.0))
        qx, qy, qz, qw = euler_deg_to_quaternion(
            float(imu.get("roll", 0.0)), float(imu.get("pitch", 0.0)), yaw_deg)
        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw
        # covariance[0]=0 = value available, variance unknown. yaw_rate sign matches yaw; lax is the
        # forward-axis accel (BNO085 x); other axes are not separately measured.
        msg.angular_velocity.z = float(imu.get("yaw_rate", 0.0)) * DEG_TO_RAD  # deg/s -> rad/s
        msg.angular_velocity_covariance[0] = 0.0
        msg.linear_acceleration.x = float(imu.get("lax", 0.0))  # m/s^2, forward
        msg.linear_acceleration_covariance[0] = 0.0
        self.pub_imu.publish(msg)

        # Cache device yaw with its capture time so _on_odom can time-pair the TX heading offset.
        with self._tx_lock:
            self._last_device_yaw_deg = yaw_deg
            self._last_device_yaw_ns = stamp_ns

    def _publish_uwb(self, uwb, ts):
        """Publish uwb/raw from the async ranging group; gated + stamped on its capture time uwb.t."""
        t = int(uwb.get("t", -1))
        if t < 0 or (self._last_uwb_t is not None and t <= self._last_uwb_t):
            return  # never fixed, or no new ranging frame since the last publish
        self._last_uwb_t = t

        msg = UwbRaw()
        self._set_stamp(msg.header, self._map_device_ns(t))
        msg.header.frame_id = self.frame_id
        # Preserve the no-fix sentinel: wire -1 stays -1 (scaling would masquerade as a near-zero range).
        raw_dist = float(uwb.get("dist", -1.0))
        msg.distance = -1.0 if raw_dist < 0.0 else raw_dist  # m (SI on wire), -1 = no fix
        # Sign correction: device +ve = tag RIGHT, REP-103 +ve = LEFT (CCW about +z).
        msg.bearing = -float(uwb.get("bearing", 0.0)) * DEG_TO_RAD  # deg -> rad, sign-corrected
        msg.age_ms = int(ts) - t  # ms since the fix was captured, as of frame emit
        self.pub_uwb_raw.publish(msg)

    def _publish_cmd(self, cmd, ts):
        """Publish command/status from the async cmd-echo group; stamped at cmd.t, published every frame."""
        # Not gated: cmd_rejects and cmd_age_ms must flow even when no new command is accepted (a
        # reject does not advance cmd.t). Staleness rides both the header stamp and cmd_age_ms.
        t = int(cmd.get("t", -1))
        msg = CommandStatus()
        if t >= 0:
            self._set_stamp(msg.header, self._map_device_ns(t))
            cmd_age_ms = int(ts) - t
        else:
            # No command accepted since boot: no receipt time to stamp with -> use the frame ts.
            self._set_stamp(msg.header, self._map_device_ns(ts))
            cmd_age_ms = -1
        msg.header.frame_id = self.frame_id
        msg.cmd_speed = float(cmd.get("speed", 0.0))  # m/s (SI on wire)
        # cmd.heading is device compass DEGREES; convert to the odom frame with the SAME offset the
        # TX path tracks, so the echo is comparable to the DriveCommand we sent. Until the offset
        # exists, fall back to the device-frame heading in radians.
        cmd_heading_dev_deg = float(cmd.get("heading", 0.0))
        with self._tx_lock:
            offset_deg = self._heading_offset_deg
        if offset_deg is not None:
            msg.cmd_heading = normalize_rad(math.radians(cmd_heading_dev_deg - offset_deg))
        else:
            msg.cmd_heading = math.radians(cmd_heading_dev_deg)  # device frame until offset
        msg.cmd_pan = float(cmd.get("pan", 0.0)) * DEG_TO_RAD  # deg -> rad
        msg.cmd_throttle = float(cmd.get("throttle", 0.0))  # DIRECT echo, dimensionless
        msg.cmd_steering = float(cmd.get("steering", 0.0))  # DIRECT echo, dimensionless
        msg.cmd_age_ms = cmd_age_ms
        msg.cmd_rejects = int(cmd.get("rejects", 0))
        self.pub_command_status.publish(msg)

    def _publish_wheel(self, wheel, ts_ns):
        """Publish wheel/state from the loop-synchronous drivetrain-fusion group; stamped at ts."""
        msg = WheelState()
        self._set_stamp(msg.header, ts_ns)
        msg.header.frame_id = self.frame_id
        msg.speed = float(wheel.get("speed", 0.0))  # m/s (SI on wire), signed (< 0 = reverse)
        msg.speed_variance = float(wheel.get("speed_var", 0.0))  # (m/s)^2 (SI on wire)
        # Stitch the odometer continuous across reboots: the onboard odo resets to ~0, so on a
        # pending reboot re-anchor the offset to the last continuous value.
        raw_dist_m = float(wheel.get("odo", 0.0))  # m (SI on wire), signed
        if self._reboot_pending:
            if self._last_cont_dist_m is not None:
                self._odo_offset_m = self._last_cont_dist_m - raw_dist_m
            self._reboot_pending = False
        msg.distance = self._odo_offset_m + raw_dist_m
        self._last_cont_dist_m = msg.distance
        msg.cogging = bool(wheel.get("cogging", 0))  # latching
        msg.enc_fault = bool(wheel.get("enc_fault", 0))  # absent = healthy
        self.pub_wheel.publish(msg)

    def _publish_ctrl(self, ctrl, ts_ns):
        """Publish actuator/status + joint_states from the loop-synchronous control group; stamped at ts."""
        act = ActuatorStatus()
        self._set_stamp(act.header, ts_ns)
        act.header.frame_id = self.frame_id
        steering = float(ctrl.get("steering", 0.0))  # normalized [-1, 1]
        pan_deg = float(ctrl.get("pan_angle", 0.0))
        act.throttle = float(ctrl.get("throttle", 0.0))  # normalized control output; < 0 = braking
        act.steering = steering
        act.pan_angle = pan_deg * DEG_TO_RAD  # deg -> rad
        # Raw servo/ESC pulse widths actually written (us, ~1500 = neutral); no SI equivalent.
        act.esc_pwm = int(ctrl.get("esc_pwm", 1500))
        act.steer_pwm = int(ctrl.get("steer_pwm", 1500))
        act.pan_pwm = int(ctrl.get("pan_pwm", 1500))
        self.pub_actuator_status.publish(act)

        # Joint states: pan drives base_link -> uwb_link, negated (wire +right -> TF +z left); the
        # front steer joints are cosmetic, scaled by MAX_STEER_RAD (see STEER_* constants; NOTES.md).
        js = JointState()
        self._set_stamp(js.header, ts_ns)
        js.header.frame_id = self.frame_id
        steer_rad = STEER_SIGN * steering * MAX_STEER_RAD
        js.name = [PAN_JOINT_NAME] + STEER_JOINT_NAMES
        js.position = [-pan_deg * DEG_TO_RAD, steer_rad, steer_rad]  # deg +right -> rad +z
        self.pub_joints.publish(js)

    def _on_cmd_drive(self, msg):
        """Latch the newest valid drive command (SETPOINT or DIRECT). Never writes serial — the timer does."""
        shape = int(msg.shape)
        vals = ((msg.throttle, msg.steering, msg.pan_deg)
                if shape == DriveCommand.DIRECT else (msg.speed, msg.heading))
        if not all(math.isfinite(v) for v in vals):
            # Non-finite would serialize to NaN/Inf JSON and trip ESP32 validation; drop it.
            self.get_logger().warn(
                "cmd_drive with non-finite field(s); ignoring.",
                throttle_duration_sec=1.0,
            )
            return

        # Treat a zero header stamp as arrival time; otherwise honor the sender's stamp
        # so genuine staleness still gates TX (see NOTES.md).
        stamp = msg.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            stamp_ns = self.get_clock().now().nanoseconds
        else:
            stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec

        with self._tx_lock:
            self._latched = {
                "shape": shape, "speed": msg.speed, "heading": msg.heading,
                "throttle": msg.throttle, "steering": msg.steering, "pan_deg": msg.pan_deg,
                "stamp_ns": stamp_ns,
            }
            self._halt = False  # a fresh command re-arms TX after a reboot halt

    def _on_odom(self, msg):
        """Track the outbound heading offset (EMA of device-yaw minus odom-yaw) from time-paired samples."""
        odom_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        q = msg.pose.pose.orientation
        odom_yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)

        with self._tx_lock:
            dev_deg = self._last_device_yaw_deg
            dev_ns = self._last_device_yaw_ns
            if dev_deg is None or dev_ns is None:
                return  # no device-yaw sample yet
            if abs(odom_ns - dev_ns) > OFFSET_SYNC_NS:
                return  # samples too far apart in time to pair reliably
            new_offset = wrap_pm180(dev_deg - math.degrees(odom_yaw))
            if self._heading_offset_deg is None:
                self._heading_offset_deg = new_offset
            else:
                self._heading_offset_deg += OFFSET_EMA_ALPHA * wrap_pm180(
                    new_offset - self._heading_offset_deg
                )

    def _tx_tick(self):
        """Write one frame — the latched command, when fresh — at 20 Hz."""
        with self._tx_lock:
            if self._halt:
                return  # halted after an ESP32 reboot until a fresh cmd_drive arrives
            latched = self._latched
            offset_deg = self._heading_offset_deg

        if latched is None:
            return  # no command latched yet

        # Staleness is silence (NOT a zero command) — that is what trips the ESP32 failsafe.
        if self.get_clock().now().nanoseconds - latched["stamp_ns"] > CMD_STALE_NS:
            return

        if latched["shape"] == DriveCommand.DIRECT:
            # Raw-actuator frame: throttle/steering pass through [-1, 1], pan in degrees. No heading
            # offset is needed (frame-relative), so DIRECT sends immediately, unlike SETPOINT.
            frame = '{"throttle":%.3f,"steering":%.3f,"target_pan":%.1f}\n' % (
                latched["throttle"], latched["steering"], latched["pan_deg"])
        else:
            # Setpoint frame needs the device/odom heading offset; a guessed heading would steer a
            # real car wrong, so hold TX until the offset is known.
            if offset_deg is None:
                self.get_logger().warn(
                    "No heading offset yet (need paired odom + device yaw); not sending.",
                    throttle_duration_sec=2.0,
                )
                return
            # ESP32 wire contract (SI m/s + compass deg); the ESP32 validates/clamps target_speed.
            target_speed = latched["speed"]  # m/s (SI on wire)
            target_heading = wrap_0_360(math.degrees(latched["heading"]) + offset_deg)
            frame = '{"target_speed":%.3f,"target_heading":%.1f}\n' % (target_speed, target_heading)

        with self._ser_lock:
            ser = self._ser
            if ser is None:
                return  # port closed/absent -> drop silently (never queue)
            try:
                ser.write(frame.encode("ascii"))
            except (serial.SerialException, serial.SerialTimeoutException):
                # Wedged/closed port: drop this tick; the reader loop owns reconnect.
                self.get_logger().debug(
                    "TX write failed; dropping tick.",
                    throttle_duration_sec=2.0,
                )


def main(args=None):
    """Init rclpy, spin the SerialBridge node, and shut down cleanly."""
    rclpy.init(args=args)
    node = SerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
