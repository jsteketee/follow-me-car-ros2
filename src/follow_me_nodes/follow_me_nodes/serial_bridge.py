#!/usr/bin/env python3
"""Serial bridge: ESP32-S3 JSON telemetry <-> ROS2 topics (SI/REP-103 conversion boundary).
Reads ~50 Hz JSON frames and republishes as topics; writes drive commands back at 20 Hz.
Topic map, naming convention, and stamp mapping: see NOTES.md; frame schema in PROJECT_PLAN.md.
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
    DisplayFlag,
    DriveCommand,
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

# Subscribed (relative): drive setpoints in, odom for the outbound heading offset.
TOPIC_CMD_DRIVE = "cmd_drive"
TOPIC_ODOM = "odom"
# Display flag events, merged into the outbound TX frame (see _tx_tick).
TOPIC_DISPLAY_FLAG = "display/flag"

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
# The ESP32 speaks mph / cm / degrees. ROS2 mandates SI: metres, m/s, radians.
# This node is the boundary between the two, so it converts here, once.
# Everything downstream is SI by construction and needs no scale factors.
# ---------------------------------------------------------------------------
MPH_TO_MPS = 0.44704  # exact by definition (1 mile = 1609.344 m, 1 h = 3600 s)
CM_TO_M = 0.01
DEG_TO_RAD = math.pi / 180.0

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

        # Device-clock -> ROS-clock offset, captured on the first frame.
        self._clock_offset_ns = None
        self._last_ts_ms = None

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
        # Display flags queue up here and ride the next TX frame (one op per tick).
        self.sub_flag = self.create_subscription(
            DisplayFlag, TOPIC_DISPLAY_FLAG, self._on_display_flag, 10
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
        self._pending_flags = deque()    # (text, action) display ops awaiting TX

        self._tx_timer = self.create_timer(CMD_TX_PERIOD_S, self._tx_tick)

        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def _open_port(self):
        """Open the serial port with read/write timeouts (test seam: monkeypatched)."""
        return serial.serial_for_url(
            self.port, baudrate=self.baud, timeout=1.0, write_timeout=0.1
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
                self.get_logger().warn(
                    f"ESP32 clock went backwards ({self._last_ts_ms} -> {ts_ms} ms); "
                    "device rebooted? Recapturing clock offset, halting TX."
                )
                # Reboot: the device yaw reference may have moved, so invalidate the offset
                # and drop the latch; halt TX until a fresh cmd_drive re-arms it. Runs on
                # the reader thread, so guard the shared TX state.
                with self._tx_lock:
                    self._latched = None
                    self._heading_offset_deg = None
                    self._last_device_yaw_deg = None
                    self._last_device_yaw_ns = None
                    self._halt = True
            self._clock_offset_ns = self.get_clock().now().nanoseconds - ts_ns

        self._last_ts_ms = ts_ms
        return ts_ns + self._clock_offset_ns

    def _stamp_from_ts(self, msg_header, ts_ms):
        """Stamp a header with the ESP32 device time, mapped into the ROS clock."""
        total_ns = self._device_ts_to_ros_ns(ts_ms)
        msg_header.stamp.sec = total_ns // 1_000_000_000
        msg_header.stamp.nanosec = total_ns % 1_000_000_000
        msg_header.frame_id = self.frame_id

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
            try:
                while not self._stop.is_set() and rclpy.ok():
                    raw = ser.readline()
                    if not raw:
                        continue  # read timeout, loop to re-check shutdown
                    line = raw.decode("utf-8", errors="replace").strip()
                    if not line.startswith("{"):
                        continue  # skip interleaved ESP-IDF log lines
                    try:
                        frame = json.loads(line)
                    except json.JSONDecodeError:
                        continue
                    self._publish(frame)
            except serial.SerialException as exc:
                self.get_logger().warn(f"Serial error: {exc}. Reconnecting...")
            finally:
                with self._ser_lock:
                    self._ser = None
                try:
                    ser.close()
                except Exception:
                    pass

    def _publish(self, f):
        """Convert one JSON frame to SI and publish all telemetry topics."""
        ts = f.get("ts", 0)

        # --- Drivetrain state (fused speed + odometer + cogging flag), converted to SI ---
        # One co-sampled message, one stamp; published before imu/data (see NOTES.md).
        wheel = WheelState()
        self._stamp_from_ts(wheel.header, ts)
        wheel.speed = float(f.get("speed", 0.0)) * MPH_TO_MPS  # mph -> m/s (ESP32 fused estimate)
        wheel.distance = float(f.get("odo", 0.0)) * CM_TO_M  # cm -> m
        wheel.cogging = bool(f.get("cogging", 0))  # wire 0/1 -> bool, latching
        self.pub_wheel.publish(wheel)

        # --- IMU: orientation from yaw/pitch/roll (degrees) ---
        imu = Imu()
        self._stamp_from_ts(imu.header, ts)
        imu.header.frame_id = self.imu_frame_id
        qx, qy, qz, qw = euler_deg_to_quaternion(
            float(f.get("roll", 0.0)),
            float(f.get("pitch", 0.0)),
            float(f.get("yaw", 0.0)),
        )
        imu.orientation.x = qx
        imu.orientation.y = qy
        imu.orientation.z = qz
        imu.orientation.w = qw
        # covariance[0]=0 = value available, variance unknown. yaw_rate unnegated (matches
        # yaw); lax is forward-axis accel (BNO085 x); other axes not separately measured.
        imu.angular_velocity.z = float(f.get("yaw_rate", 0.0)) * DEG_TO_RAD  # deg/s -> rad/s
        imu.angular_velocity_covariance[0] = 0.0  # available now; variance unknown
        imu.linear_acceleration.x = float(f.get("lax", 0.0))  # m/s^2, forward (BNO085 x)
        imu.linear_acceleration_covariance[0] = 0.0  # available now; variance unknown
        self.pub_imu.publish(imu)

        # --- Command echo/status: is the ESP32 seeing and accepting our commands? ---
        cmd = CommandStatus()
        self._stamp_from_ts(cmd.header, ts)
        cmd.mode = str(f.get("mode", ""))  # SETPOINT / DIRECT / STOPPED
        cmd.cmd_speed = float(f.get("cmd_speed", 0.0)) * MPH_TO_MPS  # mph -> m/s
        # The wire reports cmd_heading in device compass DEGREES. Convert to the odom frame
        # (per CommandStatus.msg) with the SAME offset the TX path tracks, so the echo is
        # directly comparable to the DriveCommand we sent; until the offset exists, fall
        # back to the device-frame heading in radians.
        cmd_heading_dev_deg = float(f.get("cmd_heading", 0.0))
        with self._tx_lock:
            offset_deg = self._heading_offset_deg
        if offset_deg is not None:
            cmd.cmd_heading = normalize_rad(math.radians(cmd_heading_dev_deg - offset_deg))
        else:
            cmd.cmd_heading = math.radians(cmd_heading_dev_deg)  # device frame until offset
        cmd.cmd_pan = float(f.get("cmd_pan", 0.0)) * DEG_TO_RAD  # deg -> rad
        cmd.cmd_age_ms = int(f.get("cmd_age", -1))  # ms since last accepted; -1 = none yet
        cmd.cmd_rejects = int(f.get("cmd_rejects", 0))
        self.pub_command_status.publish(cmd)

        # --- Actuator outputs: what the drivetrain/servos are actually doing ---
        act = ActuatorStatus()
        self._stamp_from_ts(act.header, ts)
        act.throttle = float(f.get("throttle", 0.0))  # normalized [0, 1], dimensionless
        act.steering = float(f.get("steering", 0.0))  # normalized [-1, 1], dimensionless
        act.pan_angle = float(f.get("pan_angle", 0.0)) * DEG_TO_RAD  # deg -> rad
        self.pub_actuator_status.publish(act)

        # --- Joint states: pan + front steer (via robot_state_publisher) ---
        # Pan drives base_link -> uwb_link, negated (wire +right -> TF +z left). Steer
        # joints cosmetic, scaled by MAX_STEER_RAD (see STEER_* constants; NOTES.md).
        steer_rad = STEER_SIGN * float(f.get("steering", 0.0)) * MAX_STEER_RAD
        js = JointState()
        self._stamp_from_ts(js.header, ts)
        js.name = [PAN_JOINT_NAME] + STEER_JOINT_NAMES
        js.position = [-float(f.get("pan_angle", 0.0)) * DEG_TO_RAD,  # deg +right -> rad +z
                       steer_rad, steer_rad]
        self.pub_joints.publish(js)

        # --- DW3000 range/bearing to the tag ---
        uwb = UwbRaw()
        self._stamp_from_ts(uwb.header, ts)
        # Preserve the no-fix sentinel: the wire sends -1 (not a real range) when the tag
        # isn't ranged; scaling that to -0.01 m would masquerade as a valid near-zero range.
        raw_dist = float(f.get("uwb_dist", -1.0))
        uwb.distance = -1.0 if raw_dist < 0.0 else raw_dist * CM_TO_M  # cm -> m
        # Sign correction: device +ve = tag RIGHT, REP-103 +ve = LEFT (CCW about +z).
        uwb.bearing = -float(f.get("uwb_bearing", 0.0)) * DEG_TO_RAD  # deg -> rad, sign-corrected
        uwb.age_ms = int(f.get("uwb_age", -1))  # ms since fix; -1 = no fix / not reported
        self.pub_uwb_raw.publish(uwb)

        # Cache device yaw with its ROS-mapped time for _on_odom to pair (heading offset).
        # Map the time OUTSIDE the lock (see NOTES.md).
        device_yaw_deg = float(f.get("yaw", 0.0))
        device_yaw_ns = self._device_ts_to_ros_ns(ts)
        with self._tx_lock:
            self._last_device_yaw_deg = device_yaw_deg
            self._last_device_yaw_ns = device_yaw_ns

    def _on_cmd_drive(self, msg):
        """Latch the newest valid drive command. Never writes serial — the timer does."""
        speed = msg.speed
        heading = msg.heading
        if not (math.isfinite(speed) and math.isfinite(heading)):
            # Non-finite would serialize to NaN/Inf JSON and trip ESP32 validation; drop it.
            self.get_logger().warn(
                "cmd_drive with non-finite speed/heading; ignoring.",
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
            self._latched = (speed, heading, stamp_ns)
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

    def _on_display_flag(self, msg):
        """Queue a display flag op (text, action) for the next TX frame."""
        with self._tx_lock:
            self._pending_flags.append((msg.text, int(msg.action)))

    def _tx_tick(self):
        """Write one frame — the latched command (if fresh) and/or one queued flag — at 20 Hz."""
        with self._tx_lock:
            if self._halt:
                return  # halted after an ESP32 reboot until a fresh cmd_drive arrives
            latched = self._latched
            offset_deg = self._heading_offset_deg
            flag = self._pending_flags.popleft() if self._pending_flags else None

        parts = []

        # Command part: only when a fresh latch AND a known heading offset exist. Staleness
        # is silence (NOT a zero command) — that is what trips the ESP32 failsafe.
        if latched is not None:
            speed_mps, heading_rad, stamp_ns = latched
            fresh = self.get_clock().now().nanoseconds - stamp_ns <= CMD_STALE_NS
            if fresh and offset_deg is not None:
                # ESP32 wire contract (mph / compass deg). No Pi-side gating: the ESP32
                # validates target_speed and rejects out-of-range frames itself.
                target_speed = speed_mps / MPH_TO_MPS
                target_heading = wrap_0_360(math.degrees(heading_rad) + offset_deg)
                parts.append(
                    '"target_speed":%.2f,"target_heading":%.1f' % (target_speed, target_heading)
                )
            elif fresh:
                # No offset, no command: a guessed heading offset would steer a real car wrong.
                self.get_logger().warn(
                    "No heading offset yet (need paired odom + device yaw); not sending.",
                    throttle_duration_sec=2.0,
                )

        # Flag part: display op, NOT gated by command staleness. json.dumps escapes the text.
        if flag is not None:
            text, action = flag
            parts.append('"flag":%s,"flag_action":%d' % (json.dumps(text), action))

        if not parts:
            return  # nothing to send this tick

        frame = "{" + ",".join(parts) + "}\n"

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
