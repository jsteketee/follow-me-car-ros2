#!/usr/bin/env python3
"""Serial bridge node: ESP32-S3 JSON telemetry -> ROS2 topics.

Reads newline-delimited JSON frames from the ESP32 over USB serial (~50 Hz) and
republishes them as ROS2 topics. Non-JSON lines (interleaved ESP-IDF logs) are
skipped. See PROJECT_PLAN.md "Serial Protocol" for the frame schema.

All published values are SI (REP-103): metres, m/s, radians. The ESP32 speaks mph,
cm and degrees; this node is the boundary and converts once, here.

Header stamps are the ESP32's device time mapped into the ROS clock (see
_device_ts_to_ros_ns), so dt stays exact device time while tf2 still accepts them.

Published topics (names are RELATIVE — see TOPIC NAMING below):
  imu/data        sensor_msgs/Imu                   (orientation from yaw/pitch/roll)
  wheel/speed     std_msgs/Float32                  (hall-effect speed, m/s)
  wheel/distance  std_msgs/Float32                  (accumulated odometer reading, m)
  tag/pose        follow_me_interfaces/FusedTagPose (ESP32 Kalman-fused bearing/dist to tag)

TOPIC NAMING
    Every topic this node owns is declared relative (no leading "/"), so the robot
    namespace is set at launch rather than baked into the source. Launching under the
    namespace "fmbot" resolves them to /fmbot/imu/data, /fmbot/wheel/speed, and so on,
    which is what makes multi-robot possible without touching this file.

"""

import json
import math
import threading

import rclpy
from rclpy.node import Node

import serial

from std_msgs.msg import Float32
from sensor_msgs.msg import Imu
from follow_me_interfaces.msg import FusedTagPose

DEFAULT_PORT = (
    "/dev/serial/by-id/"
    "usb-Espressif_USB_JTAG_serial_debug_unit_58:E6:C5:57:46:98-if00"
)

# ---------------------------------------------------------------------------
# Project-defined topics — RELATIVE names, namespaced per robot at launch.
#
# No leading "/". A relative name is resolved against the node's namespace, so
# `--ros-args -r __ns:=/fmbot` yields /fmbot/imu/data. An absolute name (leading
# "/") would ignore the namespace entirely and break multi-robot.
# ---------------------------------------------------------------------------
TOPIC_IMU = "imu/data"
TOPIC_WHEEL_SPEED = "wheel/speed"
TOPIC_WHEEL_DISTANCE = "wheel/distance"
TOPIC_TAG_POSE = "tag/pose"

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
DEG2_TO_RAD2 = DEG_TO_RAD * DEG_TO_RAD  # variance scales by the square of the unit factor


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


class SerialBridge(Node):
    def __init__(self):
        super().__init__("serial_bridge")

        self.port = self.declare_parameter("serial_port", DEFAULT_PORT).value
        self.baud = self.declare_parameter("baud", 115200).value
        # TF frame ids. Namespacing does NOT prefix these — frame ids live in the
        # global TF tree, so multi-robot needs them set explicitly per robot
        # (e.g. fmbot/base_link) via these parameters at launch.
        self.frame_id = self.declare_parameter("frame_id", "base_link").value
        self.imu_frame_id = self.declare_parameter("imu_frame_id", "imu_link").value

        self.pub_imu = self.create_publisher(Imu, TOPIC_IMU, 10)
        self.pub_speed = self.create_publisher(Float32, TOPIC_WHEEL_SPEED, 10)
        self.pub_odo = self.create_publisher(Float32, TOPIC_WHEEL_DISTANCE, 10)
        self.pub_fused = self.create_publisher(FusedTagPose, TOPIC_TAG_POSE, 10)

        # Device-clock -> ROS-clock offset, captured on the first frame.
        self._clock_offset_ns = None
        self._last_ts_ms = None

        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def destroy_node(self):
        self._stop.set()
        if self._thread.is_alive():
            self._thread.join(timeout=2.0)
        super().destroy_node()

    def _device_ts_to_ros_ns(self, ts_ms):
        """Map ESP32 uptime (ms) into ROS-clock ns via a constant offset caught on frame 1.

        Raw device uptime would stamp messages in 1970 and tf2 would drop them. The offset
        cancels in subtraction, so dt stays exact device time (PROJECT_PLAN "Serial Protocol").
        """
        ts_ns = int(ts_ms) * 1_000_000

        # First frame, or ESP32 rebooted (uptime jumped backwards) -> (re)capture offset.
        if self._clock_offset_ns is None or (
            self._last_ts_ms is not None and ts_ms < self._last_ts_ms
        ):
            if self._clock_offset_ns is not None:
                self.get_logger().warn(
                    f"ESP32 clock went backwards ({self._last_ts_ms} -> {ts_ms} ms); "
                    "device rebooted? Recapturing clock offset."
                )
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
        """Open the port (retrying on failure) and publish parsed frames until shutdown."""
        while not self._stop.is_set() and rclpy.ok():
            try:
                ser = serial.Serial(self.port, self.baud, timeout=1.0)
            except serial.SerialException as exc:
                self.get_logger().warn(
                    f"Cannot open {self.port}: {exc}. Retrying in 2s..."
                )
                self._stop.wait(2.0)
                continue

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
                try:
                    ser.close()
                except Exception:
                    pass

    def _publish(self, f):
        ts = f.get("ts", 0)

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
        # Orientation available but covariance unknown -> leave as zeros.
        # Angular velocity / linear acceleration not measured -> flag unavailable (-1).
        imu.angular_velocity_covariance[0] = -1.0
        imu.linear_acceleration_covariance[0] = -1.0
        self.pub_imu.publish(imu)

        # --- Wheel speed / odometry (simple scalar topics), converted to SI ---
        speed = Float32()
        speed.data = float(f.get("speed", 0.0)) * MPH_TO_MPS  # mph -> m/s
        self.pub_speed.publish(speed)

        odo = Float32()
        odo.data = float(f.get("odo", 0.0)) * CM_TO_M  # cm -> m
        self.pub_odo.publish(odo)

        # --- ESP32 fused output (bearing/distance to the tag), converted to SI ---
        fused = FusedTagPose()
        self._stamp_from_ts(fused.header, ts)
        # Negate: the DW3000 reports azimuth +ve = tag to the RIGHT, but REP-103
        # (and every consumer of this message) expects +ve = LEFT (CCW about +z).
        fused.angle = -float(f.get("fused_angle", 0.0)) * DEG_TO_RAD  # deg -> rad, sign-corrected
        fused.distance = float(f.get("fused_dist", 0.0)) * CM_TO_M  # cm -> m
        # Variance, not an angle: scales by the SQUARE of the unit factor.
        fused.uncertainty = float(f.get("fused_unc", 0.0)) * DEG2_TO_RAD2  # deg^2 -> rad^2
        # Snapshot the car's compass heading (yaw) at this fix so the tag's absolute
        # bearing (heading + angle) is computable from this one message.
        fused.heading = float(f.get("yaw", 0.0)) * DEG_TO_RAD  # deg -> rad
        self.pub_fused.publish(fused)


def main(args=None):
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
