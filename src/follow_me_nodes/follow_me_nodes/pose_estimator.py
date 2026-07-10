#!/usr/bin/env python3
"""Dead-reckoning pose estimator: IMU heading + wheel odometry -> 2D pose in `odom`.
"""

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from tf2_ros import TransformBroadcaster

TOPIC_IMU = "imu/data"
TOPIC_WHEEL_DISTANCE = "wheel/distance"
TOPIC_ODOM = "odom"


def yaw_from_quaternion(x, y, z, w):
    """Extract the yaw (rotation about z, radians) from a quaternion."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(a):
    """Wrap an angle to (-pi, pi]; without this, crossing +/-180 deg reads as a ~360 deg turn."""
    return math.atan2(math.sin(a), math.cos(a))


class PoseEstimator(Node):
    """Integrates IMU heading and wheel distance into a 2D pose in the `odom` frame."""

    def __init__(self):
        super().__init__("pose_estimator")

        self.odom_frame = self.declare_parameter("odom_frame", "odom").value
        self.base_frame = self.declare_parameter("base_frame", "base_link").value

        # The IMU callback drives the loop; the odometer is a bare Float32 with no stamp,
        # so it can only be sampled, not time-synchronised.
        self._odo_m = None

        # Accumulated pose, plus the previous step's values.
        self._x = 0.0
        self._y = 0.0
        self._theta = 0.0
        self._yaw_offset = None  # captured on the first frame -> odom starts at identity
        self._prev_odo_m = None
        self._prev_stamp_ns = None

        self.sub_imu = self.create_subscription(Imu, TOPIC_IMU, self._on_imu, 10)
        self.sub_odo = self.create_subscription(
            Float32, TOPIC_WHEEL_DISTANCE, self._on_odometry, 10
        )
        self.pub_odom = self.create_publisher(Odometry, TOPIC_ODOM, 10)
        # Publishes the same pose onto the global /tf tree. This node owns the
        # odom -> base_link edge; nothing else may broadcast it.
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info(
            f"pose_estimator up; subscribed to '{TOPIC_IMU}' and '{TOPIC_WHEEL_DISTANCE}'"
        )

    def _on_odometry(self, msg):
        """Cache the latest accumulated odometer reading (metres)."""
        self._odo_m = msg.data

    def _on_imu(self, msg):
        """Integrate one step: project the odometer delta along the midpoint heading."""
        if self._odo_m is None:
            return  # no odometer sample yet

        yaw_raw = yaw_from_quaternion(
            msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w
        )
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec

        # First frame: establish the baseline, integrate nothing.
        if self._yaw_offset is None:
            self._yaw_offset = yaw_raw
            self._prev_odo_m = self._odo_m
            self._prev_stamp_ns = stamp_ns
            return

        dt = (stamp_ns - self._prev_stamp_ns) / 1e9
        if dt <= 0.0:
            return  # duplicate or stale stamp

        ds = self._odo_m - self._prev_odo_m
        if ds < 0.0:
            # Odometer only ever ticks up, so this means the ESP32 rebooted. Re-baseline
            # rather than teleporting the car backwards.
            self.get_logger().warn(
                f"odometer went backwards ({self._prev_odo_m:.3f} -> {self._odo_m:.3f} m); "
                "ESP32 rebooted? Re-baselining."
            )
            self._prev_odo_m = self._odo_m
            self._prev_stamp_ns = stamp_ns
            return

        theta = normalize_angle(yaw_raw - self._yaw_offset)
        dtheta = normalize_angle(theta - self._theta)

        # The car traced an arc over this step, so project along the midpoint heading —
        # using either endpoint biases every turn to one side.
        theta_mid = self._theta + dtheta / 2.0
        self._x += ds * math.cos(theta_mid)
        self._y += ds * math.sin(theta_mid)
        self._theta = theta

        self._publish_odom(msg.header.stamp, ds / dt, dtheta / dt)
        self._broadcast_tf(msg.header.stamp)

        self._prev_odo_m = self._odo_m
        self._prev_stamp_ns = stamp_ns

        self.get_logger().info(
            f"x={self._x:+7.3f}m  y={self._y:+7.3f}m  theta={math.degrees(self._theta):+7.2f}deg",
            throttle_duration_sec=1.0,
        )

    def _yaw_quaternion(self):
        """Yaw-only quaternion (z, w) for the current heading; x and y stay zero in 2D."""
        return math.sin(self._theta / 2.0), math.cos(self._theta / 2.0)

    def _publish_odom(self, stamp, v_forward, yaw_rate):
        """Publish the accumulated pose as nav_msgs/Odometry (covariance left zero = unknown)."""
        qz, qw = self._yaw_quaternion()

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        # Twist is in the child (base_link) frame: x is forward, z is yaw.
        odom.twist.twist.linear.x = v_forward
        odom.twist.twist.angular.z = yaw_rate

        self.pub_odom.publish(odom)

    def _broadcast_tf(self, stamp):
        """Broadcast the same pose as the odom -> base_link transform on /tf."""
        qz, qw = self._yaw_quaternion()

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame

        t.transform.translation.x = self._x
        t.transform.translation.y = self._y
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    """Entry point: spin the pose estimator until interrupted."""
    rclpy.init(args=args)
    node = PoseEstimator()
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
