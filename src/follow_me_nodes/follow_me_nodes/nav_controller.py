#!/usr/bin/env python3
"""nav_controller: latched-goal follow-me — fused tag + odom -> cmd_drive, broadcasts nav_goal.
Commits the tag as a point in odom and steers to it; on high bearing uncertainty it HOLDs
(freezes the goal, keeps driving to the last trusted point) until a streak of good fixes.
Drives only while mode_manager's latched nav_mode matches the active_mode param."""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

from follow_me_interfaces.msg import DriveCommand, NavMode, TagEstimate

# Inputs and output — RELATIVE names, namespaced per robot at launch.
TOPIC_TAG_POSE = "fused/tag_pose"
TOPIC_ODOM = "odom"
TOPIC_CMD_DRIVE = "cmd_drive"
TOPIC_NAV_MODE = "nav_mode"

ACTIVE_MODE = "follow"     # the nav_mode this policy implements; other policies = other nodes

# Must match mode_manager's latched publisher QoS or the boot-time mode is missed.
LATCHED_QOS = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)

# --- Follow tuning (all exposed as ROS params; these are the defaults) ---
MPH_TO_MPS = 0.44704       # exact by definition; the stack is SI downstream of the bridge
FOLLOW_DISTANCE_M = 2.0    # hunt the tag when it is beyond this range; hold position inside it
# On/off follow speed. 3.0 mph is ABOVE the ESP32's default 2.5 mph cap — raise maxSpeedMph on
# the dashboard or the firmware rejects the frames (cmd_rejects climbs, car won't move).
CRUISE_SPEED_MPS = 3.0 * MPH_TO_MPS   # 1.34 m/s = 3.0 mph
# Trust gate on the fused bearing 1-sigma: enter HOLD above _HIGH, re-acquire below _LOW
# (hysteresis). In degrees to match the estimator's logs; measured normal is 5-10 deg, >10 bad.
BEARING_SIGMA_HIGH_DEG = 10.0
BEARING_SIGMA_LOW_DEG = 7.0
REACQUIRE_COUNT = 5        # consecutive trusted estimates in HOLD before re-latching the goal


def yaw_from_quaternion(x, y, z, w):
    """Extract the yaw (rotation about z, radians) from a quaternion."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class NavController(Node):
    """Latched-goal follow-me: commit the tag in odom, pursue it, hold through bad bearings."""

    def __init__(self):
        """Declare follow/trust params and wire the tag + odom -> cmd_drive / nav_goal paths."""
        super().__init__("nav_controller")

        self.odom_frame = self.declare_parameter("odom_frame", "odom").value
        self.goal_frame = self.declare_parameter("goal_frame", "nav_goal").value
        self.follow_distance = self.declare_parameter(
            "follow_distance_m", FOLLOW_DISTANCE_M).value
        self.cruise_speed = self.declare_parameter(
            "cruise_speed_mps", CRUISE_SPEED_MPS).value
        self.sigma_high = math.radians(
            self.declare_parameter("bearing_sigma_high_deg", BEARING_SIGMA_HIGH_DEG).value)
        self.sigma_low = math.radians(
            self.declare_parameter("bearing_sigma_low_deg", BEARING_SIGMA_LOW_DEG).value)
        self.reacquire_count = self.declare_parameter(
            "reacquire_count", REACQUIRE_COUNT).value
        self.active_mode = self.declare_parameter("active_mode", ACTIVE_MODE).value

        # Follow state.
        self._active = False    # True only while nav_mode == active_mode; no driving until then
        self._car = None        # (x, y, yaw) in odom, latest — from /odom
        self._goal = None       # (x, y) committed goal in odom; None until the first trusted fix
        self._holding = False   # True = HOLD: bearing uncertainty too high, goal frozen
        self._good_streak = 0   # consecutive trusted estimates while holding (re-acquire counter)

        self.pub = self.create_publisher(DriveCommand, TOPIC_CMD_DRIVE, 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.sub_tag = self.create_subscription(
            TagEstimate, TOPIC_TAG_POSE, self._on_tag_pose, 10)
        self.sub_odom = self.create_subscription(Odometry, TOPIC_ODOM, self._on_odom, 10)
        self.sub_mode = self.create_subscription(
            NavMode, TOPIC_NAV_MODE, self._on_nav_mode, LATCHED_QOS)

        self.get_logger().info(
            f"nav_controller up; latched follow '{TOPIC_TAG_POSE}' + '{TOPIC_ODOM}' -> "
            f"'{TOPIC_CMD_DRIVE}' (standoff {self.follow_distance:.2f} m, "
            f"cruise {self.cruise_speed:.2f} m/s, HOLD > {math.degrees(self.sigma_high):.0f} deg; "
            f"idle until nav_mode == '{self.active_mode}')"
        )

    def _on_nav_mode(self, msg):
        """Gate driving on the shared nav_mode; safe-stop and reset state on deactivation."""
        was_active = self._active
        self._active = (msg.mode == self.active_mode)
        if was_active and not self._active:
            # One-shot commanded stop at the current heading, then go cmd-silent: the
            # bridge's 500 ms staleness gate stops TX and the ESP32's failsafe cuts
            # throttle as backstops. Clear the goal so re-entry starts in acquisition.
            if self._car is not None:
                self._publish(0.0, self._car[2])
            self._goal = None
            self._holding = False
            self._good_streak = 0
            self.get_logger().info(
                f"nav_mode '{msg.mode}' != '{self.active_mode}': follow disabled")
        elif not was_active and self._active:
            self.get_logger().info(f"nav_mode '{msg.mode}': follow enabled (acquisition)")

    def _on_odom(self, msg):
        """Cache the latest car pose in odom (x, y, yaw) for goal placement and pursuit."""
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self._car = (p.x, p.y, yaw_from_quaternion(q.x, q.y, q.z, q.w))

    def _on_tag_pose(self, msg):
        """Update the committed goal under the trust gate, then drive toward it."""
        if not self._active:
            return  # another nav_mode owns the car; stay cmd-silent
        if self._car is None:
            return  # no odom yet; can't place the goal or pursue (bridge failsafe covers it)

        trusted = msg.age_ms >= 0 and not msg.coasting

        if self._goal is None:
            # Acquisition: wait for a confident fix to establish the first goal.
            if trusted and msg.bearing_sigma < self.sigma_high:
                self._goal = self._tag_in_odom(msg)
                self._holding = False
        elif not self._holding:
            # TRACKING: keep latching the live tag while the bearing stays trustworthy.
            if trusted and msg.bearing_sigma < self.sigma_high:
                self._goal = self._tag_in_odom(msg)
            else:
                self._holding = True
                self._good_streak = 0
        else:
            # HOLD: goal frozen; re-acquire only after a streak of confident fixes — a clamp
            # can look low-variance for one sample, while a live reading jitters.
            if trusted and msg.bearing_sigma < self.sigma_low:
                self._good_streak += 1
                if self._good_streak >= self.reacquire_count:
                    self._goal = self._tag_in_odom(msg)
                    self._holding = False
                    self._good_streak = 0
            else:
                self._good_streak = 0

        self._drive_and_broadcast()

    def _tag_in_odom(self, msg):
        """Place the fused tag as an absolute (x, y) point in odom from the current car pose."""
        cx, cy, _ = self._car
        return (cx + msg.distance * math.cos(msg.bearing_abs),
                cy + msg.distance * math.sin(msg.bearing_abs))

    def _drive_and_broadcast(self):
        """Steer toward the committed goal (cruise until inside the standoff), broadcast nav_goal."""
        cx, cy, yaw = self._car
        if self._goal is None:
            self._publish(0.0, yaw)   # no goal yet: hold position and current heading
            return

        gx, gy = self._goal
        dx, dy = gx - cx, gy - cy
        heading = math.atan2(dy, dx)
        speed = self.cruise_speed if math.hypot(dx, dy) > self.follow_distance else 0.0
        self._publish(speed, heading)
        self._broadcast_goal()

    def _publish(self, speed, heading):
        """Publish a freshly stamped DriveCommand (fresh stamp keeps the bridge TX alive)."""
        cmd = DriveCommand()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = self.odom_frame
        cmd.speed = float(speed)
        cmd.heading = float(heading)
        self.pub.publish(cmd)

    def _broadcast_goal(self):
        """Broadcast the committed goal as odom -> nav_goal so the dashboard can render it."""
        gx, gy = self._goal
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.goal_frame
        t.transform.translation.x = gx
        t.transform.translation.y = gy
        t.transform.rotation.w = 1.0   # a point has position but no orientation
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    """Entry point: spin the nav controller until interrupted."""
    rclpy.init(args=args)
    node = NavController()
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
