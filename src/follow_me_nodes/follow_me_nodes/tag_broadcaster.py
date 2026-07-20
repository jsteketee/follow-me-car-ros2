#!/usr/bin/env python3
"""Tag TF broadcaster: UWB bearing + distance (uwb/raw) -> uwb_link -> tag_link on /tf.
Projects an anchor-frame fix (identity rotation); skips no-fix frames (distance <= 0).
Frame parenting, lever-arm, and pan-servo rationale: see NOTES.md.
"""

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

from follow_me_interfaces.msg import UwbRaw

# The fix stream this node projects — RELATIVE name, namespaced per robot at launch.
TOPIC_UWB_RAW = "uwb/raw"


class TagBroadcaster(Node):
    """Broadcasts the tag's position as an uwb_link -> tag_link transform on /tf."""

    def __init__(self):
        """Set up frame parameters, the TF broadcaster, and the uwb/raw subscription."""
        super().__init__("tag_broadcaster")

        # Global TF frame ids; set per robot via parameters (see serial_bridge / NOTES.md).
        self.anchor_frame = self.declare_parameter("anchor_frame", "uwb_link").value
        self.tag_frame = self.declare_parameter("tag_frame", "tag_link").value

        self.tf_broadcaster = TransformBroadcaster(self)
        self.sub = self.create_subscription(UwbRaw, TOPIC_UWB_RAW, self._on_tag_pose, 10)

        self.get_logger().info(
            f"tag_broadcaster up; '{TOPIC_UWB_RAW}' -> "
            f"{self.anchor_frame} -> {self.tag_frame} on /tf"
        )

    def _on_tag_pose(self, msg):
        """Project one bearing/distance fix into an anchor-frame point and broadcast it."""
        if msg.distance <= 0.0:
            # No fix this frame (ESP32 sends distance -1); don't plant a phantom tag.
            return

        bearing = msg.bearing

        t = TransformStamped()
        t.header.stamp = msg.header.stamp  # already ESP32 device time mapped to ROS clock
        t.header.frame_id = self.anchor_frame
        t.child_frame_id = self.tag_frame

        t.transform.translation.x = msg.distance * math.cos(bearing)
        t.transform.translation.y = msg.distance * math.sin(bearing)
        # Rotation stays identity: a ranged point has position but no orientation.
        t.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    """Entry point: spin the tag broadcaster until interrupted."""
    rclpy.init(args=args)
    node = TagBroadcaster()
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
