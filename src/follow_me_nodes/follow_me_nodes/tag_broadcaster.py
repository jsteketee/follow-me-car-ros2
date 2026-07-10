#!/usr/bin/env python3
"""Tag TF broadcaster: FusedTagPose (bearing + distance) -> uwb_link -> tag_link on /tf.

Turns the ESP32's car-relative fix on `tag/pose` into a moving TF frame for the tag.
The bearing + distance are measured by the DW3000 anchor, so the transform is broadcast
from `uwb_link` (the anchor), NOT base_link: the ~0.17 m lever arm is small next to the
<10 cm ranging error but not zero, and parenting under uwb_link keeps the geometry honest.

Parenting under uwb_link also gets the tag's absolute position for free: tf2 chains
odom -> base_link -> uwb_link -> tag_link live, so a viewer set to the `odom` frame shows
the tag at its true world spot with no dead-reckoning drift baked into a stored edge.

The anchor faces forward with rpy=0 relative to base_link (see the URDF), so the
car-relative bearing needs no rotation offset:
    x = distance * cos(angle)   # angle 0 = straight ahead
    y = distance * sin(angle)   # angle +ve = left (REP-103, CCW)

Subscribes:
  tag/pose   follow_me_interfaces/FusedTagPose   (SI: angle rad, distance m)
Broadcasts:
  uwb_link -> tag_link   on /tf, per fix (~10 Hz), identity rotation (a point has none)

No fix (distance <= 0; the ESP32 sends -1 when the tag is not ranged) is skipped rather
than planting a phantom tag at or behind the origin.
"""

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

from follow_me_interfaces.msg import FusedTagPose

TOPIC_TAG_POSE = "tag/pose"


class TagBroadcaster(Node):
    """Broadcasts the tag's position as an uwb_link -> tag_link transform on /tf."""

    def __init__(self):
        super().__init__("tag_broadcaster")

        # Frame ids live in the global TF tree, so namespacing does not prefix them;
        # set them per robot via these parameters at launch, same as serial_bridge.
        self.anchor_frame = self.declare_parameter("anchor_frame", "uwb_link").value
        self.tag_frame = self.declare_parameter("tag_frame", "tag_link").value

        self.tf_broadcaster = TransformBroadcaster(self)
        self.sub = self.create_subscription(
            FusedTagPose, TOPIC_TAG_POSE, self._on_tag_pose, 10
        )

        self.get_logger().info(
            f"tag_broadcaster up; '{TOPIC_TAG_POSE}' -> "
            f"{self.anchor_frame} -> {self.tag_frame} on /tf"
        )

    def _on_tag_pose(self, msg):
        """Project one bearing/distance fix into an anchor-frame point and broadcast it."""
        if msg.distance <= 0.0:
            # No fix this frame (ESP32 sends distance -1); don't plant a phantom tag.
            return

        t = TransformStamped()
        t.header.stamp = msg.header.stamp  # already ESP32 device time mapped to ROS clock
        t.header.frame_id = self.anchor_frame
        t.child_frame_id = self.tag_frame

        t.transform.translation.x = msg.distance * math.cos(msg.angle)
        t.transform.translation.y = msg.distance * math.sin(msg.angle)
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
