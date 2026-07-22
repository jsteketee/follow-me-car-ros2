#!/usr/bin/env python3
"""mode_manager: owns the Pi-side nav_mode — publishes it latched and hosts set_nav_mode.
Controller nodes (nav_controller, future policies) subscribe and act only when the mode
they implement is active; mode entry is gated here so bad transitions are refused."""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from follow_me_interfaces.msg import NavMode
from follow_me_interfaces.srv import SetNavMode

# Topic and service — RELATIVE names, namespaced per robot at launch.
TOPIC_NAV_MODE = "nav_mode"
SRV_SET_NAV_MODE = "set_nav_mode"

DEFAULT_INITIAL_MODE = "follow"           # the stack boots straight into follow
DEFAULT_ALLOWED_MODES = ["follow", "stopped"]   # grows with new policies ("waypoint", ...)

# Latched: late joiners (controllers, dashboard) get the current mode on subscribe.
# Subscribers must also request TRANSIENT_LOCAL durability or they miss the latch.
LATCHED_QOS = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)


class ModeManager(Node):
    """Single authority for nav_mode: validates entry, applies, and announces latched."""

    def __init__(self):
        """Declare mode params, create the latched publisher + set_nav_mode service."""
        super().__init__("mode_manager")

        self.allowed_modes = list(
            self.declare_parameter("allowed_modes", DEFAULT_ALLOWED_MODES).value)
        initial_mode = self.declare_parameter("initial_mode", DEFAULT_INITIAL_MODE).value

        self._mode = None
        self.pub = self.create_publisher(NavMode, TOPIC_NAV_MODE, LATCHED_QOS)
        self.srv = self.create_service(SetNavMode, SRV_SET_NAV_MODE, self._on_set_nav_mode)

        self._apply(initial_mode)
        self.get_logger().info(
            f"mode_manager up; nav_mode '{self._mode}' latched on '{TOPIC_NAV_MODE}', "
            f"set via '{SRV_SET_NAV_MODE}' (allowed: {', '.join(self.allowed_modes)})"
        )

    def _check_entry(self, mode):
        """Entry gate -> (ok, reason); per-mode car-condition checks get added here."""
        if mode not in self.allowed_modes:
            return False, f"unknown mode '{mode}' (allowed: {', '.join(self.allowed_modes)})"
        return True, ""

    def _apply(self, mode):
        """Set the mode and announce it on the latched topic."""
        self._mode = mode
        msg = NavMode()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.mode = mode
        self.pub.publish(msg)
        self.get_logger().info(f"nav_mode -> '{mode}'")

    def _on_set_nav_mode(self, request, response):
        """Service handler: gate the requested mode, apply on change, report the outcome."""
        ok, reason = self._check_entry(request.mode)
        response.accepted = ok
        if not ok:
            response.message = reason
            self.get_logger().warning(f"set_nav_mode('{request.mode}') rejected: {reason}")
        else:
            response.message = f"nav_mode -> '{request.mode}'"
            if request.mode != self._mode:
                self._apply(request.mode)
        return response


def main(args=None):
    """Entry point: spin the mode manager until interrupted."""
    rclpy.init(args=args)
    node = ModeManager()
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
