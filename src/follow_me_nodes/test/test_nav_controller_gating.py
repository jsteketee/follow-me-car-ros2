#!/usr/bin/env python3
"""Unit tests for nav_controller's nav_mode gate: idle until "follow", safe-stop and
state reset on exit. Callbacks are invoked directly with recording publishers.
"""

import pytest
import rclpy

from nav_msgs.msg import Odometry
from follow_me_interfaces.msg import NavMode, TagEstimate
from follow_me_nodes.nav_controller import NavController


class FakePublisher:
    """Recording stand-in for the cmd_drive publisher."""

    def __init__(self):
        self.msgs = []

    def publish(self, msg):
        """Record one published message."""
        self.msgs.append(msg)


class FakeTfBroadcaster:
    """Recording stand-in for the nav_goal TF broadcaster."""

    def __init__(self):
        self.transforms = []

    def sendTransform(self, t):
        """Record one broadcast transform."""
        self.transforms.append(t)


@pytest.fixture
def node():
    """Construct a NavController with recording cmd_drive + TF outputs."""
    rclpy.init()
    n = NavController()
    n.pub = FakePublisher()
    n.tf_broadcaster = FakeTfBroadcaster()
    try:
        yield n
    finally:
        n.destroy_node()
        rclpy.shutdown()


def nav_mode(mode):
    """Build a NavMode message."""
    m = NavMode()
    m.mode = mode
    return m


def good_tag(distance=5.0, bearing_abs=0.0):
    """Build a trusted TagEstimate (fresh, low sigma, not coasting)."""
    t = TagEstimate()
    t.distance = float(distance)
    t.bearing_abs = float(bearing_abs)
    t.bearing_sigma = 0.05
    t.age_ms = 10
    t.coasting = False
    return t


def test_idle_until_mode_arrives(node):
    """Before any nav_mode message, tag fixes produce no commands and no goal."""
    node._on_odom(Odometry())
    node._on_tag_pose(good_tag())
    assert node.pub.msgs == []
    assert node._goal is None


def test_drives_when_follow_active(node):
    """Once nav_mode == "follow", a trusted fix latches a goal and commands cruise."""
    node._on_odom(Odometry())
    node._on_nav_mode(nav_mode("follow"))
    node._on_tag_pose(good_tag(distance=5.0))
    assert node._goal is not None
    assert len(node.pub.msgs) == 1
    assert node.pub.msgs[0].speed == pytest.approx(node.cruise_speed)
    assert len(node.tf_broadcaster.transforms) == 1


def test_deactivation_safe_stops_and_resets(node):
    """Leaving follow emits exactly one zero-speed command and clears the follow state."""
    node._on_odom(Odometry())
    node._on_nav_mode(nav_mode("follow"))
    node._on_tag_pose(good_tag())
    node._on_nav_mode(nav_mode("stopped"))
    assert len(node.pub.msgs) == 2                       # cruise cmd + the safe stop
    assert node.pub.msgs[-1].speed == 0.0
    assert node._goal is None and node._holding is False and node._good_streak == 0
    node._on_tag_pose(good_tag())                        # further fixes: cmd-silent
    assert len(node.pub.msgs) == 2


def test_other_mode_never_activates(node):
    """A nav_mode this node doesn't implement keeps it idle (no stop cmd either)."""
    node._on_odom(Odometry())
    node._on_nav_mode(nav_mode("waypoint"))
    node._on_tag_pose(good_tag())
    assert node.pub.msgs == []
