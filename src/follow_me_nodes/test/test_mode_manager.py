#!/usr/bin/env python3
"""Unit tests for mode_manager: boot mode, latched announcements, set_nav_mode gating.
Handlers are called directly with a recording publisher; no executor spin needed.
"""

import pytest
import rclpy

from follow_me_interfaces.srv import SetNavMode
from follow_me_nodes.mode_manager import ModeManager


class FakePublisher:
    """Recording stand-in for the nav_mode publisher."""

    def __init__(self):
        self.msgs = []

    def publish(self, msg):
        """Record one published message."""
        self.msgs.append(msg)


@pytest.fixture
def node():
    """Construct a ModeManager and swap in a recording nav_mode publisher."""
    rclpy.init()
    n = ModeManager()
    n.pub = FakePublisher()
    try:
        yield n
    finally:
        n.destroy_node()
        rclpy.shutdown()


def call(node, mode):
    """Invoke the service handler directly and return the response."""
    return node._on_set_nav_mode(SetNavMode.Request(mode=mode), SetNavMode.Response())


def test_boots_into_follow(node):
    """The manager applies the initial mode at construction (default "follow")."""
    assert node._mode == "follow"


def test_accepts_allowed_mode_and_announces(node):
    """An allowed mode is accepted, applied, and published on nav_mode."""
    resp = call(node, "stopped")
    assert resp.accepted is True
    assert node._mode == "stopped"
    assert [m.mode for m in node.pub.msgs] == ["stopped"]


def test_rejects_unknown_mode(node):
    """An unknown mode is refused with a reason and nothing is applied or published."""
    resp = call(node, "hyperdrive")
    assert resp.accepted is False
    assert "hyperdrive" in resp.message
    assert node._mode == "follow"
    assert node.pub.msgs == []


def test_setting_current_mode_does_not_republish(node):
    """Re-requesting the active mode is accepted but not re-announced."""
    resp = call(node, "follow")
    assert resp.accepted is True
    assert node.pub.msgs == []
