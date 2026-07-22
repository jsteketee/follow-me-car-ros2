#!/usr/bin/env python3
"""Unit tests for the tag_estimator EKF. No hardware; callbacks and the publish tick
are called directly with hand-built messages, publisher stubbed (see NOTES.md).
"""

import math

import pytest
import rclpy

from nav_msgs.msg import Odometry
from follow_me_interfaces.msg import ActuatorStatus, UwbRaw
from follow_me_nodes.tag_estimator import (
    TagEstimator,
    RANGE_SIGMA_M,
    BEARING_SIGMA_RAD,
    REINIT_AFTER_REJECTS,
    COAST_AFTER_MS,
    LATENCY_N_MIN,
    LATENCY_GAIN,
    LATENCY_OMEGA_MIN,
)


class FakePublisher:
    """Recording stand-in for the fused-pose publisher."""

    def __init__(self):
        self.msgs = []  # every published TagEstimate, in order

    def publish(self, msg):
        """Record one published message."""
        self.msgs.append(msg)


# --- message builders -------------------------------------------------------

def stamp_of(msg_header, ns):
    """Write a nanosecond timestamp into a std_msgs Header."""
    msg_header.stamp.sec = int(ns // 1_000_000_000)
    msg_header.stamp.nanosec = int(ns % 1_000_000_000)


def make_odom(x, y, yaw, stamp_ns=None, omega=0.0):
    """Build an Odometry with a 2D pose, yaw-only orientation, and yaw rate; stamp if given."""
    o = Odometry()
    if stamp_ns is not None:
        stamp_of(o.header, stamp_ns)
    o.pose.pose.position.x = float(x)
    o.pose.pose.position.y = float(y)
    o.pose.pose.orientation.z = math.sin(yaw / 2.0)
    o.pose.pose.orientation.w = math.cos(yaw / 2.0)
    o.twist.twist.angular.z = float(omega)
    return o


def make_act(wire_pan):
    """Build an ActuatorStatus carrying a wire-sign (+right) pan angle in radians."""
    a = ActuatorStatus()
    a.pan_angle = float(wire_pan)
    return a


def make_fix(distance, bearing, age_ms, stamp_ns):
    """Build a UwbRaw fix (REP-103 bearing) with the given stamp and age."""
    u = UwbRaw()
    stamp_of(u.header, stamp_ns)
    u.distance = float(distance)
    u.bearing = float(bearing)
    u.age_ms = int(age_ms)
    return u


def prime(n, x=0.0, y=0.0, yaw=0.0, wire_pan=0.0):
    """Cache a car pose and pan angle so the measurement model can be formed."""
    n._on_odom(make_odom(x, y, yaw))
    n._on_actuator_status(make_act(wire_pan))


def now_ns(n):
    """Current node-clock time in nanoseconds."""
    return n.get_clock().now().nanoseconds


# --- fixture ----------------------------------------------------------------

@pytest.fixture
def node():
    """Construct a TagEstimator with a recording publisher; tear it down after the test."""
    rclpy.init()
    n = TagEstimator()
    fake = FakePublisher()
    n.pub_fused = fake
    try:
        yield n, fake
    finally:
        n.destroy_node()
        rclpy.shutdown()


# --- tests ------------------------------------------------------------------

def test_ignores_no_fix_and_missing_inputs(node):
    """(1) No-fix frames and fixes arriving before odom/pan leave the filter down; no output."""
    n, fake = node
    t = now_ns(n)

    n._on_uwb_raw(make_fix(2.0, 0.0, 0, t))        # valid fix, but no odom/pan cached yet
    assert n._tag is None

    prime(n)
    n._on_uwb_raw(make_fix(-1.0, 0.0, 0, t))       # no-range sentinel
    n._on_uwb_raw(make_fix(2.0, 0.0, -1, t))       # no-fix age sentinel
    assert n._tag is None

    n._on_publish_tick()
    assert fake.msgs == []                          # uninitialized -> silence


def test_init_from_first_fix(node):
    """(2) First fix initializes state at car + range, with polar noise mapped into P."""
    n, fake = node
    prime(n)                                        # car at origin, yaw 0, pan 0
    n._on_uwb_raw(make_fix(2.0, 0.0, 0, now_ns(n)))

    tx, ty = n._tag
    assert tx == pytest.approx(2.0, abs=1e-6)
    assert ty == pytest.approx(0.0, abs=1e-6)
    # Along-ray variance is the range noise; cross-range is (d * bearing noise)^2.
    assert n._pxx == pytest.approx(RANGE_SIGMA_M**2, abs=1e-9)
    assert n._pyy == pytest.approx((2.0 * BEARING_SIGMA_RAD) ** 2, abs=1e-9)
    assert n._pxy == pytest.approx(0.0, abs=1e-9)

    n._on_publish_tick()
    est = fake.msgs[-1]
    assert est.distance == pytest.approx(2.0, abs=1e-3)
    assert est.bearing_abs == pytest.approx(0.0, abs=1e-3)
    assert est.coasting is False
    # Sigmas start at the init values and only grow (a moment of predict may have run).
    assert RANGE_SIGMA_M <= est.range_sigma < 0.3
    assert BEARING_SIGMA_RAD <= est.bearing_sigma < 0.4


def test_pan_sign_correctness(node):
    """(3) Wire pan +0.5 (right) with a boresight fix lands the tag at bearing_abs -0.5."""
    n, fake = node
    prime(n, wire_pan=0.5)                          # anchor panned RIGHT by 0.5 rad
    n._on_uwb_raw(make_fix(2.0, 0.0, 0, now_ns(n)))

    n._on_publish_tick()
    assert fake.msgs[-1].bearing_abs == pytest.approx(-0.5, abs=1e-3)


def test_absolute_vs_relative_consistency(node):
    """(4) bearing_rel is bearing_abs minus the car yaw, wrapped."""
    n, fake = node
    yaw = 1.0
    prime(n, yaw=yaw)
    n._on_uwb_raw(make_fix(2.0, 0.3, 0, now_ns(n)))

    n._on_publish_tick()
    est = fake.msgs[-1]
    expected_rel = math.atan2(math.sin(est.bearing_abs - yaw), math.cos(est.bearing_abs - yaw))
    assert est.bearing_rel == pytest.approx(expected_rel, abs=1e-6)


def test_dedup_same_fix(node):
    """(5) The bridge's 50 Hz re-reports of one fix (same stamp - age) fuse exactly once."""
    n, _ = node
    prime(n)
    t = now_ns(n)
    n._on_uwb_raw(make_fix(2.0, 0.0, 0, t))
    p_after_first = (n._pxx, n._pxy, n._pyy)
    tag_after_first = n._tag

    # Same fix re-reported on later telemetry frames: stamp and age grow in lockstep.
    n._on_uwb_raw(make_fix(2.0, 0.0, 20, t + 20_000_000))
    n._on_uwb_raw(make_fix(2.0, 0.0, 40, t + 40_000_000))

    assert (n._pxx, n._pxy, n._pyy) == p_after_first   # covariance did not collapse
    assert n._tag == tag_after_first


def test_gating_rejects_outlier(node):
    """(6) A wildly inconsistent fix is gated out: state untouched, streak counted."""
    n, _ = node
    prime(n)
    t = now_ns(n)
    n._on_uwb_raw(make_fix(2.0, 0.0, 0, t))
    tag_before = n._tag

    n._on_uwb_raw(make_fix(5.0, 0.0, 0, t + 10_000_000))  # 3 m range jump in 10 ms
    assert n._tag == tag_before
    assert n._reject_streak == 1


def test_reinit_after_reject_streak(node):
    """(7) A sustained streak of consistent-but-far fixes snaps the filter to the new spot."""
    n, _ = node
    prime(n)
    t = now_ns(n)
    n._on_uwb_raw(make_fix(2.0, 0.0, 0, t))

    # Closely spaced fixes keep P (and so the gate) tight, forcing rejects until the
    # streak trips the reinit escape.
    for i in range(1, REINIT_AFTER_REJECTS + 1):
        n._on_uwb_raw(make_fix(5.0, 0.0, 0, t + i * 1_000_000))

    tx, ty = n._tag
    assert tx == pytest.approx(5.0, abs=1e-6)       # snapped to the data
    assert ty == pytest.approx(0.0, abs=1e-6)
    assert n._reject_streak == 0


def test_pairs_fix_with_measurement_time_yaw(node):
    """(8b) A fix measured at an old yaw, processed after the car rotated, uses the OLD yaw."""
    n, _ = node
    n._on_actuator_status(make_act(0.0))
    t_meas = now_ns(n)
    n._on_odom(make_odom(0.0, 0.0, 0.0, stamp_ns=t_meas))            # yaw 0 when measured
    t_proc = t_meas + 100_000_000                                    # 100 ms later
    n._on_odom(make_odom(0.0, 0.0, math.pi / 2.0, stamp_ns=t_proc))  # car has spun to +90 deg

    # Fix stamped now (t_proc) but measured 100 ms ago -> fix_ns == t_meas (yaw 0).
    n._on_uwb_raw(make_fix(2.0, 0.0, 100, t_proc))

    tx, ty = n._tag
    assert tx == pytest.approx(2.0, abs=1e-6)   # paired with yaw 0 -> tag at +x
    assert ty == pytest.approx(0.0, abs=1e-6)   # NOT (0, 2), which the latest yaw would give


def test_rotation_dead_reckoning(node):
    """(8) Car yaw +90 deg with no new fix: bearing_rel swings, bearing_abs holds."""
    n, fake = node
    prime(n)
    n._on_uwb_raw(make_fix(2.0, 0.0, 0, now_ns(n)))  # tag dead ahead in odom (+x)

    n._on_odom(make_odom(0.0, 0.0, math.pi / 2.0))   # car spins in place, tag static
    n._on_publish_tick()
    est = fake.msgs[-1]

    assert est.bearing_abs == pytest.approx(0.0, abs=1e-6)               # tag didn't move
    assert est.bearing_rel == pytest.approx(-math.pi / 2.0, abs=1e-6)    # car did


def test_broadcasts_filtered_tag_tf(node):
    """(10) Each publish tick broadcasts the filtered tag as base_link -> tag_est_link."""
    n, _ = node
    sent = []
    n.tf_broadcaster = type("FakeTf", (), {"sendTransform": lambda self, t: sent.append(t)})()

    prime(n, yaw=math.pi / 2.0)                     # car facing +y; tag 2 m dead ahead
    n._on_uwb_raw(make_fix(2.0, 0.0, 0, now_ns(n)))
    n._on_publish_tick()

    assert len(sent) == 1
    t = sent[-1]
    assert t.header.frame_id == "base_link"
    assert t.child_frame_id == "tag_est_link"
    # Car-frame coordinates: straight ahead regardless of the car's world yaw.
    assert t.transform.translation.x == pytest.approx(2.0, abs=1e-5)
    assert t.transform.translation.y == pytest.approx(0.0, abs=1e-5)


def test_age_never_negative(node):
    """(11) A fix stamped ahead of the node clock (device-time skew) clamps age_ms to 0."""
    n, fake = node
    prime(n)
    future = now_ns(n) + 50_000_000                  # 50 ms ahead of the wall clock
    n._on_uwb_raw(make_fix(2.0, 0.0, 0, future))

    n._on_publish_tick()
    est = fake.msgs[-1]
    assert est.age_ms == 0
    assert est.coasting is False


def test_coasting_flag_and_covariance_growth(node):
    """(9) With the last fix well in the past, the output coasts and the sigmas grow."""
    n, fake = node
    prime(n)
    past = now_ns(n) - (COAST_AFTER_MS + 200) * 1_000_000
    n._on_uwb_raw(make_fix(2.0, 0.0, 0, past))       # fresh fix, measured long ago

    n._on_publish_tick()
    est = fake.msgs[-1]
    assert est.coasting is True
    assert est.age_ms > COAST_AFTER_MS
    assert est.range_sigma > RANGE_SIGMA_M           # predict grew P past the init values
    assert est.bearing_sigma > BEARING_SIGMA_RAD


def test_latency_regression_recovers_injected_slope(node):
    """(12) A rotating window with nu_b = omega*delay -> latency converges to gain*delay."""
    n, _ = node
    delay = 0.040          # s of true UWB<->IMU mispairing
    omega = 2.0            # rad/s, above the leverage threshold
    for _ in range(LATENCY_N_MIN):
        n._accumulate_latency(omega, omega * delay)

    assert n._lat_confident is True
    # First window from latency_used=0: window residual == delay, applied at LATENCY_GAIN.
    assert n._latency_ns / 1e9 == pytest.approx(LATENCY_GAIN * delay, abs=1e-4)


def test_latency_sign_and_slow_rotation(node):
    """(13) A negative slope drives latency negative; sub-threshold yaw rate is ignored."""
    n, _ = node
    for _ in range(LATENCY_N_MIN):                    # opposite sign -> negative latency
        n._accumulate_latency(2.0, 2.0 * -0.030)
    assert n._latency_ns < 0

    n2_before = n._latency_ns
    slow = LATENCY_OMEGA_MIN * 0.5                    # below the leverage threshold
    for _ in range(LATENCY_N_MIN * 3):
        n._accumulate_latency(slow, slow * 0.040)
    assert n._latency_ns == n2_before                # no rotational leverage -> no change
