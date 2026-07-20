#!/usr/bin/env python3
"""Tag position estimator: EKF fusing uwb/raw + pan angle + odom -> fused/tag_pose.

State is the tag's (x, y) in the `odom` frame, so a static tag is a constant state:
the predict step is pure process-noise growth (random-walk tag motion), and the car's
rotation/translation and the pan angle live in the measurement model. The anchor is
approximated AT base_link: the ~9 cm lever arm is inside the ~10 cm ranging noise and
deliberately ignored (tag_broadcaster's raw TF path still composes it exactly). Innovation gating rejects bad UWB fixes; a reject streak triggers
re-initialization so a tag that genuinely moved is not rejected forever.

Each fix is paired with the car yaw at its MEASUREMENT time (stamp - age - latency). The
residual UWB<->IMU latency is learned online from rotation (bearing innovation vs yaw
rate), converges, then holds; it is re-estimated each run (in-memory only).

Subscribes:
  uwb/raw          follow_me_interfaces/UwbRaw    raw range+bearing fixes (re-reported
                                                  at telemetry rate; deduped by fix time)
  actuator/status  follow_me_interfaces/ActuatorStatus  measured pan angle (wire sign)
  odom             nav_msgs/Odometry              car pose in `odom` (pose_estimator)

Publishes:
  fused/tag_pose   follow_me_interfaces/TagEstimate  20 Hz filtered range/bearings/sigmas
  display/flag     follow_me_interfaces/DisplayFlag  "Cal" raised while calibrating latency,
                                                     cleared (-1) once it converges

Broadcasts:
  base_link -> tag_est_link on /tf, per publish tick — the FILTERED tag for
  visualization, alongside tag_broadcaster's raw uwb_link -> tag_link (the
  unfiltered view). Both hang off the car; the estimate itself lives in `odom`
  and is rotated into base_link (range + bearing_rel) at each tick.
"""

import math
from collections import deque

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from follow_me_interfaces.msg import ActuatorStatus, DisplayFlag, TagEstimate, UwbRaw

TOPIC_UWB_RAW = "uwb/raw"
TOPIC_ACTUATOR_STATUS = "actuator/status"
TOPIC_ODOM = "odom"
TOPIC_FUSED_TAG_POSE = "fused/tag_pose"
TOPIC_DISPLAY_FLAG = "display/flag"
CAL_FLAG_TEXT = "Cal"        # display flag raised while latency calibration is in progress

# --- Tuning ---
RANGE_SIGMA_M = 0.10         # 1-sigma DW3000 ranging error (URDF notes "<10 cm")
BEARING_SIGMA_RAD = 0.26     # 1-sigma AoA error at boresight (~15 deg), measured from bags
BEARING_SIGMA_ANGLE_K = 1.0  # per rad of |bearing|: inflate AoA noise off-boresight (PDoA
                             #   is nonlinear/unreliable away from straight ahead) — at 90
                             #   deg the effective sigma is ~2.6x the boresight value
Q_RATE_M2_PER_S = 0.25       # random-walk diffusion (m^2/s); LOWER = more smoothing / more
                             #   lag on a moving tag. Sets the Q/R ratio, i.e. the gain.
GATE_CHI2_2DOF = 9.21        # 99% chi-square gate, 2 dof — loose, so it won't fight Q
REINIT_AFTER_REJECTS = 10    # ~1 s of consecutive rejects at 10 Hz -> snap to the data
COAST_AFTER_MS = 500         # ~5 missed fixes before the estimate is flagged as coasting
MAX_FIX_AGE_MS = 250         # fixes measured longer ago than this are ignored outright
PUBLISH_PERIOD_S = 0.05      # 20 Hz output, matching the bridge's command TX cadence
MIN_RANGE_M = 0.05           # degenerate-geometry guard: H blows up as range -> 0
CAR_HIST_LEN = 64            # cached odom poses (~1.3 s at 50 Hz) for fix-time yaw lookup

# --- Online UWB<->IMU latency calibration ---
# A residual latency between the UWB fix and the IMU yaw (not captured by age_ms) drags
# the estimate while rotating, biasing the bearing innovation by omega * latency. We
# regress that innovation on yaw rate to recover the latency, converge, then hold.
LATENCY_OMEGA_MIN = 0.5      # rad/s: only learn while genuinely rotating (else no leverage)
LATENCY_SWW_MIN = 20.0       # Sum(omega^2) excitation before a window is trusted
LATENCY_N_MIN = 30           # min rotating samples in a window before it is trusted
LATENCY_GAIN = 0.5           # fraction of each window's residual applied (damps noise)
LATENCY_MAX_MS = 200         # clamp: keep the lookup inside the CAR_HIST window, reject junk


def yaw_from_quaternion(x, y, z, w):
    """Extract the yaw (rotation about z, radians) from a quaternion."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(a):
    """Wrap an angle to (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


class TagEstimator(Node):
    """EKF over the tag's (x, y) in `odom`, measured through the panned UWB anchor."""

    def __init__(self):
        super().__init__("tag_estimator")

        self.odom_frame = self.declare_parameter("odom_frame", "odom").value
        self.base_frame = self.declare_parameter("base_frame", "base_link").value
        # Frame for the FILTERED tag on /tf; tag_broadcaster's tag_link stays the raw view.
        self.tag_est_frame = self.declare_parameter("tag_est_frame", "tag_est_link").value

        # Latest cached pan; a fix is paired with the CAR POSE AT ITS MEASUREMENT TIME,
        # not the latest, so car rotation between measurement and processing doesn't drag
        # the estimate (the UWB bearing lags by up to MAX_FIX_AGE_MS).
        self._pan = None       # rad, REP-103 (wire +right is negated on receipt)
        self._car = None       # (x, y, yaw) in `odom`, latest — for the publish tick
        self._car_hist = deque(maxlen=CAR_HIST_LEN)  # (t_ns, x, y, yaw, omega) fix-time lookup

        # Online UWB<->IMU latency estimate, applied to the fix-time yaw lookup. Starts at 0
        # (== the bare timestamp pairing) and is learned from rotation; re-estimated each run.
        self._latency_ns = 0
        self._lat_confident = False   # True once a first window has converged (applied live)
        self._lat_sww = 0.0           # Sum(omega^2) in the current window
        self._lat_swn = 0.0           # Sum(omega * nu_b) in the current window
        self._lat_n = 0               # rotating-sample count in the current window
        self._cal_announced = False   # True once the "Cal" display flag has been raised

        # Filter state: tag position in `odom` and its 2x2 covariance (symmetric,
        # stored as pxx/pxy/pyy). None until the first accepted fix initializes it.
        self._tag = None
        self._pxx = self._pxy = self._pyy = 0.0

        self._last_fix_ns = None      # measurement time of the newest fix seen (dedup)
        self._last_accept_ns = None   # measurement time of the last ACCEPTED fix
        self._last_predict_ns = None  # filter time horizon for process-noise growth
        self._reject_streak = 0       # consecutive gated-out fixes since the last accept

        self.sub_uwb = self.create_subscription(UwbRaw, TOPIC_UWB_RAW, self._on_uwb_raw, 10)
        self.sub_act = self.create_subscription(
            ActuatorStatus, TOPIC_ACTUATOR_STATUS, self._on_actuator_status, 10
        )
        self.sub_odom = self.create_subscription(Odometry, TOPIC_ODOM, self._on_odom, 10)
        self.pub_fused = self.create_publisher(TagEstimate, TOPIC_FUSED_TAG_POSE, 10)
        # Raises/clears the "Cal" display flag over the latency-calibration lifecycle.
        self.pub_flag = self.create_publisher(DisplayFlag, TOPIC_DISPLAY_FLAG, 10)
        # Broadcasts base_link -> tag_est_link only; the raw uwb_link -> tag_link edge
        # belongs to tag_broadcaster.
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(PUBLISH_PERIOD_S, self._on_publish_tick)

        self.get_logger().info(
            f"tag_estimator up; fusing '{TOPIC_UWB_RAW}' + '{TOPIC_ACTUATOR_STATUS}' + "
            f"'{TOPIC_ODOM}' -> '{TOPIC_FUSED_TAG_POSE}'"
        )

    def _on_actuator_status(self, msg):
        """Cache the measured pan angle, negated from wire sign (+right) to REP-103 (+CCW)."""
        self._pan = -msg.pan_angle

    def _on_odom(self, msg):
        """Cache the car's (x, y, yaw) + yaw rate, both latest and in the time-stamped history."""
        p = msg.pose.pose
        yaw = yaw_from_quaternion(
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w
        )
        omega = msg.twist.twist.angular.z  # yaw rate, for the latency regression
        self._car = (p.position.x, p.position.y, yaw)
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        self._car_hist.append((stamp_ns, p.position.x, p.position.y, yaw, omega))

    def _car_at(self, t_ns):
        """Car (x, y, yaw, omega) from the history nearest to t_ns; None if empty."""
        if not self._car_hist:
            return None
        # Small buffer (~64); a linear nearest-scan is cheaper than the bisect bookkeeping.
        best = min(self._car_hist, key=lambda e: abs(e[0] - t_ns))
        return best[1], best[2], best[3], best[4]

    def _accumulate_latency(self, omega, nu_b):
        """Learn the UWB<->IMU latency: slope of bearing innovation vs yaw rate.

        A yaw mispaired by dt biases the bearing innovation by omega*dt, so the slope
        Sum(omega*nu_b)/Sum(omega^2) over a rotating window recovers the residual latency
        (tag motion is uncorrelated with omega and averages out). Converge-and-hold: apply
        a damped correction once a window carries enough rotation, then keep refining — the
        residual self-drives to zero and stops moving when the car isn't turning.
        """
        if abs(omega) < LATENCY_OMEGA_MIN:
            return  # no rotational leverage this fix
        self._lat_sww += omega * omega
        self._lat_swn += omega * nu_b
        self._lat_n += 1
        if self._lat_n < LATENCY_N_MIN or self._lat_sww < LATENCY_SWW_MIN:
            return  # window not yet informative
        residual_s = self._lat_swn / self._lat_sww  # seconds: latency_true - latency_used
        self._latency_ns += int(LATENCY_GAIN * residual_s * 1e9)
        cap = LATENCY_MAX_MS * 1_000_000
        self._latency_ns = max(-cap, min(cap, self._latency_ns))
        first_convergence = not self._lat_confident
        self._lat_confident = True
        self._lat_sww = self._lat_swn = 0.0
        self._lat_n = 0
        self.get_logger().info(
            f"UWB-IMU latency -> {self._latency_ns / 1e6:+.0f} ms "
            f"(window residual {residual_s * 1e3:+.0f} ms)"
        )
        if first_convergence:
            self._send_flag(CAL_FLAG_TEXT, -1)  # calibration done -> clear the flag

    def _send_flag(self, text, action):
        """Publish a display-flag add(+1)/remove(-1) event on display/flag."""
        f = DisplayFlag()
        f.header.stamp = self.get_clock().now().to_msg()
        f.text = text
        f.action = int(action)
        self.pub_flag.publish(f)

    def _bearing_var(self, bearing):
        """Angle-inflated bearing measurement variance: PDoA degrades off boresight."""
        sigma = BEARING_SIGMA_RAD * (1.0 + BEARING_SIGMA_ANGLE_K * abs(bearing))
        return sigma * sigma

    def _predict_to(self, t_ns):
        """Advance the filter to t_ns: state unchanged, covariance grows as a random walk."""
        dt = (t_ns - self._last_predict_ns) / 1e9
        if dt <= 0.0:
            return
        self._pxx += Q_RATE_M2_PER_S * dt
        self._pyy += Q_RATE_M2_PER_S * dt
        self._last_predict_ns = t_ns

    def _init_from_fix(self, msg, cx, cy, yaw, fix_ns):
        """(Re)initialize the state from one fix, with polar noise mapped to Cartesian P."""
        b_abs = normalize_angle(yaw + self._pan + msg.bearing)
        c, s = math.cos(b_abs), math.sin(b_abs)
        d = msg.distance
        self._tag = (cx + d * c, cy + d * s)
        # P = J diag(sr^2, sb^2) J^T with J the polar->Cartesian Jacobian at (d, b_abs).
        vr = RANGE_SIGMA_M**2
        vt = d * d * self._bearing_var(msg.bearing)  # cross-range variance at this distance
        self._pxx = c * c * vr + s * s * vt
        self._pxy = s * c * (vr - vt)
        self._pyy = s * s * vr + c * c * vt
        self._last_predict_ns = fix_ns
        self._last_accept_ns = fix_ns
        self._reject_streak = 0

    def _on_uwb_raw(self, msg):
        """Measurement path: dedup the fix stream, gate, and run one EKF update."""
        if msg.distance < 0.0 or msg.age_ms < 0:
            return  # no fix this frame
        if self._car is None or self._pan is None:
            return  # can't form the measurement model yet

        # Dedup by measurement time (stamp - age): the bridge re-reports each ~10 Hz fix
        # on every ~50 Hz telemetry frame with growing age_ms.
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        fix_ns = stamp_ns - msg.age_ms * 1_000_000
        if self._last_fix_ns is not None and fix_ns <= self._last_fix_ns:
            return
        self._last_fix_ns = fix_ns
        if msg.age_ms > MAX_FIX_AGE_MS:
            return  # first sighting is already too stale to fuse

        # Pair with the car pose WHEN THE FIX WAS MEASURED (fix_ns), minus the learned
        # UWB<->IMU latency — this is what keeps a rotating car from dragging the estimate.
        # Anchor approximated at base_link.
        car = self._car_at(fix_ns - self._latency_ns)
        if car is None:
            return  # no odom history yet
        cx, cy, yaw, omega = car

        if self._tag is None:
            self._init_from_fix(msg, cx, cy, yaw, fix_ns)
            tx, ty = self._tag
            self.get_logger().info(
                f"initialized tag at ({tx:+.2f}, {ty:+.2f}) m from first fix "
                f"(d={msg.distance:.2f} m)"
            )
            if not self._cal_announced:
                self._cal_announced = True
                self._send_flag(CAL_FLAG_TEXT, 1)  # latency calibration in progress
            return

        self._predict_to(fix_ns)

        # Measurement model: predicted range and raw (anchor-relative) bearing.
        tx, ty = self._tag
        dx, dy = tx - cx, ty - cy
        r = math.hypot(dx, dy)
        if r < MIN_RANGE_M:
            return  # tag on top of the anchor: H is degenerate, skip this update
        b_pred = normalize_angle(math.atan2(dy, dx) - yaw - self._pan)
        nu_r = msg.distance - r
        nu_b = normalize_angle(msg.bearing - b_pred)

        # Feed the latency estimator BEFORE gating: gated-out outliers are uncorrelated with
        # yaw rate, so they add noise but not bias, and this avoids a learn/gate deadlock.
        self._accumulate_latency(omega, nu_b)

        # H = d h / d (tx, ty); S = H P H^T + R, all 2x2 closed-form (numpy-free).
        h11, h12 = dx / r, dy / r
        h21, h22 = -dy / (r * r), dx / (r * r)
        hp11 = h11 * self._pxx + h12 * self._pxy
        hp12 = h11 * self._pxy + h12 * self._pyy
        hp21 = h21 * self._pxx + h22 * self._pxy
        hp22 = h21 * self._pxy + h22 * self._pyy
        s11 = hp11 * h11 + hp12 * h12 + RANGE_SIGMA_M**2
        s12 = hp11 * h21 + hp12 * h22
        s22 = hp21 * h21 + hp22 * h22 + self._bearing_var(msg.bearing)
        det = s11 * s22 - s12 * s12
        if det <= 0.0:
            return  # numerically degenerate innovation covariance; drop the update
        si11, si12, si22 = s22 / det, -s12 / det, s11 / det

        # Gate on the Mahalanobis distance of the innovation; a long reject streak means
        # the tag genuinely moved (gating deadlock) — snap to the data instead.
        d2 = nu_r * (si11 * nu_r + si12 * nu_b) + nu_b * (si12 * nu_r + si22 * nu_b)
        if d2 > GATE_CHI2_2DOF:
            self._reject_streak += 1
            if self._reject_streak >= REINIT_AFTER_REJECTS:
                self.get_logger().warn(
                    f"{self._reject_streak} consecutive rejected fixes — tag moved? "
                    "Reinitializing from the current fix."
                )
                self._init_from_fix(msg, cx, cy, yaw, fix_ns)
            return

        # Accept: K = P H^T S^-1, state += K nu, P = (I - K H) P.
        pht11, pht12 = hp11, hp21  # P H^T = (H P)^T since P is symmetric
        pht21, pht22 = hp12, hp22
        k11 = pht11 * si11 + pht12 * si12
        k12 = pht11 * si12 + pht12 * si22
        k21 = pht21 * si11 + pht22 * si12
        k22 = pht21 * si12 + pht22 * si22
        self._tag = (tx + k11 * nu_r + k12 * nu_b, ty + k21 * nu_r + k22 * nu_b)
        a11 = 1.0 - (k11 * h11 + k12 * h21)
        a12 = -(k11 * h12 + k12 * h22)
        a21 = -(k21 * h11 + k22 * h21)
        a22 = 1.0 - (k21 * h12 + k22 * h22)
        pxx = a11 * self._pxx + a12 * self._pxy
        pxy = a11 * self._pxy + a12 * self._pyy
        pyx = a21 * self._pxx + a22 * self._pxy
        pyy = a21 * self._pxy + a22 * self._pyy
        self._pxx, self._pyy = pxx, pyy
        self._pxy = 0.5 * (pxy + pyx)  # re-symmetrize against round-off drift

        self._reject_streak = 0
        self._last_accept_ns = fix_ns

    def _on_publish_tick(self):
        """Publish path: predict to now, re-derive outputs from the latest car pose."""
        if self._tag is None or self._car is None:
            return  # nothing to say until the first accepted fix

        now = self.get_clock().now()
        self._predict_to(now.nanoseconds)

        cx, cy, yaw = self._car
        tx, ty = self._tag
        dx, dy = tx - cx, ty - cy
        r = math.hypot(dx, dy)
        if r < MIN_RANGE_M:
            return  # bearing undefined with the tag on the anchor

        bearing_abs = math.atan2(dy, dx)
        # Project P onto the range ray (u) and the tangent (v) for per-component sigmas.
        ux, uy = dx / r, dy / r
        var_r = ux * ux * self._pxx + 2.0 * ux * uy * self._pxy + uy * uy * self._pyy
        var_t = uy * uy * self._pxx - 2.0 * ux * uy * self._pxy + ux * ux * self._pyy

        # Clamp at 0: fix stamps ride the bridge's device-time mapping, which can drift
        # a few ms AHEAD of the node clock (ESP32 vs Pi crystal skew), going "negative".
        age_ms = max(0, int(round((now.nanoseconds - self._last_accept_ns) / 1e6)))

        est = TagEstimate()
        est.header.stamp = now.to_msg()
        est.header.frame_id = self.odom_frame
        est.distance = float(r)
        est.bearing_abs = float(bearing_abs)
        est.bearing_rel = float(normalize_angle(bearing_abs - yaw))
        est.range_sigma = float(math.sqrt(max(var_r, 0.0)))
        est.bearing_sigma = float(math.sqrt(max(var_t, 0.0)) / r)
        est.age_ms = age_ms
        est.coasting = age_ms > COAST_AFTER_MS
        self.pub_fused.publish(est)
        self._broadcast_tag_tf(est.header.stamp, r, est.bearing_rel)

        self.get_logger().info(
            f"d={est.distance:5.2f}m  abs={math.degrees(est.bearing_abs):+7.2f}deg  "
            f"rel={math.degrees(est.bearing_rel):+7.2f}deg  "
            f"sig=({est.range_sigma:.2f}m, {math.degrees(est.bearing_sigma):.1f}deg)"
            f"  lat={self._latency_ns / 1e6:+.0f}ms{'' if self._lat_confident else '?'}"
            f"{'  COASTING' if est.coasting else ''}",
            throttle_duration_sec=1.0,
        )


    def _broadcast_tag_tf(self, stamp, r, bearing_rel):
        """Broadcast the filtered tag as base_link -> tag_est_link (2D: z stays 0)."""
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.base_frame
        t.child_frame_id = self.tag_est_frame

        t.transform.translation.x = r * math.cos(bearing_rel)
        t.transform.translation.y = r * math.sin(bearing_rel)
        # Rotation stays identity: a ranged point has position but no orientation.
        t.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    """Entry point: spin the tag estimator until interrupted."""
    rclpy.init(args=args)
    node = TagEstimator()
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
