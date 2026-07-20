# Notes

## Purpose

The running state of the project: current focus, build log, navigation modes, brainstorming,
open questions, and hardware setup notes. Authoritative for what's true and decided
right now — the live counterpart to PROJECT_PLAN.md's durable specs.

## To Do

### Current focus
Data path up is working (Phases 3, 4 core, and 6 ✅). The Phase 5 Pi fusion node is NOT
built — and as of 2026-07-16 the ESP32's tag Kalman is deleted too, so the wire carries
raw `uwb_*` only: raw fixes drive `tag_link` directly until Phase 5 provides filtering.
**Command path down ✅ 2026-07-13**: ESP32 accepts `target_speed` + `target_heading`
frames with validation, `SETPOINT` mode (the boot default, holding boot heading; named
`REMOTE` until 2026-07-16), and the cmd-timeout failsafe (revised: throttle-only;
steering holds last heading) — bench-validated on the stand. Bridge updated 2026-07-16
to the slimmed telemetry frame (full schema in PROJECT_PLAN "Serial Protocol").
A reactive follow-me controller (`nav_controller`) now closes the loop: fused tag ->
`cmd_drive` -> car (first cut of Phase 8, 2026-07-17). Next layers: confidence gating +
the ±60° recovery behavior, then the `/follow_me` action and the `NavigateToPose` /
waypoint action servers. See PROJECT_PLAN.md Phases 8-10.

(The command-path design spec `docs/hal-command-path.md` was deleted 2026-07-14: the phase
is implemented and its settled decisions were folded into PROJECT_PLAN's Serial Protocol
section, which supersedes the draft on every point where they differed — `target_heading`
naming, throttle-only failsafe, REMOTE/DIRECT/STOPPED modes, echo-field set.)

- **Capabilities announcement (todo)**: ESP32 declares hardware limits over serial (boot +
  `{"get":"caps"}` + on rtConfig change): max_speed, pan_max_deg, pan_slew_dps,
  cmd_timeout_ms, fw id. Pi discovers limits instead of duplicating config — duplication
  can't work anyway since maxSpeedMph is dashboard-tunable at runtime.

### Pi reimplementation checklist (behaviors stripped from the ESP32 2026-07-13/14 that
PROJECT_PLAN does not yet capture explicitly — fold into Phases 5/8 when building them)
- **Stale-estimate throttle gating**: FOLLOW_ME only drove when fusion uncertainty was
  below threshold. Pi rule: stop sending speed setpoints when the Phase 5 fusion's own
  uncertainty exceeds ~150 deg² (`fused_unc` left the wire 2026-07-16, so there is NO
  gating signal until Phase 5 ships — interim follow logic should gate on `uwb_age`
  instead). Threshold + calibration notes ("steady state ~17, erratic ~120") live only
  in esp32 `config.h` history (`FUSION_STALE_UNCERTAINTY`).
- **Tag-distance dead reckoning**: the (now-deleted) fusion.cpp decremented the Kalman
  distance by wheel odometry between UWB fixes so distance stayed live through ranging
  dropouts. Phase 5's spec ("filter UWB bearing, track uncertainty") omits it — port it
  from esp32 repo history, or follow speed degrades exactly when UWB gets flaky.
- **Erratic-motion detector**: the innovation-variance EWMA beside the KF
  (`_innovMean`/`_innovEwma`, alphas 0.4/0.15 in config.h, both in esp32 repo history)
  that inflated uncertainty when readings scattered. Part of "port the Kalman scheme"
  but a distinct mechanism, easy to miss — and it's what makes the gating rule above
  actually trip.
- **Pan measurement model** (deferred from PROJECT_PLAN deliberately): with the pan mount,
  absolute bearing = `yaw + pan_angle + uwb_bearing` (three values from one telemetry
  frame, same `ts`); inflate measurement noise while `pan_angle` is changing; tf tree
  `base_link → pan_mount → uwb_anchor` is the motivation for Pi-side fusion.
- ~~Setpoint speed cap documentation~~ **resolved 2026-07-14**: now documented in
  PROJECT_PLAN's setpoint-frame validation note and in the T2 brief. Speed gating lives on
  the ESP32 alone; the Pi-side clamp and `max_speed_mph` param were removed 2026-07-17.
  Rejects are visible via the `cmd_rejects` telemetry counter.
- **Follow tunables migration**: `followDistanceCm`/`maxDistanceCm`/`minSpeedMph`/
  `maxSpeedMph` become ROS params in the Phase 8 port (only `maxSpeedMph` still has an
  onboard consumer — validation + PID normalization); the ESP32 dashboard follow-behavior
  sliders are then dead UI to remove.

### Open issues 
- Reverse is invisible — `odo` doesn't tick backwards, so dead reckoning freezes in reverse.
- UWB only reliable to +/- 60 degrees. 
- UWB bearing lags reality by ~0.3s (anchor-side smoothing, measured 2026-07-14) — Pi
  should inflate bearing uncertainty while `pan_angle` is changing between frames; see
  2026-07-14 build log for the deferred delayed-reporting mitigation.
- No battery voltage sensing on the car — pack voltage can't be streamed or alarmed on;
  hardware gap (noted 2026-07-13).
- `lax` telemetry field: verify on the bench that the BNO085 x-axis really is forward/back
  on this mounting, and note the sign convention for "forward".

### Parked concerns (noted, deliberately ignored unless symptomatic)
- **Tag/bearing lag is not latency-critical by architecture** (parked 2026-07-14): the car
  never steers on the tag estimate directly — the steering PID closes on IMU compass yaw
  (low-latency, onboard, 50 Hz); the tag estimate only moves the heading *setpoint*, and a
  walking tag moves it slowly. This covers the ~0.3 s UWB anchor smoothing lag and any
  Pi-side fusion/transport latency. Revisit only if following looks sluggish or overshoots
  when the tag turns sharply — the symptom would be lag-shaped, not instability-shaped.

### Open questions
- ~~Command-path numbers~~ **decided 2026-07-12**: 20 Hz re-send / 300 ms timeout, hybrid
  resume — see PROJECT_PLAN "Command stream contract".
- ~~Boot default is still `FOLLOW_ME`~~ **resolved 2026-07-13**: `DEFAULT_NAV_MODE` is now
  REMOTE — a rebooted car waits at zero throttle holding its boot heading instead of
  re-entering autonomy (FOLLOW_ME's onboard control block is commented out entirely). The
  bridge's halt-TX-on-reboot behavior stays as hygiene.
- ~~`wheel/state` hall staleness~~ **moot 2026-07-16**: the wire no longer exposes the
  raw hall value — `speed` is the ESP32's fused estimate and no Pi-side speed fusion is
  planned, so per-sensor staleness is the ESP32's problem.
- Heading conversion: verify the ESP32 IMU yaw convention (direction of increase, zero
  reference) against REP-103 yaw, and bench-verify the bridge's odom-frame → device-compass
  conversion (offset derived from `imu/data` vs `odom`) before the first Pi-commanded drive.
- ~~`fused_angle` frame ambiguity~~ **moot 2026-07-16**: `fused_angle` left the wire with
  the ESP32 tag Kalman. Raw is settled (anchor-relative, spec formula: absolute =
  yaw + pan_angle + uwb_bearing). The ambiguity lesson carries to Phase 5: define the
  fusion output's frame explicitly, including the pan term.
- ~~ESP32 wrapped-heading-error implementation~~ **verified 2026-07-13**: bench seam test
  passed (yaw≈0, `target_heading:350` steered the short way).
- (Deferred with the raw-actuator mode: steering sign convention + max steering angle
  calibration — only needed if/when the loops migrate to the Pi.)

---

## Navigation Modes / Mission Profiles

### Mode 1 — Single car, dead reckoning + UWB follow-me (current plan)
- Follow-me: UWB AoA bearing + distance, camera fusion, PID control
- Commanded nav: waypoint missions using IMU yaw + RPM odometry
- Nav2-compatible action interfaces

### Mode 2 — Single car, camera-based person following
- Primary sensor: camera with person detection algorithm (no wearable required)
- Servo-actuated camera pan to keep person in frame
- Camera decision: **add a camera attached directly to the Pi** rather than
  streaming from the existing XIAO/OV2640 — its I2C link can't carry pixels, and WiFi/USB
  streaming firmware is hassle for a worse result. The mounted XIAO stays as-is (optional
  blob sensor only)
- Hardware options (all Pi-direct):
  - **USB webcam** (~$15): 30 fps over V4L2, standard `usb_cam` ROS2 node, zero custom firmware — cheapest entry
  - **Pi Camera Module 3** (CSI): better quality/latency than USB, Pi does all inference (YOLOv8n ~10-15 fps at reduced resolution)
  - **OAK-D Lite** (~$150): onboard Myriad X neural inference chip, offloads detection from Pi, also provides stereo depth (replaces UWB distance)
- Servo pan channel: additional PWM output from ESP32 HAL

### Mode 3 — Two-car formation
- Car 1: runs waypoint mission via dead reckoning (drift acceptable for demo purposes)
- Car 2: follows Car 1
- Two complementary layers:
  - **ROS2 over WiFi**: mission coordination, start/stop, state sharing — both cars on same network, DDS auto-discovers with namespacing (`/car1/`, `/car2/`)
  - **UWB AoA between cars**: Car 2 uses its DW3000 to range + get bearing to Car 1's tag — low-latency physical following independent of WiFi round-trip
- Car 2's following does not depend on Car 1's dead reckoning accuracy — UWB gives real relative position

---

## Brainstorming

### Future hardware additions
- LIDAR (e.g. RPLIDAR A1) — would unlock full Nav2 with SLAM and obstacle avoidance
- OAK-D Lite — enables camera-based person detection (Mode 2) + stereo depth

### Future software ideas
- **Follow-me as waypoint planning with recovery** — response to the AoA ±60°
  FOV limitation above. Instead of steering directly on the instantaneous UWB bearing,
  convert confident tag fixes into waypoints in the `odom` frame and follow those. Anomalous
  UWB readings (angle clamped at ±90, bearing jump inconsistent with dead-reckoned motion,
  fusion uncertainty spike) then trigger a *recovery plan* rather than steering on a false
  tag location. Fits naturally once `NavigateToPose` exists: follow-me becomes continuous
  goal-updating on the same nav machinery as waypoint missions, unifying Mode 1 with the
  commanded-nav system.

  Sketched recovery behavior:
  1. When bearing approaches/exits the reliable cone (|bearing| past ~60°), store the last
     confident tag location.
  2. Set a recovery goal that turns the robot until its compass heading faces that stored
     location.
  3. On goal completion, UWB bearing is considered trustworthy again and normal following
     resumes.

  Engineering notes on the sketch:
  - **Store the tag location in the `odom` frame** (dead-reckoned pose + bearing + distance
    → absolute position), not as a relative bearing — a relative value goes stale the moment
    the robot moves/turns. "Face the tag" then = atan2 in odom vs. compass yaw.
  - **Ackermann can't rotate in place** — the "rotate to face" step is really a steering
    maneuver (arc forward, or K-turn if reverse is available). Simplest version that subsumes
    it: set the recovery goal to *drive toward* the stored tag location; heading converges
    onto it en route and the bearing swings back into the cone naturally.
  - **Distance stays valid during bearing clamp** — ToF ranging doesn't depend on PDoA
    (bench-confirmed: dist read a sane 27-39cm while angle was pinned at 90). Range is usable
    as a consistency check during recovery.
  - **Re-acquisition should require more than goal completion**: N consecutive readings with
    |bearing| < 60°, sample-to-sample jitter present (a live reading jitters; a clamp doesn't),
    and distance consistent with the stored location.
  - Two trigger tiers: proactive (|bearing| climbing past ~50-55° while still valid → re-aim
    early with a trustworthy fix) and reactive (clamp/jump signature → fall back to stored
    location).
- Web UI on Pi for sending waypoint missions from a phone
- Record and replay a driven path as a waypoint mission
- Multi-tag support — follow one of several tagged people
- Return-to-home behavior when tag is lost for too long

---

## Code Rationale & Gotchas (relocated from inline comments)

Non-obvious "why" pulled out of code comments (2026-07-17) when the comment rule tightened to
description-only. The code keeps one-line pointers back here.

**serial_bridge.py**
- *Topic map* (all names RELATIVE; namespace set at launch, e.g. `fmbot` → `/fmbot/imu/data`,
  which is what makes multi-robot work without touching source). Publishes: `imu/data`
  (orientation, yaw rate, forward accel), `wheel/state` (fused speed, odometer, cogging),
  `command/status` (mode + last-accepted cmd echo), `actuator/status` (throttle/steering/pan),
  `uwb/raw` (DW3000 range/bearing; stays un-processed on the Pi — Phase-5 estimator consumes it),
  `joint_states` (pan servo → `base_link→uwb_link` TF; steer joints cosmetic). Subscribes:
  `cmd_drive` (latched, re-sent 20 Hz), `odom` (feeds heading offset). *(May fit PROJECT_PLAN better.)*
- Header stamps = ESP32 device time mapped to the ROS clock; the offset cancels in subtraction so
  `dt` stays exact device time while tf2 still accepts the stamps. Raw device uptime would stamp in
  1970 and tf2 would drop the frames.
- `_open_port`: `write_timeout` is mandatory — a wedged port must fail the write fast rather than
  block the executor thread that drives the 20 Hz TX timer.
- `_read_loop`: publishes the live handle to `self._ser` under the lock so the TX timer writes the
  same port; clears it to `None` on disconnect so a TX tick in a reconnect gap drops.
- Wheel/state is published BEFORE imu/data deliberately: pose_estimator's IMU callback samples its
  cached wheel reading, so wheel-first keeps that cache same-frame instead of ~20 ms stale.
- IMU: orientation available but covariance unknown → zeros; `covariance[0]` flips −1→0 (available,
  variance unknown). `yaw_rate` converts UNNEGATED (matches the yaw treatment, so orientation+rate
  stay consistent). `lax` is forward-axis accel (BNO085 x).
- Pan joint NEGATED: wire `pan_angle` is +right (CW), a TF rotation about +z is +left (CCW).
  Modeling pan here means raw/fused bearings compose to the tag's absolute position through the TF
  tree with no separate pan term downstream.
- Device-yaw pairing: map the time OUTSIDE `_tx_lock` — `_device_ts_to_ros_ns` itself takes
  `_tx_lock` on a reboot frame, and the plain `Lock` is not reentrant.
- Zero-stamp rule (documented deviation from the umbrella brief): `ros2 topic pub` leaves
  `header.stamp` at 0, which reads as infinitely stale and blocks all bring-up. Treat a zero stamp
  as arrival time; otherwise honor the sender's stamp so genuine staleness still gates TX.
- Heading-offset (`_on_odom`): inbound imu/data uses device yaw UNMODIFIED; pose_estimator captures
  `_yaw_offset` = first `ros_yaw` and publishes `odom θ = normalize(ros_yaw − _yaw_offset)`.
  Inverting: `offset_deg = wrap_pm180(device_yaw_deg − degrees(odom_yaw))`, then
  `target_heading = wrap_0_360(degrees(cmd.heading) + offset_deg)` (PLUS sign). EMA runs on the
  WRAPPED delta so it survives the 0/360 seam.

**pose_estimator.py**
- Odometer is cached by the IMU callback; both come from the same serial frame, and the bridge
  publishes wheel before imu, so the sample is same-frame.
- Negative odometer delta = ESP32 reboot (the odometer only ticks up): re-baseline rather than
  teleporting the car backwards.
- Project the distance delta along the MIDPOINT heading (the car traced an arc over the step);
  using either endpoint biases every turn to one side.

**tag_broadcaster.py**
- Broadcast from `uwb_link` (the anchor), NOT `base_link`: the ~0.09 m lever arm is small vs the
  <10 cm ranging error but not zero, and parenting under `uwb_link` keeps the geometry honest AND
  gets the tag's absolute position for free (tf2 chains `odom→base_link→uwb_link→tag_link` live; a
  viewer in `odom` shows the true spot with no dead-reckoning drift baked into a stored edge).
- Projection is in the anchor's OWN frame, so it needs no rotation offset: `x=d·cos(angle)`,
  `y=d·sin(angle)` (angle 0 = ahead, +left per REP-103). The anchor rides the pan servo, so
  `base_link→uwb_link` is a revolute joint from serial_bridge's `/joint_states`; the pan is applied
  UPSTREAM and this node stays oblivious — `uwb_link→tag_link` is identical panned or not.
- No-fix (`distance ≤ 0`; ESP32 sends −1) is skipped rather than planting a phantom tag at/behind
  the origin.

**tag_estimator.py**
- Dedup by measurement time (`stamp − age`): the bridge re-reports each ~10 Hz fix on every ~50 Hz
  telemetry frame with growing `age_ms`; without dedup the same fix would fuse ~5×.

**bringup.launch.py**
- `DEFAULT_PORT` is imported from the bridge node so the launch and node defaults stay in sync —
  the exact mismatch that produced a silent Errno-2.
- robot_state_publisher needs the URDF *contents*, not a path — hand it a path and it silently
  fails to parse, publishing no `/tf_static` (imu_link dangles).
- rsp deliberately does NOT publish `odom→base_link`: that edge is pose_estimator's, and two
  publishers on one edge fight.

**tests**
- `test_serial_bridge_tx`: FakeSerial is injected via `_open_port` (monkeypatched at class level,
  since the reader thread opens the port inside `__init__`); do NOT use pyserial's `loop://`, whose
  loopback would feed our own TX frames back into the reader. Determinism = calling `_tx_tick`
  directly; only the first test does a short real spin to prove the 20 Hz stream.
- `test_tag_estimator`: pure callback math — determinism from calling callbacks + publish tick
  directly with hand-built messages, publisher replaced with a recording stub, fix stamps from the
  node clock.

---

## Build Log

What's been built, newest first. Release-notes style — one line per change. Gotchas at the bottom.

### 2026-07-17
- **Latched-goal follow-me (`nav_controller`)** — first cut of Phase 8: subscribes `fused/tag_pose`
  + `odom`, publishes `cmd_drive`, broadcasts `odom -> nav_goal` (the dashboard renders it as a
  green target reticle, distinct from the tag dot). Commits the tag as a point in `odom` and steers
  to it (heading = atan2 to the goal; on/off `cruise_speed_mps` beyond `follow_distance_m` = 2.0 m,
  else hold; no reverse). Trust gate on the fused bearing sigma: above `bearing_sigma_high_deg`
  (10°) it HOLDs — freezes the goal, keeps driving to the last trusted point — and re-acquires after
  `reacquire_count` (5) fixes below `bearing_sigma_low_deg` (7°, hysteresis; measured normal is
  5-10°, >10 bad). Storing the goal in `odom` is what makes "keep going to the last known spot"
  fall out for free — dead reckoning keeps the heading correct as the car moves. No new msg/topic —
  reuses `cmd_drive`/`fused/tag_pose` + one TF frame. In bringup behind `follow:=true` (default
  off — it drives the car); by hand it needs the matching namespace (`-r __ns:=/fmbot`) or its
  topics don't connect. Deferred: the `/follow_me` action (feedback/give-up/cancel), a give-up timeout, a proactive
  pan-saturation trigger. **Gotcha:** default cruise is 3 mph, above the ESP32's 2.5 mph cap — raise
  `maxSpeedMph` on the dashboard or every frame is rejected (`cmd_rejects` climbs, car won't move).

### 2026-07-16
- **Telemetry slim-down: bridge + topics match the new ESP32 frame** — the ESP32 dropped
  its tag Kalman (`fused_angle`/`fused_dist`/`fused_unc`) and per-sensor speed
  (`enc_speed`); `speed` is now the onboard fused estimate (throttle PID feedback) and
  the remaining ESP32 fusion is permanent, so the `fused/` isolation namespace is
  retired. Accordingly: `FusedTagPose.msg` + `fused/tag_pose` deleted; `CoggingStatus.msg`
  + `fused/cogging` deleted, cogging folded into `wheel/state` as a bool; new wire fields
  `yaw_rate` → `imu/data` `angular_velocity.z` (unnegated, consistent with yaw) and
  `mode` (`SETPOINT`/`DIRECT`/`STOPPED` — `REMOTE` renamed `SETPOINT` on the ESP32) →
  new `CommandStatus.mode` string. `tag_broadcaster` collapsed to a single instance on
  `uwb/raw` → `tag_link` (`source` param, `tag_raw_link`, and `raw_tag_broadcaster`
  removed). Interfaces package needs a clean rebuild — the .msg set changed.
- **Topic layout: raw-vs-derived convention + renames** — adopted the convention (now in
  PROJECT_PLAN "Key topics"): raw co-sampled wire fields bundle into one stamped message;
  derived estimates get their own per-producer topics (`fused/*` / future `pi_fused/*`).
  Accordingly: `wheel/speed` + `wheel/enc_speed` + `wheel/distance` (unstamped Float32s)
  merged into `wheel/state` (`WheelState`, stamped — fixes pose_estimator pairing wheel
  data with IMU by arrival time; bridge publishes wheel BEFORE imu so the cached sample
  is same-frame); `car/status`/`CarStatus` renamed `actuator/status`/`ActuatorStatus`
  ("car" was redundant inside the robot namespace). Interfaces package needs a clean
  rebuild (`rm -rf build install log`) since the .msg set changed.
- **Raw tag visualization** — `tag_broadcaster` gained a `source` param (`fused` default /
  `raw`); bringup launches a second instance (`raw_tag_broadcaster`) that projects
  `uwb/raw` to `uwb_link -> tag_raw_link`, so raw vs fused tag position render side by
  side. Fixed `UwbRaw.msg` frame comment: bearing is ANCHOR-relative, not car-relative
  (spec: absolute = yaw + pan_angle + uwb_bearing).
- **URDF visual pass + steerable front wheels** — front wheel joints are now revolute
  (steer about z), driven cosmetically from telemetry `steering` via `/joint_states`
  (`STEER_SIGN`/`MAX_STEER_RAD` in serial_bridge are uncalibrated guesses: ±30° lock,
  wire + = right — flip/correct when measured). Body shell enlarged to cover the chassis
  plate (0.300 × 0.170); UWB redrawn as a flat front-back board on the hood (pan-axis
  lever arm now x = 0.090 — measure); IMU moved to the rear shell deck (x = −0.110,
  z = 0.125 — measure). Sensor joint origins are TF-real, not just cosmetic.

### 2026-07-14
- **ESP32 mode restructure: REMOTE / DIRECT / STOPPED** — the mode roster now matches the
  HAL role. `DIRECT` implements the raw-actuator frames (`{"throttle":..,"steering":..}`,
  validated: finite, throttle [0,1] no-reverse, steering [-1,1]; own 300 ms failsafe =
  throttle cut + steering holds; `target_pan` accepted in any frame shape, so DIRECT
  drives all three actuators). FOLLOW_ME / TEST / THROTTLE_TEST deleted (follow logic
  readable at esp32 commit `075ab58`), **`nav.cpp`/`nav.h` eliminated** — control owns the
  mode (`ControlMode`, `control_set_mode`/`control_mode`); dashboard buttons are now
  Remote / Direct / Stopped and its target arrow reads control's held REMOTE heading.
  Speed PID measure re-pointed from fusion's `fusedSpeedMph` to `rpm` directly →
  control.cpp has no fusion dependency (prerequisite for the fusion strip). Bench note:
  THROTTLE_TEST's slider is gone — bench throttle testing is now DIRECT mode + pasted
  direct frames (car-quiet build); dashboard direct-sliders are a possible follow-up.
- **Pan servo for the UWB anchor** (GPIO 6, driven by direct LEDC at 50Hz/14-bit — NOT
  ESP32Servo: its S3 MCPWM path routes 3rd+ servos' GPIO to the wrong timer output, so the
  pin silently carries another servo's waveform; bug at ESP32PWM.cpp:492).
- **pan-cal env**: UWB-referenced calibration — 5-point least-squares fit of bearing vs
  pulse width → `PAN_SERVO_US_PER_DEG` (≈−10.5, ~±65° travel at 800–2200µs endpoints) and
  trim from the fit's bearing-zero. Measurement span is ±30% of travel so apparent bearings
  stay inside the DW3000's linear zone (AOA response bends past ~±40°).
- **Anchor angle latency measured** (pan-cal phase 4): the anchor firmware smooths its
  angle output — ~0.3s effective latency, constant across pan rates. While the pan moves,
  the reported bearing is stale by (pan rate × 0.3s); a moving average also *smears*
  during motion, so prefer small discrete pan corrections over long sweeps. Deferred
  mitigation: ESP32 reports pan_angle from 0.3s ago so bearing + pan_angle in one telemetry
  frame describe the same instant (`PAN_REPORT_DELAY_MS` idea, not yet implemented — current
  firmware reports the live slew-limited angle).
- **HAL pan interface**: optional `target_pan` in the command frame (deg, 0 = car nose,
  +right, ±90 validation; absent field = keep current target); firmware clamps targets to
  a symmetric ±55° (`PAN_MAX_DEG`) so the Pi sees one limit both ways despite asymmetric
  physical travel (~+69°/−64° at current trim); ESP32 slews at ~50°/s regardless of
  commanded jump size; live `pan_angle` (deg, same convention) added to the telemetry
  frame. Pan-aiming *policy* deliberately lives on the Pi; the tf tree corrects bearings
  by `pan_angle`.

### 2026-07-12 (later — supersedes the entry below)
- **Decision revised: heading-setpoint interface** — command frame is now
  `{"target_speed":<mph>,"target_heading":<compass deg>}`. Priority shifted to a minimal
  ESP32 diff: the tuned steering PID stays onboard and only its error source swaps by mode
  (FOLLOW_ME: tag-relative `fused_angle` → 0; Pi mode: wrapped `imu.yaw − target_heading`
  → 0 — structurally identical, gains transfer). All fusion behavior stays in place.
  Steering-as-direct-position survives as the reserved raw-actuator mode / eventual pure-HAL
  end state. Command topic is now custom `cmd_drive` (`follow_me_interfaces/DriveCommand`:
  header + speed m/s + heading rad in `odom` frame; bridge converts to device compass deg on
  write) — `cmd_ackermann` superseded (Ackermann's `steering_angle` is a wheel angle, not a
  heading). Gotchas recorded in open questions: odom↔device yaw offset, [-180,180] error wrap.

### 2026-07-12
- **Decision — steering commanded as direct normalized position [-1, 1]** *(superseded same
  day — see entry above)*, replacing
  PROJECT_PLAN's earlier `{"target_speed","target_angle"}` frame. Rationale: the ESP32
  steering PID's measure is tag-relative `fused_angle`, so a bearing setpoint only means
  anything while onboard fusion tracks the tag — unusable for teleop and for `NavigateToPose`
  (no tag). The bearing regulator moves to the Pi as the Phase 8 controller (heading PID +
  kinematic feedforward); the speed PID stays on the ESP32. A cascaded ESP32-side heading
  controller was considered and rejected (thin-HAL inversion, no latency need, yaw-rate inner
  loop has zero authority at v=0) — see the decision log in PROJECT_PLAN's Control Loop
  Placement section.
- Interim command topic changed to `cmd_ackermann` (`ackermann_msgs/AckermannDriveStamped`)
  from `/cmd_vel` (`Twist`); thin Twist→Ackermann shim planned for stock teleop tools.
- PROJECT_PLAN.md amended: Serial Protocol outgoing frame, Control Loop Placement (rewritten),
  Phases 2/3/7/8, Key topics table. `serial_hal.cpp` header comment updated in the ESP32 repo.

### 2026-07-10
- `tag_broadcaster.py`: subscribes `tag/pose`, broadcasts `uwb_link → tag_link` on `/tf` per fix (~10 Hz) — the DW3000 tag now shows as a moving TF frame.
- Parented under `uwb_link` (the anchor), not `base_link`: matches where the bearing/distance are actually measured, and tf2 chains `odom → base_link → uwb_link → tag_link` so the tag's absolute position comes for free with no drift baked into a stored edge.
- Anchor is rpy=0 vs base_link, so bearing needs no rotation offset: `x = d·cos(angle)`, `y = d·sin(angle)`; identity rotation (a point has none).
- Skips broadcast when `distance <= 0` (ESP32 sends -1 on no fix) — no phantom tag at the origin.
- Added to `bringup.launch.py` + `setup.py` entry points.

### 2026-07-09
- URDF + robot_state_publisher: RSP publishes `base_link → {chassis, body, 4 wheels, imu_link, uwb_link}` from `follow_me_car.urdf`; fixes the dangling `imu_link`.
- URDF is box/cylinder primitives (Foxglove can't resolve `package://` meshes); dimensions and IMU/uwb positions are estimates — measure and correct.
- All URDF joints fixed (continuous would demand a `/joint_states` stream we don't publish).
- `bringup.launch.py`: RSP + serial_bridge + pose_estimator + foxglove_bridge (`foxglove:=false` to skip).
- Phase 6: `pose_estimator.py` dead reckoning — publishes `/odom` + TF `odom → base_link`; differences the odometer, projects along midpoint heading; heading from IMU, never integrated; `odom` starts at identity.
- rosbag2 record + replay verified: raw topics → `--clock` replay into the estimator (`use_sim_time:=true`), pose retraces in Foxglove with no hardware.
- `src/` now tracked + pushed to GitHub (`.gitignore` was missing build/install/log).
- Bridge: maps ESP32 uptime → ROS clock; converts to SI at the boundary; topic names relative (runtime namespace).
- Renames: `wheel/odometry` → `wheel/distance`; `FusedPose` → `FusedTagPose` (+ heading field); `/follow_me/pose` → `/tag/pose`.
- Removed `UWBReading.msg` + `CameraBlob.msg` — Phase 5 must re-add.

### 2026-07-08
- Phases 3–4: `follow_me_interfaces` (custom msgs) + `follow_me_nodes/serial_bridge.py` — reads ESP32 JSON over USB serial, skips ESP-IDF logs, reconnects on drop, publishes 4 topics.
- Verified on hardware: `/imu/data` ~44 Hz, values move in Foxglove.
- Created CLAUDE.md and cheat.md.

---

## Hardware Setup Notes

Nitty-gritty for re-setting-up a replacement board — just what's needed to get a fresh unit talking again, not project history.

### DW3000 AoA UWB

- Anchor talks over UART1 (`TXD1`/`RXD1` header pins, 115200 baud) — not native USB CDC.
- Command syntax is bare `CMDNAME arg1 arg2...` (not `AT+CMD=`) — confirmed via firmware repo `Makerfabs/UWB-AOA-with-Display-STM32F103C8T6`.
- Note: the generic `Makerfabs/MaUWB_ESP32S3-with-STM32-AT-Command` repo (range-only, no angle field in `AT+RANGE`) is a DIFFERENT product line — don't confuse the two when looking up firmware/docs.
- Firmware ships factory-flashed; ST-Link only needed if updating module firmware (repo has default `Project_Anchor_v1.0.hex` / `Project_Tag_v1.0.hex`).

**Two report modes:**
- Default JSON: `"JS"` + 4 hex-digit length + JSON payload, e.g. `{"TWR":{"a16":"E1AE","D":50,"Xcm":19,"Ycm":48,...}}` — bearing = `atan2(Xcm, Ycm)`.
- Binary "carfollow" mode (`USER_CMD 1` + `save`): fixed 31-byte frame, `0x2A` header, length byte, payload (`sn`, `addr16`, `angle` int32, `distance` int32 cm, plus power/accel fields), XOR checksum, `0x23` footer. Pre-computed `angle`/`distance` as plain ints — no parsing math needed on the ESP32 side.

**Pairing required before ranging starts** (once per tag, persists across power cycles): anchor auto-discovers the tag over UWB and emits an unsolicited `"NewTag":"<64-bit hex id>"` JSON message; host must reply `addtag <id64> <addr16> 0001 64 00` then `save` to bind it into the anchor's known-tag list (`fastrate=1`=10Hz, `useIMU=0` — matches Makerfabs' own Windows GUI defaults).

**Gotcha:** `addtag` can fail with `error function` (handler returns NULL) even when the known-tag list isn't full — a factory reset (`RTOKEN` command) clears the stale flash state. If `addtag` fails on a fresh board, try `RTOKEN` first.


