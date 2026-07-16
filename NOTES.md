# Notes

## Purpose

The running state of the project: current focus, build log, navigation modes, brainstorming,
open questions, and hardware setup notes. Authoritative for what's true and decided
right now — the live counterpart to PROJECT_PLAN.md's durable specs.

## To Do

### Current focus
Data path up is working (Phases 3, 4 core, and 6 ✅ — the Phase 5 Pi fusion node is NOT
built, and as of 2026-07-16 the ESP32's onboard fusion is **stripped**: no `fused_*`
telemetry, no tag estimate anywhere until Phase 5 ships. Raw `uwb_*` + `yaw`/`yaw_rate`
are the fusion node's inputs; tunables snapshot below).
**Command path down ✅ 2026-07-13**: ESP32 accepts `target_speed` + `target_heading`
frames with validation, `NavMode::REMOTE` (now the boot default, holding boot heading),
and the cmd-timeout failsafe (revised: throttle-only; steering holds last heading) —
bench-validated on the stand. Next: **update the serial bridge** for the changed frame
(`cam_*` fields gone; `lax`, `cmd_speed`/`cmd_heading`/`cmd_age`, `throttle`/`steering`
added; 2026-07-16: `enc_speed`/`fused_speed` dropped and `speed` now carries the fused
estimate — the throttle PID's feedback — not the raw hall speed), then a
`NavigateToPose` action server and the Goal Pose click demo. See
PROJECT_PLAN.md Phases 2 and 10 for detail.

Design spec for the command-path phase (interface, failsafe, HAL transition) is drafted at
[docs/hal-command-path.md](./docs/hal-command-path.md) — settled decisions fold back into
PROJECT_PLAN once implemented.

- **Capabilities announcement (todo)**: ESP32 declares hardware limits over serial (boot +
  `{"get":"caps"}` + on rtConfig change): max_speed, pan_max_deg, pan_slew_dps,
  cmd_timeout_ms, fw id. Pi discovers limits instead of duplicating config — duplication
  can't work anyway since maxSpeedMph is dashboard-tunable at runtime.

- **Split rpm.cpp into hall / encoder / speed-fusion modules (todo, deferred 2026-07-16)**:
  rpm.cpp mixes three concerns with separate state — hall driver (ISR, EMA, odometry,
  spike rejection), AS5600 encoder driver (I2C, EMA velocity, cogging state machine),
  and the 2-state fused-speed KF. Split into `hall.cpp`, `encoder.cpp`, and a fusion
  module (`speed_fusion` or `speed_est` — NOT `rpm_fusion`, nothing is in RPM units;
  and not `hal.cpp`, which collides with serial_hal's HAL). Coupling to route through
  main.cpp per the existing wiring pattern: (1) cogging ↔ hall is bidirectional — hall
  speed gates cogging detection, cogging flag zeroes hall speed and pauses odometry;
  (2) fusion corrects on per-sample *raw* values (currently driver-local), so drivers
  must expose new-raw-sample events; the hall-silence-at-speed zero correction is
  fusion policy consuming hall staleness, not hall-driver logic; (3) the speed-ramped
  encoder R gates on the fused estimate, so `fused_enc_r()` moves to the fusion module.
  KF correction ordering (predict → encoder correct → hall correct per loop) must
  survive exactly; re-verify on the bench. Field rename (hallRaw/hallSpeed/encRaw/
  encSpeed/fusedSpeed, done 2026-07-16) already landed separately.

### ESP32 fusion split — permanent vs removable (decided 2026-07-16)
Authoritative placement of estimation/fusion logic. Rule: complexity lives on the Pi unless
it is inside an onboard control loop or must survive serial loss.

**Permanent on the ESP32 (never migrates):**
- **Speed estimation** (hall EMA + spike rejection, rpm.cpp) — it is the throttle PID's
  measurement; putting the serial link inside that loop is never acceptable.
- **Cogging detection** — fused into the speed estimate (rpm.cpp forces `speedMph` → 0 while
  cogging is latched), so it is part of the PID measurement path, and it gates odometry
  accumulation at the integrator. Detector quality is a control concern: a false positive
  reads as zero speed and winds the throttle up.
- **Odometry integration** (+ its cogging gate) — must integrate at pulse resolution on-device.

**Removable — and now removed (stripped from firmware 2026-07-16, before Phase 5, by
explicit decision; Phase 5 is the replacement):**
- **Tag bearing/distance Kalman + uncertainty/erratic detector** (`fusion.cpp` entirely) —
  bearing composition verified correct 2026-07-16 (do not "fix" its sign); improvements
  (UWB lag compensation ~300–500 ms anchor smoothing, yaw-rate trust gating, AOA
  linearization ~23° residual at 60°) are built Pi-side only, never in fusion.cpp.

**Fusion tunables snapshot (captured 2026-07-16, before the firmware strip — these values
otherwise exist only in deleted code; Phase 5 port starts here):**
- Bearing KF: scalar Kalman on absolute compass bearing; measurement `yaw − uwb_bearing`
  wrapped to [0,360); innovation wrapped to ±180; gain `k = P/(P+R)`; no per-step process
  noise — P grows only via staleness: `P += (STALE_UNCERTAINTY/TIMEOUT_SEC)·dt` every loop.
  Seed: bearing = boot yaw (tag assumed ahead), initial P = 1000 deg².
- `FUSION_KALMAN_R_UWB = 15.0` deg² (stationary bench: σ≈3–4°)
- `FUSION_SENSOR_TIMEOUT_SEC = 3.0` s; `FUSION_STALE_UNCERTAINTY = 150` deg²
  (steady state ~17, erratic movement ~120 — gate threshold calibration)
- Erratic detector: `mean += 0.4·(innov − mean)` (`FUSION_INNOV_MEAN_ALPHA`);
  `ewma = 0.15·(innov − mean)² + 0.85·ewma` (`FUSION_INNOV_EWMA_ALPHA`), capped at
  1.1×STALE_UNCERTAINTY. Reported uncertainty = P + ewma.
- Distance KF (scalar, cm): `UWB_KALMAN_Q = 8.0` (process, tracks walking speed),
  `UWB_KALMAN_R = 20.0` (stationary σ≈4 cm); dead-reckoned between fixes by subtracting
  wheel-odometry delta (floor 0).
- Known flaws to fix in the port, not reproduce: anchor angle lag ~300–500 ms uncompensated;
  AOA nonlinearity uncorrected (~23° residual at 60° true); erratic detector inflates
  *reported* uncertainty but never the Kalman gain, so recovery from a genuine step is slow.

**Pi's role for the permanent items:** offline characterization and threshold
validation/tuning from telemetry (`enc_speed`, `speed`, `throttle`, `cogging`) — never
runtime detection. Open firmware question noted 2026-07-16: the speed-threshold cogging
clear (rpm.cpp `RPM_COGGING_MAX_SPEED_MPH` check) looks unreachable while latched, since
cogging forces `speedMph` to 0 — encoder velocity is the only live exit path.

### Pi reimplementation checklist (behaviors stripped from the ESP32 2026-07-13/14 that
PROJECT_PLAN does not yet capture explicitly — fold into Phases 5/8 when building them)
- **Stale-estimate throttle gating**: FOLLOW_ME only drove when fusion uncertainty was
  below threshold. Pi rule: stop sending speed setpoints when its own fusion uncertainty
  exceeds ~150 deg² (`fused_unc` telemetry is gone as of the 2026-07-16 strip). Threshold +
  calibration ("steady state ~17, erratic ~120") preserved in the tunables snapshot above.
- **Tag-distance dead reckoning**: fusion.cpp decrements the Kalman distance by wheel
  odometry between UWB fixes so distance stays live through ranging dropouts. Phase 5's
  spec ("filter UWB bearing, track uncertainty") omits it — port it, or follow speed
  degrades exactly when UWB gets flaky.
- **Erratic-motion detector**: the innovation-variance EWMA beside the KF
  (`_innovMean`/`_innovEwma`, alphas 0.4/0.15 in config.h) that inflates uncertainty when
  readings scatter. Part of "port the Kalman scheme" but a distinct mechanism, easy to
  miss — and it's what makes the gating rule above actually trip.
- **Pan measurement model** (deferred from PROJECT_PLAN deliberately): with the pan mount,
  absolute bearing = `yaw + pan_angle + uwb_bearing` (three values from one telemetry
  frame, same `ts`); inflate measurement noise while `pan_angle` is changing; tf tree
  `base_link → pan_mount → uwb_anchor` is the motivation for Pi-side fusion.
- **Setpoint speed cap documentation**: the ESP32 rejects `target_speed >
  rtConfig.maxSpeedMph` (2.5 default) — not documented in PROJECT_PLAN's setpoint-frame
  section or the T2 brief (T2 only says clamp ≥ 0). Bridge should clamp to the cap.
  (Rejects are now visible: `cmd_rejects` counter added to telemetry 2026-07-14.)
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
- Heading conversion: verify the ESP32 IMU yaw convention (direction of increase, zero
  reference) against REP-103 yaw, and bench-verify the bridge's odom-frame → device-compass
  conversion (offset derived from `imu/data` vs `odom`) before the first Pi-commanded drive.
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

## Build Log

What's been built, newest first. Release-notes style — one line per change. Gotchas at the bottom.

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


