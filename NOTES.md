# Notes

## Purpose

The running state of the project: current focus, build log, navigation modes, brainstorming,
open questions, and hardware setup notes. Authoritative for what's true and decided
right now — the live counterpart to PROJECT_PLAN.md's durable specs.

## To Do

### Current focus
Data path up is working (Phases 3–6 ✅). Next: **command path down** — setpoint cmd path +
cmd-timeout failsafe on the ESP32, then a `NavigateToPose` action server and the Goal Pose
click demo. See PROJECT_PLAN.md Phases 2 and 10 for detail.

### Open issues 
- Reverse is invisible — `odo` doesn't tick backwards, so dead reckoning freezes in reverse.
- UWB only reliable to +/- 60 degrees. 

### Open questions

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


