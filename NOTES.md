# Notes

## To Do

### Current Sprint
- [x] Get a 5V/3A USB-C power supply for Pi desk development
- [x] Flash Pi with Ubuntu 24.04 Server (64-bit) via Raspberry Pi Imager
- [x] Install ROS2 Jazzy on Pi (verified with `ros2 topic list`)
- [x] Verify SSH access from Mac (`ssh ubuntu@followme-pi.local`)
- [x] Validate DW3000 anchor/tag hardware end-to-end (discovery, pairing, both JSON and binary ranging output) — see DW3000 section below
- [x] Rewrite `uwb.cpp`/`fusion.cpp` on `follow-me-car-esp32` to consume the DW3000 protocol instead of RYUW122 trilateration (committed on main; old trilateration preserved via tag `v2.0-three-anchor-uwb`)
- [x] Integrate AS5600 magnetic encoder — role settled as cogging detection; hall-effect sensor stays for RPM/speed
- [x] Fast-forward `ros2-hal` branch to validated main (2495c85) and push to GitHub — HAL work starts from here
- [ ] **Interview sprint Day 1**: serial_hal telemetry stream (strip nothing) → bridge node + custom msgs → dead-reckoning pose estimator + TF2 → Foxglove/rosbag (see PROJECT_PLAN.md "Interview Critical Path")
- [ ] **Interview sprint Day 2**: setpoint cmd path + failsafe on ESP32 → NavigateToPose action server → Goal Pose click demo

---

## Hardware Updates (Planned)

### DW3000 AoA UWB — replaces 3x RYUW122
- Hardware: Makerfabs [MaUWB STM32 AOA Development Kit](https://www.makerfabs.com/mauwb-stm32-aoa-development-kit.html) — anchor + tag pair, STM32F103C8T6 + Qorvo DW3000, dual antenna
- Correction: this is STM32-based, not ESP32-based as originally assumed. No onboard ESP32 co-processor.
- Controlled over UART1, same physical pattern as the RYUW122s, but a different command syntax — bare `CMDNAME arg1 arg2...` (not `AT+CMD=`) — confirmed via firmware repo `Makerfabs/UWB-AOA-with-Display-STM32F103C8T6`
- Spec (from that repo's README): angle ±60°, angle error ±5°, ranging error <10cm, positioning error <10cm, coverage radius 30m@6.8Mbps
- Note: the generic `Makerfabs/MaUWB_ESP32S3-with-STM32-AT-Command` repo (range-only, no angle field in `AT+RANGE`) is a DIFFERENT product line — don't confuse the two when looking up firmware/docs
- One board on car as anchor, one as tag (carried by person or mounted on Car 2)
- Main ESP32-S3 reads anchor's UART output, aggregates into existing JSON serial frame to Pi — single serial connection preserved
- Serial protocol changes: `uwb_l/uwb_r/uwb_f` → `uwb_dist` + `uwb_bearing`
- Eliminates all 3-anchor cycling logic in uwb.cpp and most of the trilateration in fusion.cpp
- Firmware ships factory-flashed; ST-Link only needed if updating module firmware (repo has default `Project_Anchor_v1.0.hex` / `Project_Tag_v1.0.hex`)
- Plan: install on `main` first to validate hardware before folding into `ros2-hal`

**Hardware validated end-to-end (bench test, 2026-07-02).** Anchor talks over UART1 (`TXD1`/`RXD1` header pins, 115200 baud) — not native USB CDC. Two report modes:
- Default JSON: `"JS"` + 4 hex-digit length + JSON payload, e.g. `{"TWR":{"a16":"E1AE","D":50,"Xcm":19,"Ycm":48,...}}` — bearing = `atan2(Xcm, Ycm)`.
- Binary "carfollow" mode (`USER_CMD 1` + `save`): fixed 31-byte frame, `0x2A` header, length byte, payload (`sn`, `addr16`, `angle` int32, `distance` int32 cm, plus power/accel fields), XOR checksum, `0x23` footer. Pre-computed `angle`/`distance` as plain ints — no parsing math needed on the ESP32 side.

**Pairing required before ranging starts** (once per tag, persists across power cycles): anchor auto-discovers the tag over UWB and emits an unsolicited `"NewTag":"<64-bit hex id>"` JSON message; host must reply `addtag <id64> <addr16> 0001 64 00` then `save` to bind it into the anchor's known-tag list (`fastrate=1`=10Hz, `useIMU=0` — matches Makerfabs' own Windows GUI defaults).

**Gotcha hit during validation:** `addtag` initially failed with `error function` (handler returned NULL) even though the known-tag list wasn't full (1/9 slots used) — root cause never fully pinned down in source, but a factory reset (`RTOKEN` command) cleared whatever stale/corrupted flash state was blocking it. If `addtag` fails on a fresh board, try `RTOKEN` first.

**Known limitation — AoA field of view (recorded 2026-07-04):** bearing is only reliable
within roughly ±60°; past ~90° the anchor cannot distinguish front from back (a single
dual-antenna pair has inherent front/back ambiguity — the same problem the old third RYUW122
anchor existed to resolve). Bench signature: with the tag outside the cone, reported angle
pins at exactly ±90 with no sample-to-sample jitter — a detectable clamp, not a plausible
measurement. Implication: a tag behind the car produces confidently wrong bearings; naive
bearing-following can steer on a false assumption. Mitigation direction: see "Follow-me as
waypoint planning with recovery" in brainstorming below.

Bench test firmware lives in `follow-me-car-esp32` on `main` (does not touch the real car firmware — isolated via `build_src_filter` + separate `lib_deps`):
- `src/dw3000_test.cpp` / `pio run -e dw3000-test` — automated: switches anchor to binary mode, auto-pairs on `NewTag`, logs parsed frames.
- `src/dw3000_bridge.cpp` / `pio run -e dw3000-bridge` — transparent UART passthrough for manual command/response testing (typed into `pio device monitor`, or driven directly via a pyserial script for fast iteration without rebuilding).

### Magnetic encoder — replaces hall effect RPM sensor
- Module TBD (likely AS5600 or AS5048)
- Higher resolution odometry → better dead reckoning accuracy
- Contained change: rewrites rpm.cpp driver only

---

## Navigation Modes / Mission Profiles

### Mode 1 — Single car, dead reckoning + UWB follow-me (current plan)
- Follow-me: UWB AoA bearing + distance, camera fusion, PID control
- Commanded nav: waypoint missions using IMU yaw + RPM odometry
- Nav2-compatible action interfaces

### Mode 2 — Single car, camera-based person following
- Primary sensor: camera with person detection algorithm (no wearable required)
- Servo-actuated camera pan to keep person in frame
- Camera decision (2026-07-04): **add a camera attached directly to the Pi** rather than
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
- **Follow-me as waypoint planning with recovery (2026-07-04)** — response to the AoA ±60°
  FOV limitation above. Instead of steering directly on the instantaneous UWB bearing,
  convert confident tag fixes into waypoints in the `odom` frame and follow those. Anomalous
  UWB readings (angle clamped at ±90, bearing jump inconsistent with dead-reckoned motion,
  fusion uncertainty spike) then trigger a *recovery plan* rather than steering on a false
  tag location. Fits naturally once `NavigateToPose` exists: follow-me becomes continuous
  goal-updating on the same nav machinery as waypoint missions, unifying Mode 1 with the
  commanded-nav system.

  Sketched recovery behavior (2026-07-04):
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

### Open questions
- What connector does the LiPo use? (needed to order battery splitter)
- Should the Pi host its own WiFi AP (like the ESP32 does) or connect to home network? Tradeoff: AP mode works anywhere, home network mode gives internet access on the Pi.
- ros2_control controller vs standalone node for PID — worth deciding before Phase 7
- How to preserve the RYUW122 3-anchor trilateration code once `uwb.cpp`/`fusion.cpp` are rewritten for DW3000 — discussed tag + branch + `#ifdef`/build-env approach early on but never implemented; needs a decision before starting the rewrite (see chat 2026-07-02)
