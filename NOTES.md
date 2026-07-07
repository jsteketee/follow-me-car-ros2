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
- [ ] Hardware pre-flight: on-car Pi power (Pololu or power bank), USB A→C cable, anchor 5V fix (optional for demo)

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
- Hardware options:
  - **OAK-D Lite** (~$150): onboard Myriad X neural inference chip, offloads detection from Pi, also provides stereo depth (replaces UWB distance)
  - **Pi Camera Module 3 + Pi inference**: YOLOv8n at reduced resolution (~10-15 fps), cheaper but Pi does all compute
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
- Web UI on Pi for sending waypoint missions from a phone
- Record and replay a driven path as a waypoint mission
- Multi-tag support — follow one of several tagged people
- Return-to-home behavior when tag is lost for too long

### Open questions
- What connector does the LiPo use? (needed to order battery splitter)
- Should the Pi host its own WiFi AP (like the ESP32 does) or connect to home network? Tradeoff: AP mode works anywhere, home network mode gives internet access on the Pi.
- ros2_control controller vs standalone node for PID — worth deciding before Phase 7
- How to preserve the RYUW122 3-anchor trilateration code once `uwb.cpp`/`fusion.cpp` are rewritten for DW3000 — discussed tag + branch + `#ifdef`/build-env approach early on but never implemented; needs a decision before starting the rewrite (see chat 2026-07-02)
