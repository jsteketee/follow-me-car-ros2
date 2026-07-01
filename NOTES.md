# Notes

## To Do

### Current Sprint
- [x] Get a 5V/3A USB-C power supply for Pi desk development
- [x] Flash Pi with Ubuntu 24.04 Server (64-bit) via Raspberry Pi Imager
- [x] Install ROS2 Jazzy on Pi (verified with `ros2 topic list`)
- [x] Verify SSH access from Mac (`ssh ubuntu@followme-pi.local`)
- [ ] Connect ESP32 to Pi via USB serial and verify communication (hold until DW3000 HAL protocol settles)

### Up Next
- [ ] Install DW3000 AoA UWB (rewrite uwb.cpp + simplify fusion.cpp on main or ros2-hal)
- [ ] Install magnetic encoder in place of hall effect RPM sensor
- [ ] ESP32 HAL firmware (Phase 2) — strip fusion/nav/control, add serial_hal.cpp
- [ ] ROS2 bridge node (Phase 3)

---

## Hardware Updates (Planned)

### DW3000 AoA UWB — replaces 3x RYUW122
- Hardware: Makerfabs [MaUWB STM32 AOA Development Kit](https://www.makerfabs.com/mauwb-stm32-aoa-development-kit.html) — anchor + tag pair, STM32F103C8T6 + Qorvo DW3000, dual antenna
- Correction: this is STM32-based, not ESP32-based as originally assumed. No onboard ESP32 co-processor.
- Controlled via AT commands over UART1 (same pattern as existing RYUW122s) — confirmed via firmware repo `Makerfabs/UWB-AOA-with-Display-STM32F103C8T6`
- Spec (from that repo's README): angle ±60°, angle error ±5°, ranging error <10cm, positioning error <10cm, coverage radius 30m@6.8Mbps
- Note: the generic `Makerfabs/MaUWB_ESP32S3-with-STM32-AT-Command` repo (range-only, no angle field in `AT+RANGE`) is a DIFFERENT product line — don't confuse the two when looking up firmware/docs
- One board on car as anchor, one as tag (carried by person or mounted on Car 2)
- Main ESP32-S3 reads anchor's UART output, aggregates into existing JSON serial frame to Pi — single serial connection preserved
- Serial protocol changes: `uwb_l/uwb_r/uwb_f` → `uwb_dist` + `uwb_bearing`
- Eliminates all 3-anchor cycling logic in uwb.cpp and most of the trilateration in fusion.cpp
- Firmware ships factory-flashed; ST-Link only needed if updating module firmware (repo has default `Project_Anchor_v1.0.hex` / `Project_Tag_v1.0.hex`)
- Plan: install on `main` first to validate hardware before folding into `ros2-hal`

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
- Exact AT command set for the AOA anchor's angle report (need to check `Makerfabs/UWB-AOA-with-Display-STM32F103C8T6` repo's AT command docs/firmware source directly — haven't confirmed the report line format yet, e.g. whether it extends `AT+RANGE` or uses a separate command)
