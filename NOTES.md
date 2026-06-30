# Notes

## To Do

### Current Sprint
- [ ] Get a 5V/3A USB-C power supply for Pi desk development
- [ ] Flash Pi with Ubuntu 24.04 Server (64-bit) via Raspberry Pi Imager
- [ ] Install ROS2 Jazzy on Pi
- [ ] Verify SSH access from Mac (`ssh ubuntu@followme-pi.local`)
- [ ] Connect ESP32 to Pi via USB serial and verify communication

### Up Next
- [ ] Install DW3000 AoA UWB (rewrite uwb.cpp + simplify fusion.cpp on main or ros2-hal)
- [ ] Install magnetic encoder in place of hall effect RPM sensor
- [ ] ESP32 HAL firmware (Phase 2) — strip fusion/nav/control, add serial_hal.cpp
- [ ] ROS2 bridge node (Phase 3)

---

## Hardware Updates (Planned)

### DW3000 AoA UWB — replaces 3x RYUW122
- 2x Makerfabs ESP32-UWB-DW3000 (dual antenna, supports PDoA/AoA)
- One board on car as UWB co-processor: runs initiator + AoA firmware, outputs dist + bearing over UART to main ESP32-S3
- One board as tag: carried by person or mounted on Car 2 (responder firmware)
- Main ESP32-S3 aggregates UWB data into existing JSON serial frame to Pi — single serial connection preserved
- Serial protocol changes: `uwb_l/uwb_r/uwb_f` → `uwb_dist` + `uwb_bearing`
- Eliminates all 3-anchor cycling logic in uwb.cpp and most of the trilateration in fusion.cpp
- Firmware library TBD: check Makerfabs example sketches first

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
- DW3000 firmware library: Makerfabs examples, Qorvo DW3_QM33_SDK, or community Arduino library?
- Which branch to install DW3000 on: main (validate hardware, simplify existing firmware) vs ros2-hal (fold into HAL transition)?
