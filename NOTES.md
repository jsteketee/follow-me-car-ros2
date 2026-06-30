# Notes

## To Do

### Current Sprint
- [ ] Get a 5V/3A USB-C power supply for Pi desk development
- [ ] Flash Pi with Ubuntu 24.04 Server (64-bit) via Raspberry Pi Imager
- [ ] Install ROS2 Jazzy on Pi
- [ ] Verify SSH access from Mac (`ssh ubuntu@followme-pi.local`)
- [ ] Connect ESP32 to Pi via USB serial and verify communication

### Up Next
- [ ] ESP32 HAL firmware (Phase 2) — strip fusion/nav/control, add serial_hal.cpp
- [ ] ROS2 bridge node (Phase 3)

---

## Brainstorming

### Future hardware additions
- LIDAR (e.g. RPLIDAR A1) — would unlock full Nav2 with SLAM and obstacle avoidance
- Second camera for wider FOV or depth sensing

### Future software ideas
- Web UI on Pi for sending waypoint missions from a phone
- Record and replay a driven path as a waypoint mission
- Multi-tag support — follow one of several tagged people
- Return-to-home behavior when tag is lost for too long

### Open questions
- What connector does the LiPo use? (needed to order battery splitter)
- Should the Pi host its own WiFi AP (like the ESP32 does) or connect to home network? Tradeoff: AP mode works anywhere, home network mode gives internet access on the Pi.
- ros2_control controller vs standalone node for PID — worth deciding before Phase 7
