# CLAUDE.md

This file provides guidance to Claude Code when working in this repository.

## Project Overview

ROS2 stack for a follow-me RC car. Runs on a Raspberry Pi 4B mounted on the car. Communicates with an ESP32-S3 HAL over USB serial. The ESP32 handles all hardware I/O (UWB ranging, IMU, camera, RPM, servo PWM); this repo contains all navigation and control logic.

Companion ESP32 firmware: `follow-me-car-esp32` repo, `ros2-hal` branch.

## Coding Rules

- Never include Claude attribution in git commit messages.
- Python nodes follow ROS2 naming conventions: `snake_case` for files, nodes, topics, and parameters.
- C++ packages (hardware interface, controllers) follow ROS2/ament conventions.

## Serial Protocol (ESP32 ↔ Pi)

**Incoming from ESP32 (50 Hz target, newline-delimited JSON):**
```json
{"uwb_dist":183.2,"uwb_bearing":-12.4,"yaw":23.4,"pitch":0.1,"roll":-0.3,"speed":1.82,"odo":4821.3,"enc_speed":1.79,"cam_found":1,"cam_x":0.23,"cam_y":0.11}
```
`uwb_dist` (cm) + `uwb_bearing` (deg) come directly from the DW3000 AoA anchor. `speed`/`odo`
are hall-effect derived; `enc_speed` is the AS5600 encoder (for Pi-side cogging detection —
exact encoder field set is pending that decision). Frames carry the ESP32 timestamp so the
Pi computes dt from device time, not arrival time.

**Outgoing to ESP32 (on demand, newline-delimited JSON):**
```json
{"target_speed":1.8,"target_angle":-12.4}
```
Setpoints feed the ESP32's existing tuned PID loops (decided 2026-07-04 — see PROJECT_PLAN.md
"Control Loop Placement"). The protocol reserves a raw-actuator mode
(`{"throttle":0.31,"steering":-0.18}`) for the post-interview migration of the loops to the Pi.
ESP32 applies a cmd-timeout failsafe: neutral throttle if no command arrives within the timeout.

## Package Structure

```
src/
├── follow_me_interfaces/     — custom message + action definitions
├── follow_me_hardware/       — ros2_control hardware interface (C++ plugin)
├── follow_me_nodes/          — Python nodes: fusion, dead reckoning, nav, visualization
└── follow_me_bringup/        — launch files + YAML parameter configs
```

## Build & Run

```bash
# Build
cd ~/follow-me-car-ros2
colcon build --symlink-install

# Source
source install/setup.bash

# Launch everything
ros2 launch follow_me_bringup follow_me.launch.py
```

## Key Topics

| Topic | Type | Direction |
|-------|------|-----------|
| `/uwb/reading` | `follow_me_interfaces/UWBReading` | ESP32 → ROS2 |
| `/imu/data` | `sensor_msgs/Imu` | ESP32 → ROS2 |
| `/camera/blob` | `follow_me_interfaces/CameraBlob` | ESP32 → ROS2 |
| `/follow_me/pose` | `follow_me_interfaces/FusedPose` | fusion node output |
| `/odometry` | `nav_msgs/Odometry` | dead reckoning node output |
| `/cmd_vel` | `geometry_msgs/Twist` | controller → hardware interface |

## Actions

| Action | Type | Description |
|--------|------|-------------|
| `/follow_me` | `follow_me_interfaces/FollowMe` | Start/stop autonomous following |
| `/navigate_to_pose` | `nav2_msgs/NavigateToPose` | Dead reckoning single goal |
| `/follow_waypoints` | `nav2_msgs/FollowWaypoints` | Dead reckoning waypoint mission |
