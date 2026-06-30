# Follow Me Car — ROS2 Project Plan

## Goals

1. **Follow-me mode** — car autonomously follows the UWB tag (+ camera fusion), implemented as ROS2 nodes on the Pi.
2. **Dead reckoning commanded nav** — send the car a heading + distance, or a sequence of waypoints; executed using IMU yaw + RPM odometry. No map or LIDAR required.
3. **Nav2-compatible interfaces** — implement standard `nav2_msgs/NavigateToPose` and `nav2_msgs/FollowWaypoints` action servers. Compatible with Nav2 if a LIDAR/map is added later.

## Architecture

```
┌─────────────────────────────────┐   USB serial   ┌──────────────────────────────────────┐
│         ESP32-S3 (HAL)          │ ◄────────────► │         Raspberry Pi 4B              │
│                                 │                 │                                      │
│  UWB ranging (left/right/front) │  sensor JSON →  │  ros2_control hardware interface      │
│  IMU (BNO085)                   │  ← cmd JSON     │  fusion node (Kalman bearing filter) │
│  Camera (OV2640 blob via I2C)   │                 │  dead reckoning pose estimator       │
│  RPM hall-effect sensor         │                 │  follow-me controller (PID)          │
│  ESC + steering servo PWM       │                 │  nav action servers                  │
│  Serial framing                 │                 │  RViz visualization                  │
└─────────────────────────────────┘                 └──────────────────────────────────────┘
```

ESP32 repo: `follow-me-car-esp32`, branch `ros2-hal`.

## Hardware

- Raspberry Pi 4B 4GB
- ESP32-S3 (on car) connected via USB-C → USB-A
- Pololu D24V50F5 (5V/5A) powering Pi from 7.4V 2S LiPo
- Open-frame RC car chassis

## ROS2 Skills Showcased

- `ros2_control` hardware interface (C++ plugin)
- Custom ros2_control controller
- Custom message and action types
- Action servers (follow-me + Nav2-compatible nav)
- Sensor fusion node (Kalman filter on absolute compass bearing)
- Dead reckoning pose estimator (IMU + RPM → `nav_msgs/Odometry` + TF2)
- TF2 transforms (`odom` → `base_link`)
- RViz visualization
- Parameter YAML configuration
- Launch files
- rosbag2 logging

## Implementation Phases

### Phase 1 — Hardware setup
- Flash Pi with Ubuntu 24.04, install ROS2 Jazzy
- Connect Pi to ESP32 via USB serial
- Verify serial communication (minicom / Python script)

### Phase 2 — ESP32 HAL firmware
- Strip `fusion.cpp`, `nav.cpp`, `control.cpp` from ESP32
- Add `serial_hal.cpp`: sends sensor JSON at 20 Hz, receives throttle/steering commands
- Keep WiFi + dashboard for side-by-side debugging during transition

### Phase 3 — ROS2 bridge node
- Python node: read serial frames, publish raw sensor topics
- Confirm data in `ros2 topic echo` and RViz
- Also write throttle/steering commands from subscribed topic to serial

### Phase 4 — Custom interfaces package
- `follow_me_interfaces`: `UWBReading.msg`, `CameraBlob.msg`, `FusedPose.msg`
- `FollowMe.action`

### Phase 5 — Fusion node
- Port Kalman bearing filter from `fusion.cpp` to Python ROS2 node
- Subscribes: `/uwb/reading`, `/imu/data`, `/camera/blob`
- Publishes: `/follow_me/pose` (`FusedPose`)

### Phase 6 — Dead reckoning pose estimator
- Integrates IMU yaw + RPM odometry into 2D pose in `odom` frame
- Publishes: `nav_msgs/Odometry`, TF2 `odom → base_link`

### Phase 7 — ros2_control hardware interface
- C++ `SystemInterface` plugin replaces Python bridge node
- `read()`: parse serial frame → fill state interfaces
- `write()`: serialize command interfaces → send to ESP32

### Phase 8 — Follow-me controller
- ros2_control controller (or standalone node)
- Speed PID: `rpm_speed` → throttle
- Steering PID: `fused_angle` → steering

### Phase 9 — Follow-me action server
- `/follow_me` action: goal = start/stop, feedback = distance + angle + uncertainty, result = reason stopped

### Phase 10 — Dead reckoning nav action servers
- `/navigate_to_pose` (`nav2_msgs/NavigateToPose`): dead reckoning single goal
- `/follow_waypoints` (`nav2_msgs/FollowWaypoints`): ordered waypoint missions

### Phase 11 — Visualization + launch files
- RViz config: heading arrow, path trace, sensor status markers
- Single launch file starts everything
- rosbag2 recording in launch file
