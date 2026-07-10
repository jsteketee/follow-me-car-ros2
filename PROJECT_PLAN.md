# Follow Me Car — ROS2 Project Plan

## Goals

1. **Follow-me mode** — car autonomously follows the UWB tag (+ camera fusion), implemented as ROS2 nodes on the Pi.
2. **Dead reckoning commanded nav** — send the car a heading + distance, or a sequence of waypoints; executed using IMU yaw + RPM odometry. No map or LIDAR required.
3. **Nav2-compatible interfaces** — implement standard `nav2_msgs/NavigateToPose` and `nav2_msgs/FollowWaypoints` action servers. Compatible with Nav2 if a LIDAR/map is added later.
4. **(Stretch) Web waypoint canvas** — custom browser UI (rosbridge + roslibjs) for dropping waypoints onto a map view. The same interaction exists in the standard stack first (RViz/Foxglove "Goal Pose" click → `NavigateToPose`), so this is post-interview polish, not core.

## Architecture

```
┌──────────────────────────────────┐   USB serial   ┌──────────────────────────────────────┐
│         ESP32-S3 (HAL)           │ ◄────────────► │         Raspberry Pi 4B              │
│                                  │                │                                      │
│  UWB AoA (DW3000: dist+bearing)  │  sensor JSON → │  ros2_control hardware interface     │
│  IMU (BNO085)                    │  ← cmd JSON    │  fusion node (bearing blend +        │
│  Camera (OV2640 blob via I2C)    │                │    cogging detection candidate)      │
│  RPM hall-effect sensor          │                │  dead reckoning pose estimator       │
│  AS5600 encoder (cog detection)  │                │  follow-me controller (PID)          │
│  ESC + steering servo PWM        │                │  nav action servers                  │
│  Serial framing + cmd-timeout    │                │  RViz/Foxglove visualization         │
│    failsafe                      │                │                                      │
└──────────────────────────────────┘                └──────────────────────────────────────┘
```

ESP32 repo: `follow-me-car-esp32`, branch `ros2-hal`.

## Serial Protocol (ESP32 ↔ Pi)

**Incoming from ESP32 (50 Hz target, newline-delimited JSON):**
```json
{"ts":12345,"uwb_dist":183.2,"uwb_bearing":-12.4,"yaw":23.4,"pitch":0.1,"roll":-0.3,"speed":1.82,"odo":4821.3,"enc_speed":1.79,"cogging":0,"cam_found":1,"cam_x":0.23,"cam_y":0.11,"fused_angle":5.2,"fused_dist":185.0,"fused_unc":17.3}
```
`ts` (ms) is the ESP32 device timestamp — Pi computes dt from device time, not arrival time.
The bridge maps `ts` into the ROS clock via a constant offset captured on the first frame
(re-captured if `ts` jumps backwards, i.e. ESP32 reboot). Raw device uptime in a header would
stamp messages in 1970 and tf2 would silently drop the transforms; the offset cancels under
subtraction, so downstream dt is still exact device time.
`uwb_dist` (cm) + `uwb_bearing` (deg) are Kalman-filtered DW3000 AoA readings; -1 if no fix.

`odo` is the accumulated odometer in **cm** (verified on hardware 2026-07-09).

**Units:** the ESP32 speaks mph / cm / degrees. The bridge node is the boundary and converts
everything to SI (REP-103: metres, m/s, radians) before publishing, so every ROS2 topic
downstream is SI by construction and needs no scale factors. Note `fused_unc` is a *variance*
(deg²), so it converts by the **square** of the degree→radian factor.
`speed`/`odo` are hall-effect derived. `enc_speed` is AS5600 encoder EMA velocity (mph, forward
positive) for Pi-side cogging detection; `cogging` is the ESP32 latching cogging flag (0/1).
`fused_angle`/`fused_dist`/`fused_unc` are the ESP32's Kalman-fused bearing, distance, and
bearing variance — streamed because Pi-side fusion is deferred post-interview.

**Outgoing to ESP32 (on demand, newline-delimited JSON):**
```json
{"target_speed":1.8,"target_angle":-12.4}
```
Setpoints feed the ESP32's existing tuned PID loops (see "Control Loop Placement" below). The
protocol reserves a raw-actuator mode (`{"throttle":0.31,"steering":-0.18}`) for the
post-interview migration of the loops to the Pi. ESP32 applies a cmd-timeout failsafe: neutral
throttle if no command arrives within the timeout.

## Control Loop Placement — DECIDED (2026-07-04): setpoints first, migrate later

**Before the interview:** both PID loops (speed + steering) stay on the ESP32, already tuned
and working. The Pi commands setpoints (`target_speed`, `target_angle`) — no control retuning
on the critical path. **Post-interview:** migrate loops up to the Pi as the ros2_control
custom-controller showcase; the cmd protocol's mode field makes that a config change, not a
re-architecture.

Latency reference (why either placement works): wire latency over USB-CDC is negligible
(~1-2ms); the real terms are frame rate (20 Hz = up to 50ms staleness each way; 50 Hz is the
target) and Linux scheduling jitter (~1-10ms, hurts PID derivative terms — mitigate by
timestamping frames ESP32-side so the Pi computes dt from device time).

Permanent placement regardless of migration:
- **Cmd-timeout failsafe** (neutral on serial loss) → ESP32, non-negotiable.
- **Actuator conditioning** (deadband, trim, clamp, smoothing) → ESP32 (`actuators.cpp`).
- **Cogging detection** → Pi fusion node (candidate; see Phase 5). If cogging *recovery*
  needs fast throttle intervention, that reflex may stay ESP32-side even with detection on
  the Pi.
- **Steering PID** (post-interview migration order: first) — plant time constant 300ms+ at
  2.5 mph; Pi-side latency is invisible.
- **Speed PID** (migration order: second, and only if it performs) — the one latency-sensitive
  loop (ESC deadband, cogging, stiction at low speed). If Pi-side control underperforms,
  it stays on the ESP32 permanently and that's fine.

## Interview Critical Path (2 focused days)

Objective: maximum hands-on ROS2 surface area before the interview, robot driveable at every
step, zero control retuning.

### Day 1 — data flowing up ✅
1. ESP32 (`ros2-hal`): add `serial_hal.cpp` telemetry stream (50 Hz JSON over USB serial).
   **Strip nothing** — standalone modes keep working.
2. Pi: `follow_me_interfaces` custom messages + Python bridge node publishing
   `/imu/data`, `/tag/pose`, `/wheel/speed`, `/wheel/distance`.
3. Dead-reckoning pose estimator node → `nav_msgs/Odometry` + TF2 `odom → base_link`.
4. Visualization + rosbag2: watch the pose estimate track while pushing the car by hand;
   record and replay a bag.

Banked by end of Day 1: packages, colcon, nodes, custom msgs, TF2, odometry, visualization, rosbag2.

### Day 2 — commands flowing down
5. ESP32: accept `{"target_speed", "target_angle"}` setpoint commands feeding the existing
   tuned loops + cmd-timeout failsafe to neutral.
6. Pi: `NavigateToPose` action server — computes bearing/distance to goal from the
   dead-reckoned pose, streams setpoints, feedback = remaining distance.
7. **Demo: click Goal Pose on the canvas → car drives to the waypoint.**
8. If time: `FollowWaypoints` multi-goal missions; single bringup launch file.

### Consciously deferred (post-interview)
- Fusion node on the Pi (ESP32 fusion keeps running; its output is streamed)
- ros2_control C++ hardware interface + custom controller (loop migration)
- Follow-me action server
- Custom web waypoint canvas (Goal 4) — RViz/Foxglove Goal Pose covers the demo interaction
- Stripping fusion/nav/control out of the ESP32 firmware

### Critical-path risks
- **Visualization host**: ROS2/RViz on macOS is effectively unsupported — do not burn hours
  there. Recommended: Foxglove Studio on the Mac connected to `foxglove_bridge` on the Pi
  (native Mac app, no Mac ROS2 install, supports click-to-publish goal poses). Decide first
  thing Day 1.

## Hardware

Main components only — power distribution and wiring not tracked here.

| Component | Role | Status |
|---|---|---|
| Raspberry Pi 4B 4GB | runs all ROS2 nodes | ✅ Ubuntu 24.04 + ROS2 Jazzy, SSH verified |
| ESP32-S3 | HAL firmware (`ros2-hal` branch) | ✅ on car |
| Makerfabs MaUWB AOA kit (DW3000) | distance + bearing to tag | ✅ installed & validated |
| Hall-effect sensor | RPM / speed | ✅ |
| AS5600 encoder (I2C) | cogging detection | ✅ installed & validated |
| BNO085 IMU (I2C) | yaw for dead reckoning + fusion | ✅ |
| OV2640 on XIAO ESP32-S3 (I2C) | blob camera (optional, deferred) | ⚠️ confirm status |
| SSD1306 OLED | on-car display | ✅ |
| Open-frame RC chassis + 2S LiPo | vehicle | ✅ |

## ROS2 Skills Showcased

- `ros2_control` hardware interface (C++ plugin) *(post-interview)*
- Custom ros2_control controller *(post-interview)*
- Custom message and action types
- Action servers (Nav2-compatible nav + follow-me)
- Sensor fusion node (Kalman filter on absolute compass bearing) *(post-interview)*
- Dead reckoning pose estimator (IMU + RPM → `nav_msgs/Odometry` + TF2)
- TF2 transforms (`odom` → `base_link`)
- RViz/Foxglove visualization
- Parameter YAML configuration
- Launch files
- rosbag2 logging

## Implementation Phases

### Phase 1 — Hardware setup ✅
- Flash Pi with Ubuntu 24.04, install ROS2 Jazzy
- Connect Pi to ESP32 via USB serial
- Verify serial communication (minicom / Python script)

### Phase 2 — ESP32 HAL firmware [Day 1–2: additive only]
- Day 1: add `serial_hal.cpp` telemetry stream (50 Hz sensor JSON out) — strip nothing,
  standalone modes keep working
- Day 2: accept setpoint commands (`target_speed`, `target_angle`) + cmd-timeout failsafe
- Post-interview: strip `fusion.cpp`, `nav.cpp`, `control.cpp` down to a pure HAL as the
  loops migrate to the Pi
- Keep WiFi + dashboard for side-by-side debugging during transition

### Phase 3 — ROS2 bridge node [Day 1]
- Python node: read serial frames, publish raw sensor topics
- Confirm data in `ros2 topic echo` and the visualizer
- Day 2: also write setpoint commands from subscribed topic to serial

### Phase 4 — Custom interfaces package [Day 1]
- `follow_me_interfaces`: `FusedTagPose.msg`
- **Removed 2026-07-09** to keep the surface minimal; both must be **re-added for Phase 5**
  (they are the fusion node's inputs). Backups in the session scratchpad; `src/` is untracked
  in git, so there is no `git show` restore path.
  - `UWBReading.msg` (distance + bearing from DW3000 AoA) — raw UWB bearing + `valid` fix flag
    are no longer published anywhere.
  - `CameraBlob.msg` (blob found + normalized x/y) — camera hardware status is unconfirmed
    anyway (see Hardware table).
- `FollowMe.action`

### Phase 5 — Fusion node [post-interview]
- DW3000 provides bearing directly — no trilateration needed. Fusion blends UWB bearing +
  camera blob angle on absolute compass bearing (port the Kalman scheme from `fusion.cpp`),
  and tracks uncertainty
- Candidate: cogging detection moves here (compare commanded throttle vs encoder motion;
  gate speed/odometry while cogging)
- Subscribes: `/uwb/reading`, `/imu/data`, `/camera/blob`
  (**re-add `UWBReading.msg` + `CameraBlob.msg` first — both removed 2026-07-09**)
- Publishes: `/tag/pose` (`FusedTagPose`)

### Phase 6 — Dead reckoning pose estimator ✅
- Integrates IMU yaw + wheel distance into 2D pose in `odom` frame
- Publishes: `/odom` (`nav_msgs/Odometry`), TF2 `odom → base_link`
- Subscribes: `/imu/data` (heading), `/wheel/distance` (accumulated metres)
- `odom` starts at identity (initial yaw subtracted). Reverse motion is invisible —
  the odometer does not tick backwards.

### Phase 7 — ros2_control hardware interface [post-interview]
- C++ `SystemInterface` plugin replaces Python bridge node
- `read()`: parse serial frame → fill state interfaces
- `write()`: serialize command interfaces → send to ESP32
- Works with setpoint commands too (velocity command interface) — does not require the
  loop migration

### Phase 8 — Follow-me controller [post-interview — loop migration]
- ros2_control controller (or standalone node)
- Steering PID first (`fused_angle` → steering), speed PID second and only if it performs
- Ports the tuned gains from the ESP32's `control.cpp`

### Phase 9 — Follow-me action server [post-interview]
- `/follow_me` action: goal = start/stop, feedback = distance + angle + uncertainty, result = reason stopped

### Phase 10 — Dead reckoning nav action servers [Day 2]
- `/navigate_to_pose` (`nav2_msgs/NavigateToPose`): dead reckoning single goal
- `/follow_waypoints` (`nav2_msgs/FollowWaypoints`): ordered waypoint missions (if time)

### Phase 11 — Visualization + launch files [Day 1–2 partial; polish post-interview]
- Foxglove/RViz config: heading arrow, path trace, sensor status markers
- Single launch file starts everything
- rosbag2 recording in launch file

## Repository Reference

### Package structure
```
src/
├── follow_me_interfaces/     — custom message + action definitions
├── follow_me_hardware/       — ros2_control hardware interface (C++ plugin)
├── follow_me_nodes/          — Python nodes: fusion, dead reckoning, nav, visualization
└── follow_me_bringup/        — launch files + YAML parameter configs
```

### Build & run
See [cheat.md](./cheat.md) for the commands that actually work today.

```bash
colcon build --symlink-install
source install/setup.bash
ros2 run follow_me_nodes serial_bridge

# FUTURE — follow_me_bringup does not exist yet. Build it when there are multiple nodes
# to start together (Phase 6 estimator + Foxglove), not before.
# ros2 launch follow_me_bringup follow_me.launch.py
```

### Key topics
| Topic | Type | Direction |
|-------|------|-----------|
| `/uwb/reading` | `follow_me_interfaces/UWBReading` | ESP32 → ROS2 *(removed 2026-07-09; re-add in Phase 5)* |
| `/imu/data` | `sensor_msgs/Imu` | ESP32 → ROS2 |
| `/camera/blob` | `follow_me_interfaces/CameraBlob` | ESP32 → ROS2 *(removed 2026-07-09; re-add in Phase 5)* |
| `/tag/pose` | `follow_me_interfaces/FusedTagPose` | fusion node output (bearing/dist to tag) |
| `/odom` | `nav_msgs/Odometry` | dead reckoning node output |
| `/wheel/distance` | `std_msgs/Float32` | accumulated odometer reading, m |
| `/wheel/speed` | `std_msgs/Float32` | hall-effect speed, m/s |
| `/cmd_vel` | `geometry_msgs/Twist` | controller → hardware interface |

### Actions
| Action | Type | Description |
|--------|------|-------------|
| `/follow_me` | `follow_me_interfaces/FollowMe` | Start/stop autonomous following |
| `/navigate_to_pose` | `nav2_msgs/NavigateToPose` | Dead reckoning single goal |
| `/follow_waypoints` | `nav2_msgs/FollowWaypoints` | Dead reckoning waypoint mission |
