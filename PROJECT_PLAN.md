# Follow Me Car — ROS2 Project Plan

## Purpose

The durable reference for how this project is meant to work: goals, architecture, serial-link
behavior, phase plan, and hardware roster. Authoritative for specs — the design and the plan,
not their live status or change history (NOTES.md). The field-level wire interface (every
telemetry/command field, its units, sentinels, and conversions) is specified in **`interface.md`**,
the SOT for the ESP32 ↔ Pi contract.

## Goals

1. **Follow-me mode** — car autonomously follows the UWB tag, implemented as ROS2 nodes on the Pi.
2. **Dead reckoning commanded nav** — send the car a heading + distance, or a sequence of waypoints; executed using IMU yaw + RPM odometry. No map or LIDAR required.
3. **Nav2-compatible interfaces** — implement standard `nav2_msgs/NavigateToPose` and `nav2_msgs/FollowWaypoints` action servers. Compatible with Nav2 if a LIDAR/map is added later.
4. **(Stretch) Web waypoint canvas** — custom browser UI (rosbridge + roslibjs) for dropping waypoints onto a map view. The same interaction exists in the standard stack first (RViz/Foxglove "Goal Pose" click → `NavigateToPose`), so this is later polish, not core.

## Architecture

```
┌──────────────────────────────────┐   USB serial   ┌──────────────────────────────────────┐
│         ESP32-S3 (HAL)           │ ◄────────────► │         Raspberry Pi 4B              │
│                                  │                │                                      │
│  UWB AoA (DW3000: dist+bearing)  │  sensor JSON → │  ros2_control hardware interface     │
│  IMU (BNO085)                    │  ← cmd JSON    │  fusion node (tag bearing filter)    │
│  Speed fusion + cog detection    │                │                                      │
│  RPM hall-effect sensor          │                │  dead reckoning pose estimator       │
│  AS5600 encoder (cog detection)  │                │  follow-me setpoint generator        │
│  ESC + steering servo PWM        │                │  nav action servers                  │
│  Speed + heading PID loops       │                │  RViz/Foxglove visualization         │
│  Serial framing + cmd-timeout    │                │                                      │
│    failsafe                      │                │                                      │
└──────────────────────────────────┘                └──────────────────────────────────────┘
```

ESP32 repo: `follow-me-car-esp32`, branch `ros2-hal`.

## Serial Link (ESP32 ↔ Pi)

The field-level wire contract — every telemetry/command field, its type, units, sign,
sentinels, and the mph/cm/deg → SI conversions — lives in **`interface.md`** (the SOT for the
wire format). This section covers link *behavior*, not the field schema.

**Command stream:** the bridge re-sends the current absolute setpoint at a fixed 20 Hz even when
unchanged — the stream is the heartbeat, and idempotent absolute frames are why no acks/checksums
are needed (USB-CDC has per-packet CRC). Staleness is silence: the bridge stops sending when the
latched `cmd_drive` is older than 500 ms.

**Failsafe:** no valid frame for 300 ms → neutral throttle only; steering holds the last commanded
position (centering mid-turn on a comms blip would plow the car straight). STOPPED latches both
axes neutral until a human re-arms.

**Control modes** (ESP32-internal, reported in telemetry `mode`; **never set by an outside actor** —
the ESP32 infers the mode from command-frame shape, or drops into STOPPED on its own): `SETPOINT`
(setpoint frames → onboard PIDs; the boot default), `DIRECT` (raw-actuator frames, PIDs bypassed;
inferred from frame shape), `STOPPED` (onboard anomaly-detection failsafe, latches). The Pi neither
sends a mode nor needs to trigger STOPPED. Distinct from the Pi-side `nav_mode`, which selects the
active Pi controller.

**Reboot recovery:** a backward jump in the ESP32 `ts` marks a reboot. The bridge logs it as an
ERROR, halts TX until a fresh command (no auto-resume into a stale setpoint), and stitches the
odometer continuous so `wheel/state.distance` doesn't reset. Yaw is compass-absolute, so the
`odom` frame — and everything expressed in it, including the goal — is preserved; `pose_estimator`
keeps integrating from the pre-reboot pose. Motion during the blackout is unmeasured (the held
pose is stale by that drift), acceptable since the car sits un-throttled while rebooting.

## Control Loop Placement — heading + speed setpoints, minimal ESP32 diff

**Speed:** the tuned speed PID stays on the ESP32 — it's the one latency-sensitive loop
(ESC deadband, cogging, stiction at low speed). The Pi commands `target_speed`; no control
retuning needed. Optional later migration to the Pi via the raw-actuator mode, and only if
Pi-side control performs — if it underperforms, the loop stays on the ESP32 permanently and
that's fine.

**Steering:** the tuned steering PID also stays on the ESP32 — only its **error source**
changes with the nav mode. In standalone FOLLOW_ME it regulated the tag-relative
`fused_angle` to zero. In SETPOINT (then REMOTE) it regulates the wrapped heading error
`(imu.yaw − target_heading)` to zero. The two are structurally identical — the
tag-relative angle *is* a heading error (`yaw − bearing_to_tag`, wrapped) — so the tuned
gains transferred directly and the firmware diff was a few lines in `control.cpp`'s mode
switch. (2026-07-14: FOLLOW_ME is deleted from the firmware entirely — the Pi owns follow
logic; its speed-interpolation code is readable at esp32 repo commit `075ab58`.
2026-07-16: the UWB tag Kalman — bearing filter, distance dead reckoning, uncertainty
tracking — is deleted too; the wire carries raw `uwb_*` only and Phase 5 owns tag
filtering. The fused speed estimate stays on the ESP32 permanently as `speed`.)

Decision log (2026-07-12, two revisions same day):
- (a) Steering-as-direct-position on the wire was adopted first, then superseded by (b).
  Direct position survives as the reserved raw-actuator mode and the eventual pure-HAL end
  state.
- (b) **Heading-setpoint interface** adopted to keep the phase-1 ESP32 change minimal: reuse
  the tuned steering PID with a swapped error source instead of building a Pi-side steering
  controller before anything drives. Accepted costs: heading-loop tuning iterates by
  firmware reflash rather than Pi-side parameter change, and stick-style teleop maps
  awkwardly onto heading commands (workable as heading-nudge). The raw-actuator mode is the
  migration path if/when the loops move up.
- A cascaded ESP32 heading controller (heading → yaw *rate* → servo) remains rejected — a
  yaw-rate inner PID has speed-dependent plant gain (ω ≈ v·tan(δ)/L, zero authority at
  v = 0). The adopted design is a single heading PID, no inner rate loop.

Latency reference: wire latency over USB-CDC is negligible (~1-2ms); the real terms are
command rate (20 Hz = up to 50ms setpoint staleness; telemetry already ships at 50 Hz) and
Linux scheduling jitter (~1-10ms) — both comfortably irrelevant while the PIDs close on the
ESP32 at 50 Hz and the Pi only moves setpoints.

Permanent placement regardless of migration:
- **Cmd-timeout failsafe** (neutral throttle on serial loss; steering holds the last
  commanded heading — revised 2026-07-13) → ESP32, non-negotiable.
- **Actuator conditioning** (deadband, trim, clamp, smoothing) → ESP32 (`actuators.cpp`).
- **Speed fusion + cogging detection** → ESP32, permanently (decided 2026-07-16): the Pi
  does no speed-sensor fusion and treats telemetry `speed`/`cogging` as authoritative.
- **Heading + speed PIDs** → ESP32 for now; migration to the Pi (via raw-actuator mode) is
  optional, later, and only if it performs.

## Hardware

Main components only — power distribution and wiring not tracked here.

| Component | Role | Status |
|---|---|---|
| Raspberry Pi 4B 4GB | runs all ROS2 nodes | ✅ Ubuntu 24.04 + ROS2 Jazzy, SSH verified |
| ESP32-S3 | HAL firmware (`ros2-hal` branch) | ✅ on car |
| Makerfabs MaUWB AOA kit (DW3000) | distance + bearing to tag | ✅ installed & validated |
| Hall-effect sensor | RPM / speed | ✅ |
| AS5600 encoder (I2C) | cogging detection | ✅ installed & validated |
| Pan servo (UWB anchor mount, GPIO 6) | aims the DW3000 anchor, ±55° | ✅ installed & calibrated 2026-07-14 |
| BNO085 IMU (I2C) | yaw for dead reckoning + fusion | ✅ |
| OV2640 on XIAO ESP32-S3 (I2C) | blob camera | ❌ removed from firmware 2026-07-13 — not planned (Mode 2's Pi-direct camera is a separate future decision) |
| SSD1306 OLED | on-car display | ✅ |
| Open-frame RC chassis + 2S LiPo | vehicle | ✅ |

## ROS2 Skills Showcased

- `ros2_control` hardware interface (C++ plugin)
- Custom ros2_control controller
- Custom message and action types
- Action servers (Nav2-compatible nav + follow-me)
- Sensor fusion node (Kalman filter on absolute compass bearing)
- Dead reckoning pose estimator (IMU + RPM → `nav_msgs/Odometry` + TF2)
- TF2 transforms (`odom` → `base_link`)
- RViz/Foxglove visualization
- Parameter YAML configuration
- Launch files
- rosbag2 logging

## Implementation Phases

Loosely sequential — the order is a guide, not a commitment; priorities get decided as the
work progresses. ✅ marks what's built (the single source for phase status).

### Phase 1 — Hardware setup ✅
- Flash Pi with Ubuntu 24.04, install ROS2 Jazzy
- Connect Pi to ESP32 via USB serial
- Verify serial communication (minicom / Python script)

### Phase 2 — ESP32 HAL firmware
- Add `serial_hal.cpp` telemetry stream (50 Hz sensor JSON out) — strip nothing, standalone
  modes keep working
- ✅ Accept command frames (`target_speed`, `target_heading`) + cmd-timeout failsafe — the
  steering PID's error source swaps from tag bearing to wrapped heading error in the
  Pi-commanded mode (built + bench-validated 2026-07-13: `NavMode::REMOTE`, non-blocking
  RX parser with strict validation, boot-yaw heading hold, throttle-only failsafe).
  Went further than "everything else stays as-is": `DEFAULT_NAV_MODE` is now REMOTE,
  FOLLOW_ME's onboard control block is commented out, and the camera module was removed
  from the firmware entirely
- ✅ (2026-07-16) Strip `fusion.cpp`'s tag Kalman: `fused_*` telemetry is gone; the wire
  carries raw `uwb_*` only and Phase 5 owns tag filtering. The fused speed estimate
  (`speed` on the wire, the throttle PID's feedback) stays on the ESP32 permanently —
  no Pi-side speed fusion planned. The speed PID itself may stay permanently
- Keep WiFi + dashboard for side-by-side debugging during transition

### Phase 3 — ROS2 bridge node
- Python node: read serial frames, publish raw sensor topics
- Confirm data in `ros2 topic echo` and the visualizer
- Also write command frames to serial from the subscribed command topic (`cmd_drive` —
  see Key topics), converting odom-frame heading (rad) → device compass degrees on write

### Phase 4 — Custom interfaces package
- `follow_me_interfaces`: `UwbRaw.msg` (tag fix stream), `WheelState.msg`,
  `CommandStatus.msg`, `ActuatorStatus.msg`, `DriveCommand.msg`
- `FusedTagPose.msg` + `CoggingStatus.msg` removed 2026-07-16 with the telemetry
  slim-down (ESP32 tag Kalman deleted; cogging folded into `WheelState`).
- `UWBReading.msg` + `CameraBlob.msg` were **removed** earlier to keep the surface
  minimal. `UwbRaw.msg` supersedes `UWBReading` as the Phase 5 fusion input; `CameraBlob`
  is moot (camera removed 2026-07-13). Do not re-add either.
- `FollowMe.action`

### Phase 5 — Fusion node
- DW3000 provides bearing directly — no trilateration needed. Fusion filters UWB bearing
  on absolute compass bearing (port the Kalman scheme from the deleted `fusion.cpp`,
  readable in esp32 repo history), and tracks uncertainty
- Subscribes: `uwb/raw` (`UwbRaw`), `imu/data`
- Publishes: its own tag estimate topic (message defined in this phase — the old
  `FusedTagPose.msg` shape is the starting point)

### Phase 6 — Dead reckoning pose estimator ✅
- Integrates IMU yaw + wheel distance into 2D pose in `odom` frame
- Publishes: `/odom` (`nav_msgs/Odometry`), TF2 `odom → base_link`
- Subscribes: `/imu/data` (heading), `/wheel/state` (accumulated metres in `distance`)
- `odom` starts at identity (initial yaw subtracted).

### Phase 7 — ros2_control hardware interface
- C++ `SystemInterface` plugin replaces Python bridge node
- `read()`: parse serial frame → fill state interfaces
- `write()`: serialize command interfaces → send to ESP32
- Command interfaces mirror the wire contract: velocity (traction → `target_speed`) + a
  custom heading-setpoint interface; switches to the standard Ackermann layout
  (velocity + position) if/when the loops migrate via the raw-actuator mode

### Phase 8 — Follow-me controller
- Standalone node (or ros2_control controller): a *setpoint generator* — both PIDs stay on
  the ESP32, so no Pi-side control loop is needed for this phase
- Heading: absolute tag bearing from the Phase 5 estimate (or directly from `uwb/raw`
  chained through TF: the tag's odom-frame position already composes yaw + pan + bearing)
  → publish as the `cmd_drive` heading setpoint
- Speed: distance-interpolated `target_speed` (port the min/max-speed-vs-distance logic
  from the ESP32's `control.cpp` FOLLOW_ME case — deleted from the firmware 2026-07-14;
  read it at esp32 repo commit `075ab58`, where it was last active)

### Phase 9 — Follow-me action server
- `/follow_me` action: goal = start/stop, feedback = distance + angle + uncertainty, result = reason stopped

### Phase 10 — Dead reckoning nav action servers
- `/navigate_to_pose` (`nav2_msgs/NavigateToPose`): dead reckoning single goal
- `/follow_waypoints` (`nav2_msgs/FollowWaypoints`): ordered waypoint missions

### Phase 11 — Visualization + launch files
- Foxglove/RViz config: heading arrow, path trace, sensor status markers
- Single launch file starts everything
- rosbag2 recording in launch file
- **Visualization host:** ROS2/RViz on macOS is effectively unsupported — use Foxglove Studio
  on the Mac connected to `foxglove_bridge` on the Pi (native Mac app, no Mac ROS2 install,
  supports click-to-publish goal poses)

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
| `/uwb/raw` | `follow_me_interfaces/UwbRaw` | ESP32 → ROS2 (tag range/bearing; Phase 5 fusion input) |
| `/imu/data` | `sensor_msgs/Imu` | ESP32 → ROS2 |
| `/tag/pose` | *(Phase 5 — message defined then)* | fusion node output (filtered bearing/dist to tag) |
| `/odom` | `nav_msgs/Odometry` | dead reckoning node output |
| `/wheel/state` | `follow_me_interfaces/WheelState` | fused speed + odometer + cogging flag, one stamped message |
| `/command/status` | `follow_me_interfaces/CommandStatus` | ESP32 control mode (`command_mode`) + accepted-command echo |
| `/actuator/status` | `follow_me_interfaces/ActuatorStatus` | live actuator outputs (throttle/steering/pan) |
| `/cmd_drive` | `follow_me_interfaces/DriveCommand` | controller/nav → bridge (later: hardware interface) |
| `/nav_mode` | `follow_me_interfaces/NavMode` | mode_manager → all (latched/transient_local): active Pi-side nav policy |
| `/sensor_health` | `follow_me_interfaces/SensorHealth` | ESP32 → ROS2 (~1-2 Hz): per-sensor update rates from `{"type":"health"}` frames, plus link-health/throughput stats (telem rate, seq gaps, TX drops, inter-frame jitter) |

Field-level schema for the ESP32-sourced topics — units, types, sentinels, wire conversions —
is in **`interface.md`** (the SOT); this table is a topic directory, not the field spec.

### Services
| Service | Type | Description |
|---------|------|-------------|
| `/set_nav_mode` | `follow_me_interfaces/SetNavMode` | request a nav_mode switch; mode_manager gates entry (unknown mode / car conditions) and answers accepted + reason. Callable from the web dashboard via foxglove_bridge. |

**nav_mode vs command_mode (2026-07-20).** Two independent mode axes: `command_mode` is the
**ESP32's** command interface (`SETPOINT`/`DIRECT`/`STOPPED`, ESP32-owned — inferred from
command-frame shape or set by the onboard anomaly-detection failsafe, never by the Pi; reported
in telemetry, read-only) and stays in SETPOINT for normal operation. `nav_mode` is the
**Pi-side** navigation policy (`follow`, `stopped`, future `waypoint`, …), owned by
`mode_manager`, boots to `stopped` (car idle until a human selects a driving mode). Controller nodes subscribe and act only when the mode
they implement (their `active_mode` param) is active — multiple follow policies become
sibling nodes with different `active_mode` values. On losing the mode, a controller sends
one zero-speed command then goes cmd-silent (bridge 500 ms staleness gate + ESP32 300 ms
failsafe are the backstops).

**Topic layout convention.** Fields co-sampled in one wire frame bundle into a single
stamped message per subsystem (`wheel/state`, `uwb/raw`): one sample, one stamp.
ESP32-side derivations that live there permanently (fused `speed`, `cogging`) ride the
bundle — there is no parallel Pi estimate to isolate them from. Pi-side estimators
(Phase 5 tag fusion) publish their own topics so they can be compared against their
inputs and swapped without touching the wire layer.

`/cmd_drive` is a custom stamped message — `{header, speed (m/s), heading (rad, odom
frame)}` — because no standard message carries a heading setpoint: Twist's `angular.z` is a
yaw *rate*, and Ackermann's `steering_angle` is a wheel angle, neither of which matches the
wire contract (`target_heading`). The header stamp lets the bridge drop stale commands. The
earlier `/cmd_vel` (Twist) and `/cmd_ackermann` plans are superseded; if the loops later
migrate via the raw-actuator mode, Ackermann becomes the natural fit again.

### Actions
| Action | Type | Description |
|--------|------|-------------|
| `/follow_me` | `follow_me_interfaces/FollowMe` | Start/stop autonomous following |
| `/navigate_to_pose` | `nav2_msgs/NavigateToPose` | Dead reckoning single goal |
| `/follow_waypoints` | `nav2_msgs/FollowWaypoints` | Dead reckoning waypoint mission |
