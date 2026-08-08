# Follow Me Car — ROS2 Project Plan

## Purpose

The durable reference for how this project is meant to work: goals, architecture, serial-link
behavior, phase plan, and hardware roster. Authoritative for specs — the design and the plan,
not their live status or change history (NOTES.md). The field-level wire interface (every
telemetry/command field, its units, sentinels, and conversions) is specified in **`interface.md`**,
the SOT for the ESP32 ↔ Pi contract.

## Goals

1. **Follow-me mode** — car autonomously follows the UWB tag, as ROS2 nodes on the Pi.
   Delivered in two policies: a **simple** "steer at the tag" cut and a **complex**
   confidence-gated / recovery-capable version (see NOTES.md).
2. **Manual field teleop** — drive from the dashboard with a single auto-centering virtual
   joystick, no laptop or bench rig. Speed + heading-nudge (spec under "Dashboard Field Joystick").
3. **Waypoint missions** — drop numbered points on the dashboard canvas; the car executes them
   sequentially using IMU yaw + wheel odometry. **Custom dead-reckoning nav — not Nav2**
   (decided 2026-07-24; rationale below). No map or LIDAR required.
4. **UWB pan tracking** — the Pi keeps the DW3000 anchor aimed at the tag, dead-reckoning the aim
   through UWB dropouts. Pan *policy* on the Pi; smooth rate-limited actuation on the ESP32.
5. **Field-ready operation** — the Pi brings the whole stack up on its own (no Mac in the loop);
   the car is drivable and demoable standalone.

**Dropped: Nav2 + action-server interfaces (2026-07-24).** Nav2 assumes a costmap/global-planner
world and a `Twist` (`cmd_vel`) command surface; this is an Ackermann car whose command interface
is a *heading setpoint* with the PIDs on the ESP32. Nav2 would be bolted-on and doesn't fit the
command model, so `nav2_msgs/NavigateToPose` / `FollowWaypoints` and the `/follow_me` action are
cut. Nav is **mode-based** instead: `mode_manager` selects a continuous controller per `nav_mode`.
Waypoint missions are a custom dead-reckoning controller, not `FollowWaypoints`.

## Architecture

```
  ESP32-S3 (HAL)                    USB serial       Raspberry Pi 4B
  ----------------------------    <----------->    --------------------------------
  UWB AoA (DW3000: dist+brg)      sensor JSON ->    serial bridge (Python HAL)
  IMU (BNO085)                    <- cmd JSON       tag EKF (uwb + pan + odom -> tag)
  RPM hall + AS5600 encoder                         dead-reckoning pose estimator
  Speed fusion + cog detection                      pan-aiming policy (-> target_pan)
  ESC + steer + pan servo PWM                       mode controllers (per nav_mode):
  Speed + heading PID loops                           follow / manual / waypoint / stopped
  Serial framing + cmd failsafe                     web dashboard + Foxglove viz
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
- **UWB pan** (2026-07-24) — *split*: the aiming **policy** is on the Pi (`pan_controller`,
  from the filtered tag estimate + odom, so it can dead-reckon the aim through UWB dropouts),
  while **smooth rate-limited actuation** (`target_pan`, max slew) is on the ESP32. The Pi sends
  a pan setpoint; the ESP32 slews to it. Runs independently of `nav_mode`.

## Dashboard Field Joystick (manual mode)

A single auto-centering virtual joystick for field driving (the `manual` `nav_mode`; the dashboard
publishes `cmd_drive` directly, no Pi controller active). Control law:

- **Reachable area:** press/drag anywhere except the rear half — **no commanded reverse** (matches
  the `target_speed ≥ 0` wire clamp).
- **Speed setpoint = radial distance from center**, proportional → `cmd_drive.speed`.
- **Steering = the horizontal (x) offset drives a *continual adjustment* of the heading setpoint** —
  a rate, not an absolute angle: holding the stick left winds `cmd_drive.heading` leftward at a rate
  ∝ the x-offset, integrated over time. *(2026-07-24: confirm rate vs. absolute.)*
- **Gate:** the integrated heading setpoint is clamped so it can't run too far ahead of the car's
  current measured yaw — you can't wind the target arbitrarily far from reality. *(2026-07-24:
  confirm the reference is measured yaw, and the bound.)*
- **No turn-in-place, for free:** any x-offset also increases the radial distance, so a steering
  input always commands nonzero speed. Consistent with Ackermann (can't turn without rolling) —
  intentional, not a bug.
- Mechanically this is a **heading-nudge + speed-magnitude** device, exactly what the heading-setpoint
  wire contract wants; releasing to center → speed 0, heading holds.

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

- Custom message + service types (`follow_me_interfaces`)
- Mode-managed control: a gated service (`set_nav_mode`) selects the active controller per latched
  `nav_mode`; sibling controllers implement different policies
- Sensor fusion node (EKF: UWB bearing + pan angle + odom → tag position in the `odom` frame)
- Dead reckoning pose estimator (IMU yaw + wheel odometry → `nav_msgs/Odometry` + TF2)
- TF2 transforms (`odom` → `base_link` → `uwb_link` → `tag_link`)
- Live web dashboard over `foxglove_bridge` (telemetry, service calls, teleop publish)
- Parameter YAML configuration
- Launch files
- rosbag2 logging

(Dropped 2026-07-24: `ros2_control` hardware interface, custom `ros2_control` controller, and
Nav2-compatible action servers — see Goals. The Python `serial_bridge` is the permanent HAL
boundary; nav is mode-based, not action-based.)

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

### Phase 5 — Fusion node ✅ (core, 2026-07-24)
- Built: `tag_estimator` — an EKF whose state is the tag's `(x, y)` in `odom`, fusing
  `uwb/raw` + pan angle + odom. The car's motion and the pan angle live in the measurement model.
- Subscribes: `uwb/raw` (`UwbRaw`), `imu/data`, odom/pan. Publishes: `fused/tag_pose` (`TagEstimate`).
- **Refinements still open** (see NOTES "Pi reimplementation checklist"): distance dead-reckoning
  through UWB dropouts, the erratic-motion detector, and ~0.3 s anchor-lag compensation —
  carried from the deleted `fusion.cpp`, not yet ported.

### Phase 6 — Dead reckoning pose estimator ✅
- Integrates IMU yaw + wheel distance into 2D pose in `odom` frame
- Publishes: `/odom` (`nav_msgs/Odometry`), TF2 `odom → base_link`
- Subscribes: `/imu/data` (heading), `/wheel/state` (accumulated metres in `distance`)
- `odom` starts at identity (initial yaw subtracted).

### Phase 7 — ros2_control hardware interface ❌ DROPPED (2026-07-24)
Cut deliberately. `ros2_control`'s payoff is highest when the control loop runs on the Pi; here the
speed + heading PIDs live on the ESP32 and the Pi only shuttles setpoints, so a `SystemInterface`
would be a thin wrapper whose `write()` just forwards a heading setpoint. It also doesn't fit the
hardware: one serial bus carrying rich telemetry (IMU+cov, UWB, health, logs) that doesn't map to
flat `(name, double)` state interfaces — so it wouldn't even fully replace `serial_bridge`. The
Python bridge is the permanent HAL boundary. (If a "standard" is ever wanted, micro-ROS fits the
MCU-over-serial reality far better — parked in NOTES.)

### Phase 8 — Follow-me controller ✅ (core, 2026-07-24)
- Built: `nav_controller` — a *setpoint generator* (both PIDs stay on the ESP32). Commits the fused
  tag as a point in `odom`, steers to it via `cmd_drive` heading, HOLDs on high bearing uncertainty.
  Gated on `nav_mode` (`active_mode` param).
- **Two policies planned** (2026-07-24): a **simple** "steer at the tag" cut (minimal, easy to
  reason about) and the current/**complex** confidence-gated version, growing toward the ±60°-FOV
  recovery behavior (NOTES "Follow-me as waypoint planning with recovery"). They become sibling
  controllers with different `active_mode` values.
- Speed: distance-interpolated `target_speed` (port the min/max-speed-vs-distance logic from the
  ESP32's old `control.cpp` FOLLOW_ME case, readable at esp32 commit `075ab58`).

### Phase 9 — Manual field teleop (dashboard joystick)
- A `manual` `nav_mode` in which no Pi controller drives; the dashboard publishes `cmd_drive`
  directly. Single auto-centering virtual joystick — control law under "Dashboard Field Joystick".
  Supersedes the earlier `/follow_me` action idea (actions dropped 2026-07-24).

### Phase 10 — Waypoint missions (custom dead-reckoning, NOT Nav2)
- A `waypoint` `nav_mode` + controller. User drops numbered points on the dashboard canvas (in
  `odom`); the controller drives them sequentially via `cmd_drive`, using the pose estimator for
  progress. No costmap, no planner, no Nav2 — dead reckoning only.
- Related future ideas (NOTES): record/replay a driven path as a mission; phone waypoint UI.

### Phase 11 — UWB pan tracking
- A standalone Pi node (`pan_controller`) — **not** tied to any `nav_mode` — that keeps the DW3000
  anchor aimed at the tag. Subscribes `fused/tag_pose` (+odom), computes `target_pan`, dead-reckons
  the aim through UWB dropouts. Smooth rate-limited actuation is already on the ESP32 (`target_pan`,
  max slew). Open: separate pan-setpoint topic vs. field on `DriveCommand` (lean: separate topic —
  pan is independent of the drive command).

### Phase 12 — Field-ready bringup + visualization
- **Laptop-free bringup:** the Pi starts the whole stack on boot (systemd unit or equivalent); car
  drivable/demoable with no Mac attached.
- Single launch file starts everything; rosbag2 recording in the launch file.
- **Visualization host:** ROS2/RViz on macOS is effectively unsupported — use Foxglove Studio on the
  Mac against `foxglove_bridge` on the Pi (native Mac app, no Mac ROS2 install), plus the project's
  own web dashboard.

## Repository Reference

### Package structure
```
src/
├── follow_me_interfaces/     — custom message + service definitions (msg/, srv/)
└── follow_me_nodes/          — Python nodes + launch/ (serial_bridge, tag_estimator,
                                pose_estimator, tag_broadcaster, mode_manager,
                                nav_controller, pi_health)
```

Two packages, deliberately (2026-07-24). `follow_me_interfaces` is separate because message/service
generation must build first and be depended on without dragging in node code — the one split ROS2
forces. `follow_me_hardware` is gone with Phase 7 (no C++ plugin). A separate `follow_me_bringup`
isn't worth the package boilerplate for a solo project — launch files live in
`follow_me_nodes/launch/` until config churn justifies splitting.

### Build & run
See [cheat.md](./cheat.md) for the commands that actually work today.

```bash
colcon build --symlink-install
source install/setup.bash
ros2 run follow_me_nodes serial_bridge

# Whole stack (launch lives in follow_me_nodes, not a separate bringup package):
ros2 launch follow_me_nodes bringup.launch.py
```

### Key topics
| Topic | Type | Direction |
|-------|------|-----------|
| `/uwb/raw` | `follow_me_interfaces/UwbRaw` | ESP32 → ROS2 (tag range/bearing; Phase 5 fusion input) |
| `/imu/data` | `sensor_msgs/Imu` | ESP32 → ROS2 |
| `/fused/tag_pose` | `follow_me_interfaces/TagEstimate` | `tag_estimator` output — EKF tag position in `odom` |
| `/odom` | `nav_msgs/Odometry` | dead reckoning node output |
| `/wheel/state` | `follow_me_interfaces/WheelState` | fused speed + odometer + cogging flag, one stamped message |
| `/command/status` | `follow_me_interfaces/CommandStatus` | ESP32 control mode (`command_mode`) + accepted-command echo |
| `/actuator/status` | `follow_me_interfaces/ActuatorStatus` | live actuator outputs (throttle/steering/pan) |
| `/cmd_drive` | `follow_me_interfaces/DriveCommand` | controller/nav → bridge (later: hardware interface) |
| `/nav_mode` | `follow_me_interfaces/NavMode` | mode_manager → all (latched/transient_local): active Pi-side nav policy |
| `/sensor_health` | `follow_me_interfaces/SensorHealth` | ESP32 → ROS2 (~1-2 Hz): per-sensor update rates from `{"type":"health"}` frames, plus link-health/throughput stats (telem rate, seq gaps, TX drops, inter-frame jitter) |
| `/pi_health` | `follow_me_interfaces/PiHealth` | `pi_health` (~1 Hz): Pi host CPU load/util, bridge-process CPU, memory, temp — "is the Pi the bottleneck?" |
| `/cmd_pan` *(planned)* | scalar pan setpoint (deg) | `pan_controller` → bridge; merged into every TX frame as `target_pan`. Topic vs. `DriveCommand` field TBD |

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

**None (2026-07-24).** Action servers and Nav2 interfaces were dropped — see Goals. Navigation is
mode-based: `mode_manager` selects a continuous controller per `nav_mode` (`follow` simple/complex,
`manual`, `waypoint`, `stopped`), each publishing `cmd_drive`. Start/stop is a `set_nav_mode` service
call, not an action goal. Waypoint missions are a custom dead-reckoning controller, not
`nav2_msgs/FollowWaypoints`.
