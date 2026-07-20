# Follow Me Car — ROS2 Project Plan

## Purpose

The durable reference for how this project is meant to work: goals, architecture, serial
protocol, phase plan, and hardware roster. Authoritative for specs — the design and the plan,
not their live status or change history (NOTES.md).

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

## Serial Protocol (ESP32 ↔ Pi)

**Incoming from ESP32 (50 Hz target, newline-delimited JSON; slimmed 2026-07-16 — the
onboard tag Kalman and per-sensor speed fields are gone):**
```json
{"ts":184223,"uwb_dist":142.3,"uwb_bearing":-12.45,"uwb_age":38,"yaw":87.25,"yaw_rate":-1.53,"pitch":0.82,"roll":-0.41,"lax":0.113,"speed":1.462,"odo":10482,"cogging":0,"mode":"SETPOINT","cmd_speed":1.50,"cmd_heading":90.0,"cmd_pan":0.0,"cmd_age":21,"cmd_rejects":0,"throttle":0.312,"steering":-0.084,"pan_angle":0.00}
```
`ts` (ms) is the ESP32 device timestamp — Pi computes dt from device time, not arrival time.
The bridge maps `ts` into the ROS clock via a constant offset captured on the first frame
(re-captured if `ts` jumps backwards, i.e. ESP32 reboot). Raw device uptime in a header would
stamp messages in 1970 and tf2 would silently drop the transforms; the offset cancels under
subtraction, so downstream dt is still exact device time.
`uwb_dist` (cm) + `uwb_bearing` (deg) are DW3000 AoA readings (-1 if no fix); `uwb_age` is
ms since the last fix (-1 = none since boot). This is the only tag stream on the wire — the
ESP32's tag Kalman (`fused_*`) was removed; Pi-side fusion (Phase 5) consumes these directly.

`odo` is the accumulated odometer in **cm** (printed `%.1f`).

**Units:** the ESP32 speaks mph / cm / degrees. The bridge node is the boundary and converts
everything to SI (REP-103: metres, m/s, radians) before publishing, so every ROS2 topic
downstream is SI by construction and needs no scale factors.
`speed` is the ESP32's fused speed estimate (mph) — the throttle PID's feedback signal. The
per-sensor speeds (hall, AS5600 `enc_speed`) are no longer streamed: speed fusion lives
permanently on the ESP32 and the Pi treats `speed` as *the* speed. `cogging` is the ESP32
latching cogging flag (0/1).
`yaw_rate` is the IMU yaw rate (deg/s, same sign convention as `yaw`) — the first difference
of consecutive rotation-vector yaws at ~100 Hz, for Pi-side trust gating of UWB bearing
measurements during rotation.
`lax` is IMU linear acceleration along the sensor x-axis (forward/back, m/s², gravity removed;
axis/sign pending bench verification). `mode` is the live control mode (`control_mode_str()`:
`SETPOINT` / `DIRECT` / `STOPPED`). The dashboard is the sole mode authority (serial frames
never change it), so the bridge should surface mode and warn when it is streaming command
frames the current mode will not act on. `cmd_speed`/`cmd_heading`/`cmd_pan`/`cmd_age` echo
the last **accepted** command values and the age of the last accepted setpoint frame in ms
(−1 = none since boot; age > 300 = cmd-timeout failsafe active) so the bridge can verify
what the car is acting on. `cmd_rejects` (added 2026-07-14) counts command values rejected
by validation since boot — counter ticking with `cmd_age` flat means frames are arriving
but invalid; `cmd_age` climbing with the counter flat means frames aren't arriving at all.
`throttle`/`steering` are the normalized [-1, 1] control outputs (post-PID, pre
hardware-shaping) for logging, tuning, and eventual Pi-side system identification.
`pan_angle` is the UWB anchor pan servo's live angle (deg, 0 = car nose, +right — same
convention as `target_pan`); absolute tag bearing = `yaw + pan_angle + uwb_bearing`. The
anchor smooths its bearing output (~0.3 s latency, measured 2026-07-14), so treat
`uwb_bearing` as stale while `pan_angle` is changing between frames.
(`cam_found`/`cam_x`/`cam_y` were removed 2026-07-13 with the camera module;
`fused_angle`/`fused_dist`/`fused_unc`/`enc_speed` removed 2026-07-16.)

**Outgoing to ESP32 (newline-delimited JSON):**
```json
{"target_speed":1.8,"target_heading":214.5}
```
`target_speed` (mph) feeds the ESP32's existing tuned speed PID. `target_heading` is an
**absolute compass heading in degrees, same convention as the telemetry `yaw` field** — the
ESP32 steers to hold it using the existing tuned steering PID with its error source swapped
from tag bearing to heading error (see "Control Loop Placement" below).

Validation (whole frame rejected, `cmd_rejects` ticked, watchdog NOT petted): non-finite
values, `target_speed` outside **[0, `rtConfig.maxSpeedMph`]** (2.5 default — note the cap
is dashboard-tunable, so the bridge should clamp to it rather than assume 2.5; the planned
capabilities announcement makes it discoverable), or `target_heading` outside [-360, 720]
(accepted headings are normalized to [0, 360)). A sustained out-of-range stream therefore
trips the 300 ms failsafe — by design.

Frame rule: ROS-side headings live in the `odom` frame (rad, REP-103), which
`pose_estimator` zeroed with a captured yaw offset — so the bridge converts on write to
device compass degrees, deriving the device-vs-odom yaw offset from streams it already
handles (`imu/data` carries device yaw; `odom` carries odom yaw). Implementation note for
the ESP32: heading error must be computed **wrapped to [-180, 180]** (the pattern exists in
`fusion.cpp`'s innovation wrapping) — a naive `setpoint − measure` across the 0/360 seam
commands a full spin.

**Raw-actuator frames (implemented 2026-07-14, was "reserved"):**
```json
{"throttle":0.31,"steering":-0.18}
```
Normalized efforts, acted on in the ESP32's DIRECT mode (PIDs bypassed; actuator
conditioning — deadband/scale/smoothing — still applies). Validation: reject the whole
frame on non-finite values, `throttle` outside **[0, 1]** (no reverse for now — reverse is
invisible to odometry), or `steering` outside **[-1, 1]**. DIRECT has its own 300 ms
cmd-timeout failsafe: throttle cut, steering holds the last commanded position (same
rationale as SETPOINT's heading hold). The ESP32 always parses and stores both frame shapes
into separate slots with separate timestamps; the current mode decides which one is acted
on. This is also the migration path if the control loops ever move to the Pi.

**Optional `target_pan` field (honored in any frame shape, added 2026-07-14):** aims the
UWB anchor's pan servo (deg, 0 = car nose, +right). Absent → previous pan target holds;
non-finite or |value| > 90 → counted in `cmd_rejects` without affecting the rest of the
frame. The pan module slew-limits motion (50°/s) and clamps travel to ±55° (`PAN_MAX_DEG`)
— values between 55° and 90° are accepted and silently clamped, a gap the planned
capabilities announcement closes. Pan-aiming *policy* lives on the Pi; the ESP32 only
executes. Telemetry `pan_angle` reports the live angle.

**ESP32 control modes (restructured 2026-07-14 — the mode roster now matches the HAL
role):** `SETPOINT` (setpoint frames → onboard PIDs; the boot default; named `REMOTE`
until 2026-07-16), `DIRECT` (raw-actuator frames), `STOPPED` (kill switch, latches until
a human re-arms). The old FOLLOW_ME / TEST / THROTTLE_TEST modes are gone, along with
`nav.cpp` — control owns the mode. The live mode is reported in telemetry (`mode`); the
dashboard `/mode` endpoint remains the sole mode authority.

**Command stream contract (decided 2026-07-12):** the bridge re-sends the current absolute
setpoint at a fixed **20 Hz** even when unchanged — the stream is the heartbeat, and
idempotent absolute frames are why no acks/checksums are needed (USB-CDC has per-packet
CRC). ESP32 failsafe (revised 2026-07-13): if no valid frame arrives for **300 ms**
(~33 cm at 2.5 mph), **neutral throttle only** — steering stays active, holding the last
commanded heading (centering the wheels mid-turn on a comms blip would make the car plow
straight; the original both-axes-neutral design is superseded). Before any command has
ever arrived, the held heading is the boot yaw, captured in `control_init()`. Resume
semantics are hybrid: after a timeout trip the car stays in SETPOINT and throttle
auto-resumes on the next valid frame; an ESP32 reboot (bridge detects `ts` jumping
backwards) still makes the bridge **halt TX and wait for a fresh command** — good hygiene,
though no longer safety-critical since the boot default is SETPOINT (`DEFAULT_CONTROL_MODE`,
2026-07-13): a
rebooted car waits at zero throttle holding its boot heading instead of re-entering
autonomy. A dashboard STOPPED always latches until a human re-arms, and neutrals both
axes (steering included). Frames of the wrong shape for the current mode are stored but
not acted on; serial frames never change the mode (dashboard `/mode` is the mode
authority).

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
- `odom` starts at identity (initial yaw subtracted). Reverse motion is invisible —
  the odometer does not tick backwards.

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
| `/command/status` | `follow_me_interfaces/CommandStatus` | ESP32 control mode + accepted-command echo |
| `/actuator/status` | `follow_me_interfaces/ActuatorStatus` | live actuator outputs (throttle/steering/pan) |
| `/cmd_drive` | `follow_me_interfaces/DriveCommand` | controller/nav → bridge (later: hardware interface) |

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
