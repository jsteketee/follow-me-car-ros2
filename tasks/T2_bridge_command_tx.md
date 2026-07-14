# T2 — DriveCommand message + serial bridge TX path

Task brief for an implementation agent. Self-contained, but the authoritative spec is this
repo's `PROJECT_PLAN.md` — read "Serial Protocol (ESP32 ↔ Pi)" (esp. the "Command stream
contract") before writing code. Conventions: relative topic names (namespacing at launch),
snake_case, SI units on every ROS topic; the bridge is the single SI/frame boundary.

## Context

- Repo: `follow-me-car-ros2` (ROS2 Jazzy workspace). Packages: `follow_me_interfaces`
  (msgs), `follow_me_nodes` (Python nodes).
- `src/follow_me_nodes/follow_me_nodes/serial_bridge.py` currently reads 50 Hz telemetry
  from the ESP32 over USB serial (pyserial, newline-delimited JSON) and publishes 4 topics.
  It has **no transmit path**, and the `serial.Serial` handle is a local variable inside the
  `_read_loop` daemon thread — restructuring is required.
- Companion firmware task (T1, other repo) is **complete and bench-validated
  (2026-07-13)**: the ESP32 accepts `{"target_speed":<mph>,"target_heading":<compass deg>}`
  frames in its REMOTE mode — now the boot default — with a 300 ms timeout failsafe
  (throttle-only; steering holds the last commanded heading). The telemetry frame also
  changed: `cam_*` fields removed; `lax`, `cmd_speed`/`cmd_heading`/`cmd_age`,
  `throttle`/`steering` added (see PROJECT_PLAN "Serial Protocol"). The TX path can still
  be verified against a pty.

## Scope

### 1. New message: `follow_me_interfaces/msg/DriveCommand.msg`

```
std_msgs/Header header
float32 speed      # m/s, forward; >= 0 (no reverse)
float32 heading    # rad, absolute heading in the odom frame, REP-103 (CCW positive)
```

Register it in `follow_me_interfaces/CMakeLists.txt`.

### 2. Bridge TX path (`serial_bridge.py`)

- **Handle restructure:** promote the serial handle to `self._ser`, guarded by a
  `threading.Lock`. The reconnect loop in `_read_loop` opens/closes/replaces it **under the
  lock**. Set a `write_timeout` (e.g. 0.1 s) so a wedged port cannot block the executor.
- **Subscribe** `cmd_drive` (`DriveCommand`, relative name, QoS depth 10 to match house
  style). The callback only **latches** the newest message — it does not write to serial.
- **20 Hz TX timer** (`create_timer(0.05, ...)`): every tick, serialize the latched command
  and write one newline-terminated JSON frame:
  `{"target_speed":<mph>,"target_heading":<device compass deg>}\n`
  Re-send even when unchanged — the stream is the ESP32's heartbeat.
- **Conversions (the bridge is the boundary, both directions):**
  - `speed`: m/s → mph (divide by 0.44704 — the inverse of `MPH_TO_MPS` used on read).
  - `heading`: odom-frame rad → device compass deg. Derive the offset from data the bridge
    already has or can subscribe to: device yaw arrives in every telemetry frame (`yaw`,
    deg); odom yaw comes from subscribing `odom` (`nav_msgs/Odometry`, extract yaw from the
    quaternion). `offset_deg = device_yaw_deg − degrees(odom_yaw)`, smoothed (EMA) and only
    updated when both sources are fresh. Then
    `target_heading = wrap_0_360(degrees(cmd.heading) + offset_deg)` — **derive the exact
    sign/direction mapping as the inverse of this file's existing inbound yaw handling**
    (`euler_deg_to_quaternion`), don't guess; leave a comment showing the derivation.
- **Safety rules (all mandatory):**
  - **Drop, never queue:** if the port is closed/absent, discard the tick. No buffered
    commands may ever replay on reconnect.
  - **Staleness gate:** if the latched command's stamp is older than 500 ms, stop sending
    (let the ESP32 timeout catch it). Zeroing is a command; silence is the failsafe.
  - **Reboot halt:** the bridge already detects ESP32 reboots (`ts` jumps backwards, see
    `_device_ts_to_ros_ns`). On detection, clear the latched command and **halt TX until a
    fresh `cmd_drive` arrives**, with a warn log — a rebooted ESP32 comes up in REMOTE at
    zero throttle holding its boot heading (no autonomy risk since 2026-07-13, but stale
    pre-reboot setpoints must still never be streamed at a car whose state was just reset).
    Also invalidate the heading offset (recompute from fresh data) since the device yaw
    reference may have changed.
  - Send `allow_nan=False`-safe JSON (reject/clamp non-finite values before serializing;
    clamp `speed` to >= 0).

### 3. Tests

- Structure the port access so it is injectable, and add a test against pyserial's
  `loop://` URL (or a pty pair) asserting: 20 Hz cadence, correct m/s→mph conversion,
  frames stop when the latched command goes stale, nothing is sent while disconnected, and
  nothing replays after reconnect.

## Out of scope

Teleop, action servers, launch-file changes beyond what the node needs, ros2_control,
any change to the four existing published topics, committing/pushing.

## Acceptance criteria

1. `colcon build --symlink-install` clean; existing telemetry topics unchanged
   (`ros2 topic hz imu/data` still ~44-50 Hz with hardware attached).
2. Unit tests above pass without hardware.
3. With hardware: `ros2 topic pub -r 10 cmd_drive follow_me_interfaces/DriveCommand ...`
   produces 20 Hz frames on the wire; stopping the pub stops TX within ~500 ms; unplugging
   and replugging the ESP32 never replays stale commands.

## Report back

List files changed, the derived heading sign/offset mapping (show the math), any deviation
from this brief with justification, and test results. Do not commit or push.
