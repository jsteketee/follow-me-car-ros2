# Serial Interface — ESP32 ↔ Pi

The field-level contract for the USB-serial link: what is on the wire, in what units, and how
to interpret each value. **Behavior** (failsafe, control modes, reboot recovery, command-stream
design) lives in `PROJECT_PLAN.md` "Serial Link" — this doc is only the field schema.

- **Transport:** USB-CDC, newline-delimited JSON, one object per line.
- **Rates:** telemetry ESP32→Pi at 50 Hz; commands Pi→ESP32 at 20 Hz.
- **Units:** the ESP32 speaks mph / cm / degrees; the bridge (`serial_bridge.py`) converts to
  SI / REP-103 (m, m/s, rad, CCW-positive) once, so every downstream ROS2 topic is SI.

## Wire rules

1. **No `null`, and no non-numeric value in a numeric field** — it crashes the bridge's
   `float()`/`int()` coercion. Missing keys are tolerated (a default is substituted); a *present*
   bad value drops the frame with a throttled `/rosout` warning.
2. **N/A is a sentinel, not absence.** Only `uwb_dist` (`< 0`), `uwb_age` (`-1`), and `cmd_age`
   (`-1`) carry one; every other field is always present with a real value.
3. **Stable JSON type per field** (int / float / string do not vary frame to frame).
4. **Unknown telemetry keys are ignored;** unknown `"type"` event frames warn once and drop.

**Conversion constants:** `MPH_TO_MPS = 0.44704`, `CM_TO_M = 0.01`, `DEG_TO_RAD = π/180`.

---

## Telemetry: ESP32 → Pi (50 Hz, flat JSON)

```json
{"ts":148230,"uwb_dist":212.4,"uwb_bearing":-3.75,"uwb_age":40,"yaw":271.30,"yaw_rate":1.85,"pitch":-1.20,"roll":0.55,"lax":0.142,"speed":1.983,"odo":4521.6,"cogging":0,"enc_fault":0,"mode":"SETPOINT","cmd_speed":2.00,"cmd_heading":270.0,"cmd_pan":-5.0,"cmd_age":60,"cmd_rejects":0,"throttle":0.318,"steering":-0.045,"esc_pwm":1567,"steer_pwm":1489,"pan_pwm":1472,"pan_angle":-4.80}
```

| Wire key | Wire (type·unit) | → Topic · field | ROS (type·unit) | Notes |
|---|---|---|---|---|
| `ts` | uint · ms | *(header stamp, all msgs)* | ROS time | device uptime; mapped via a first-frame offset. Backward jump = reboot marker. |
| `yaw` | float · deg `[0,360)` | `imu/data` · orientation | rad | compass-absolute; roll+pitch+yaw → quaternion |
| `pitch` | float · deg | `imu/data` · orientation | rad | |
| `roll` | float · deg | `imu/data` · orientation | rad | |
| `yaw_rate` | float · deg/s | `imu/data` · angular_velocity.z | rad/s | ×`DEG_TO_RAD`, sign as `yaw` |
| `lax` | float · m/s² | `imu/data` · linear_acceleration.x | m/s² | passthrough; forward axis |
| `speed` | float · mph | `wheel/state` · speed | m/s | ×`MPH_TO_MPS`; **signed** (`< 0` = reverse/rollback) |
| `odo` | float · cm | `wheel/state` · distance | m | ×`CM_TO_M`; **signed**; stitched continuous across reboots (does not reset) |
| `cogging` | int · 0/1 | `wheel/state` · cogging | bool | |
| `enc_fault` | int · 0/1 | `wheel/state` · enc_fault | bool | true → trust `distance` less |
| `mode` | string | `command/status` · command_mode | string | `SETPOINT` / `DIRECT` / `STOPPED` |
| `cmd_speed` | float · mph, ≥0 | `command/status` · cmd_speed | m/s | ×`MPH_TO_MPS`; never negative |
| `cmd_heading` | float · deg (compass) | `command/status` · cmd_heading | rad (odom) | `radians(deg − heading_offset)`, normalized |
| `cmd_pan` | float · deg | `command/status` · cmd_pan | rad | ×`DEG_TO_RAD` |
| `cmd_age` | long · ms | `command/status` · cmd_age_ms | int32 | **`-1` = none since boot** |
| `cmd_rejects` | ulong · count | `command/status` · cmd_rejects | uint32 | monotonic; ticks on a rejected command value |
| `throttle` | float · `[-1,1]` | `actuator/status` · throttle | dimensionless | control **output**, not command; `< 0` = braking. SETPOINT clamps to `[-0.25,1]` (PID brake floor); DIRECT passes the commanded effort through, so the full `[-1,1]` appears in DIRECT mode. |
| `steering` | float · `[-1,1]` | `actuator/status` · steering | dimensionless | control output |
| `esc_pwm` | int · µs | `actuator/status` · esc_pwm | uint16 | raw pulse written; 1500 neutral |
| `steer_pwm` | int · µs | `actuator/status` · steer_pwm | uint16 | raw pulse written; 1500 neutral |
| `pan_pwm` | int · µs | `actuator/status` · pan_pwm | uint16 | raw pulse written; 1500 neutral |
| `pan_angle` | float · deg | `actuator/status` · pan_angle | rad | ×`DEG_TO_RAD` |
| `uwb_dist` | float · cm | `uwb/raw` · distance | m | ×`CM_TO_M`; **`< 0` (wire `-1`) = no fix, not scaled** |
| `uwb_bearing` | float · deg (+ = right) | `uwb/raw` · bearing | rad (+ = left) | ×`DEG_TO_RAD` then **negated** (device +right → REP-103 +left) |
| `uwb_age` | long · ms | `uwb/raw` · age_ms | int32 | **`-1` = no fix / not reported** |

---

## Command: Pi → ESP32 (20 Hz)

### Setpoint frame (SETPOINT mode)

```json
{"target_speed":1.8,"target_heading":214.5,"target_pan":-5.0}
```

| Field | Wire (type·unit) | Source (SI) | Conversion / notes |
|---|---|---|---|
| `target_speed` | float · mph, ≥0 | `DriveCommand.speed` (m/s) | ÷`MPH_TO_MPS`; clamped ≥0 (no commanded reverse) |
| `target_heading` | float · deg (compass) | `DriveCommand.heading` (rad, odom) | `degrees(rad) + heading_offset`, wrapped `[0,360)`. Offset = EMA of `device_yaw − odom_yaw`. |
| `target_pan` | float · deg | pan policy (Pi) | optional; honored in any frame shape; 0 = nose, + = right |

### Direct frame (DIRECT mode)

```json
{"throttle":0.31,"steering":-0.18,"target_pan":-5.0}
```

| Field | Wire (type·unit) | Notes |
|---|---|---|
| `throttle` | float · `[-1,1]` | normalized effort, PIDs bypassed; `< 0` = brake/reverse (accepted). |
| `steering` | float · `[-1,1]` | normalized steering effort |
| `target_pan` | float · deg | optional; honored in any frame shape; 0 = nose, + = right |

Out-of-range or non-finite command values are rejected by the ESP32 and counted in `cmd_rejects`.
Rejection / failsafe behavior: see `PROJECT_PLAN.md`.

---

## Event frames (typed)

Any inbound line with a `"type"` key is an event, not telemetry.

**`log`** — re-logged as `[esp32] <msg>` at the mapped severity:
```json
{"type":"log","level":"error","msg":"ESC overtemp"}
```
`level` ∈ `debug|info|warn|error|fatal` (absent → `info`).

**`health`** — per-sensor update rates → `sensor_health` (`SensorHealth`, parallel arrays):
```json
{"type":"health","max_loop_us":6498,"sensors":{"imu":100.0,"uwb":10.0,"enc":248.0,"hall":42.0,"loop":1180.0}}
```
`sensors` keys are firmware-chosen (open set); values are Hz (`0` = silent/dead). Message staleness
= the reporter is unhealthy.

`max_loop_us` (top-level, µs) is the worst gap between control-loop iterations since the previous
health frame — a watchdog on loop-task stalls, distinct from the `sensors.loop` average rate. It
lives outside `sensors` because it is a duration, not a Hz rate. A spike here with the `sensors`
rates unchanged points at something blocking the loop task (e.g. USB-CDC TX back-pressure) rather
than a slow sensor.
