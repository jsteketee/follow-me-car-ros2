# Serial Interface — ESP32 ↔ Pi

The field-level contract for the USB-serial link: what is on the wire, in what units, and how
to interpret each value. **Behavior** (failsafe, control modes, reboot recovery, command-stream
design) lives in `PROJECT_PLAN.md` "Serial Link" — this doc is only the field schema.

- **Transport:** USB-CDC, newline-delimited JSON, one object per line.
- **Rates:** telemetry ESP32→Pi at 50 Hz; commands Pi→ESP32 at 20 Hz.
- **Units:** the ESP32 speaks mph / cm / degrees; the bridge (`serial_bridge.py`) converts to
  SI / REP-103 (m, m/s, rad, CCW-positive) once, so every downstream ROS2 topic is SI.
- **Idempotency (serial + telemetry requirement):** every telemetry frame is a complete,
  self-contained snapshot of current state — no deltas, no dependence on prior frames. Any
  single received frame fully reconstructs the receiver's view, so a dropped or torn frame
  costs only latency (the next frame restores it), never lost or ambiguous state. Data
  freshness is carried explicitly in-frame (timestamps), never inferred from frame arrival.

## Wire rules

1. **No `null`, and no non-numeric value in a numeric field** — it crashes the bridge's
   `float()`/`int()` coercion. Missing keys are tolerated (a default is substituted); a *present*
   bad value drops the frame with a throttled `/rosout` warning.
2. **N/A is a sentinel, not absence.** Every field is always present with a value; the only
   sentinels are the group timestamps `imu.t` / `uwb.t` / `cmd.t` (**`-1`** = that producer has
   produced nothing since boot) and `uwb.dist` (**`< 0`**, wire `-1` = no fix, not scaled).
3. **Stable JSON type per field** (int / float / string do not vary frame to frame).
4. **Unknown telemetry keys are ignored;** unknown `"type"` event frames warn once and drop.

**Conversion constants:** `MPH_TO_MPS = 0.44704`, `CM_TO_M = 0.01`, `DEG_TO_RAD = π/180`.

---

## Telemetry: ESP32 → Pi (50 Hz, grouped JSON)

Fields are grouped by producer. `ts` is the frame emit time (envelope). A group whose producer is
async — able to stall while the loop keeps emitting frames — carries its own capture timestamp `t`;
loop-driven groups have no `t` and are stamped with the frame `ts`.

```json
{
  "ts": 148230, "seq": 148231,
  "imu":   { "t": 148228, "yaw": 271.30, "yaw_rate": 1.85, "pitch": -1.20, "roll": 0.55, "lax": 0.142 },
  "uwb":   { "t": 148190, "dist": 212.4, "bearing": -3.75 },
  "cmd":   { "t": 148170, "speed": 2.00, "heading": 270.0, "pan": -5.0, "throttle": 0.00, "steering": 0.00, "rejects": 0 },
  "wheel": { "speed": 1.983, "odo": 4521.6, "cogging": 0, "enc_fault": 0 },
  "ctrl":  { "throttle": 0.318, "steering": -0.045, "esc_pwm": 1567, "steer_pwm": 1489, "pan_pwm": 1472, "pan_angle": -4.80 }
}
```

**Group timestamps** — ESP32 `millis()`, same clock as `ts`; **`-1` = producer has produced nothing since boot**:
- `imu.t` — sensor **capture** time of the last BNO085 rotation-vector decode; applies to all `imu.*` (orientation + `lax`).
- `uwb.t` — sensor **capture** time of the last accepted DW3000 ranging frame; applies to all `uwb.*`.
- `cmd.t` — command **receipt** time of the last accepted Pi command of any shape (setpoint or direct); applies to all `cmd.*`.

The Pi stamps each `t`-bearing group's ROS header with that group's `t` and derives the `age_ms` fields
as `ts − t`; `wheel` and `ctrl` have no `t` and are stamped with the frame `ts`. `enc_fault` is the
fusion-input health signal.

| Wire key | Wire (type·unit) | → Topic · field | ROS (type·unit) | Notes |
|---|---|---|---|---|
| `ts` | uint · ms | *(stamps groups without a `t`)* | ROS time | device uptime; first-frame offset maps to ROS clock. Backward jump = reboot marker. |
| `seq` | uint · count | *(bridge: seq-gap detector)* | — | monotonic per emitted frame; ++ before the TX drop-check, so a gap = frames dropped/lost. Restarts at 0 on reboot. |
| `imu.t` | long · ms | *(stamps `imu/data` header)* | ROS time | last rotation-vector decode; **`-1` = none since boot** |
| `imu.yaw` | float · deg `[0,360)` | `imu/data` · orientation | rad | compass-absolute; roll+pitch+yaw → quaternion |
| `imu.pitch` | float · deg | `imu/data` · orientation | rad | |
| `imu.roll` | float · deg | `imu/data` · orientation | rad | |
| `imu.yaw_rate` | float · deg/s | `imu/data` · angular_velocity.z | rad/s | ×`DEG_TO_RAD`, sign as `yaw` |
| `imu.lax` | float · m/s² | `imu/data` · linear_acceleration.x | m/s² | passthrough; forward axis |
| `uwb.t` | long · ms | *(stamps `uwb/raw`; → `age_ms`)* | ROS time / int32 | last accepted ranging frame; Pi sets `age_ms = ts − t`; **`-1` = none since boot** |
| `uwb.dist` | float · cm | `uwb/raw` · distance | m | ×`CM_TO_M`; **`< 0` (wire `-1`) = no fix, not scaled** |
| `uwb.bearing` | float · deg (+ = right) | `uwb/raw` · bearing | rad (+ = left) | ×`DEG_TO_RAD` then **negated** (device +right → REP-103 +left) |
| `cmd.t` | long · ms | *(stamps `command/status`; → `cmd_age_ms`)* | ROS time / int32 | last accepted command; Pi sets `cmd_age_ms = ts − t`; **`-1` = none since boot** |
| `cmd.speed` | float · mph, ≥0 | `command/status` · cmd_speed | m/s | ×`MPH_TO_MPS`; never negative |
| `cmd.heading` | float · deg (compass) | `command/status` · cmd_heading | rad (odom) | `radians(deg − heading_offset)`, normalized |
| `cmd.pan` | float · deg | `command/status` · cmd_pan | rad | ×`DEG_TO_RAD` |
| `cmd.throttle` | float · `[-1,1]` | `command/status` · cmd_throttle | dimensionless | accepted **DIRECT** throttle echo (PIDs bypassed), not the actuator output; the Pi reads it when it commanded DIRECT — it knows the shape it sent |
| `cmd.steering` | float · `[-1,1]` | `command/status` · cmd_steering | dimensionless | accepted **DIRECT** steering echo |
| `cmd.rejects` | ulong · count | `command/status` · cmd_rejects | uint32 | monotonic; ticks on a rejected command value |
| `wheel.speed` | float · mph | `wheel/state` · speed | m/s | ×`MPH_TO_MPS`; **signed** (`< 0` = reverse/rollback) |
| `wheel.odo` | float · cm | `wheel/state` · distance | m | ×`CM_TO_M`; **signed**; stitched continuous across reboots (does not reset) |
| `wheel.cogging` | int · 0/1 | `wheel/state` · cogging | bool | |
| `wheel.enc_fault` | int · 0/1 | `wheel/state` · enc_fault | bool | true → trust `distance` less |
| `ctrl.throttle` | float · `[-1,1]` | `actuator/status` · throttle | dimensionless | control **output**, not command; `< 0` = braking. SETPOINT clamps to `[-0.25,1]` (PID brake floor); DIRECT passes the commanded effort through, so the full `[-1,1]` appears in DIRECT mode. |
| `ctrl.steering` | float · `[-1,1]` | `actuator/status` · steering | dimensionless | control output |
| `ctrl.esc_pwm` | int · µs | `actuator/status` · esc_pwm | uint16 | raw pulse written; 1500 neutral |
| `ctrl.steer_pwm` | int · µs | `actuator/status` · steer_pwm | uint16 | raw pulse written; 1500 neutral |
| `ctrl.pan_pwm` | int · µs | `actuator/status` · pan_pwm | uint16 | raw pulse written; 1500 neutral |
| `ctrl.pan_angle` | float · deg | `actuator/status` · pan_angle | rad | ×`DEG_TO_RAD` |

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

Out-of-range or non-finite command values are rejected by the ESP32 and counted in `cmd.rejects`.
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
{"type":"health","max_loop_us":6498,"tx_drops":0,"sensors":{"imu":100.0,"uwb":10.0,"enc":248.0,"hall":42.0,"loop":1180.0}}
```
`sensors` keys are firmware-chosen (open set); values are Hz (`0` = silent/dead). Message staleness
= the reporter is unhealthy.

`max_loop_us` (top-level, µs) is the worst gap between control-loop iterations since the previous
health frame — a watchdog on loop-task stalls, distinct from the `sensors.loop` average rate. It
lives outside `sensors` because it is a duration, not a Hz rate. A spike here with the `sensors`
rates unchanged points at something blocking the loop task (e.g. USB-CDC TX back-pressure) rather
than a slow sensor.

`tx_drops` (top-level, count) is the cumulative number of telemetry frames the ESP32 dropped
because the USB-CDC TX buffer could not hold them since boot — the producer-side view of
back-pressure.

The bridge augments `SensorHealth` with Pi-derived link stats that are **not** on the wire:
`telem_frames_1s` (frames parsed in the last 1 s), `seq_gaps` (missing `seq` numbers, cumulative),
`parse_fails` (unparseable / dropped lines, cumulative), and `max_inter_frame_gap_ms` (worst gap
between consecutive frames since the last health emit — the jitter metric). Together with
`tx_drops` these localize where telemetry is lost: `tx_drops` ≈ `seq_gaps` → dropped at the ESP
buffer; `seq_gaps` with `tx_drops` ≈ 0 → lost in transit; `parse_fails` → torn/garbled frames.
