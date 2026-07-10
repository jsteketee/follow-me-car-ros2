# ESP32 HAL — Command Path Design Spec

**Status:** design, not yet implemented.
**Owns:** the down-link (Pi → ESP32) command path and the ESP32's transition toward a
pure HAL. Refines the "Serial Protocol" and "Control Loop Placement" sections of
[PROJECT_PLAN.md](../PROJECT_PLAN.md); fold the settled decisions back into PROJECT_PLAN once
implemented and verified.

## 1. Purpose & scope

The data path up (ESP32 → Pi telemetry) works. This phase adds the **command path down**:
the Pi streams motion setpoints to the ESP32, the ESP32 executes them through its existing
tuned control loops, and a cmd-timeout failsafe brings the vehicle to neutral if the link
goes quiet. In the same step the ESP32 stops driving itself — it moves to **Pi-commanded
mode only**, the first step toward being a true Hardware Abstraction Layer (abstraction, no
autonomous behavior).

### In scope this phase
- Parse newline-delimited JSON setpoint frames on the existing USB-CDC serial link.
- Feed `target_speed` → speed loop, `target_angle` → steering loop (see §3–4).
- Cmd-timeout failsafe: drive both actuators to neutral on link loss (§5).
- Disable autonomous **actuation** — only Pi setpoints move the car (§6).
- Echo command state back in the telemetry frame for observability (§7).

### Deferred (explicitly not this phase)
- **Deleting** `fusion.cpp` / `nav.cpp` and other autonomous code. Fusion *computation*
  stays for now because its `fused_angle`/`fused_dist` telemetry is still how the Pi knows
  where the tag is; delete only after Pi-side fusion (PROJECT_PLAN Phase 5) exists. See §6.
- **Raw-actuator command mode** (`{"throttle","steering"}`) and migrating the control loops
  to the Pi (PROJECT_PLAN Phase 8). The framing here is forward-compatible with it (§8).

## 2. Transport & framing

- Same USB-CDC link that carries telemetry, opposite direction. Newline-delimited JSON,
  one setpoint object per line — mirrors the outbound telemetry framing.
- **Units stay native.** The ESP32 speaks mph / degrees on the command path just as it does
  on telemetry. The Pi-side bridge node is the *sole* SI boundary: it converts m/s → mph and
  rad → deg on the way down (§9). The ESP32 never sees SI and needs no ROS awareness.
- **Parser dispatches on which keys are present**, not on a fixed schema. Unknown keys are
  ignored. This is what lets the reserved raw-actuator mode (`throttle`/`steering`) be added
  later without touching the framing.
- Implementation note (to confirm against firmware): the stack is **ESP-IDF**, so the RX path
  parses with **cJSON** and reads a line at a time from the USB-CDC driver. Interleaved
  ESP-IDF log lines on the same UART are already tolerated on the Pi side by skipping any line
  that does not start with `{`.

## 3. Command interface

```json
{"target_speed": 1.8, "target_angle": -12.4}
```

| Field | Unit | Meaning |
|---|---|---|
| `target_speed` | mph, forward + | Setpoint for the ESP32 speed loop. |
| `target_angle` | degrees | **Absolute heading setpoint** in the same reference frame as the telemetry `yaw` field. |

- Both fields optional per the key-dispatch rule; a frame may carry one or both.
- Sign of `target_angle` matches the telemetry `yaw` sign exactly (see §4). The bridge does
  **not** negate it — unlike `fused_angle`, which it does negate for REP-103.
- Reverse (`target_speed` < 0): handling is firmware-dependent and out of scope to specify
  here — note that reverse is already invisible to odometry (NOTES open issue). Flag for the
  firmware pass.

## 4. Setpoint semantics

### `target_speed` → speed loop
Feeds the existing speed PID setpoint. No change to that loop's structure — it already nulls
`measured_speed − target_speed` and compensates ESC deadband / cogging / load. Keeping it is
consistent with "no autonomous behavior": a velocity setpoint is the standard ros2_control
velocity command interface (PROJECT_PLAN Phase 7), an *abstraction* of the drivetrain, not a
decision about where to go.

### `target_angle` → steering loop (absolute compass heading)
`target_angle` is an **absolute heading** the car should hold, expressed in the telemetry
`yaw` frame. The steering loop error is therefore `yaw − target_angle` (shortest-angle,
wrapped at ±180°).

**This repoints the steering loop's measurement input.** Today the loop nulls *bearing-to-tag*
(process variable = the on-board fusion's `fused_angle`, setpoint = 0 = tag ahead). With
autonomy disabled the ESP32 no longer uses "where's the tag," so:

| | Before (autonomous) | After (Pi-commanded) |
|---|---|---|
| Process variable | `fused_angle` (bearing to tag, from fusion) | **`yaw`** (IMU heading) |
| Setpoint | 0 (tag dead ahead) | **`target_angle`** (from Pi) |
| Output | steering | steering (unchanged) |

- Same structure, same units (degrees of heading error → steering), so the **tuned gains
  should transfer directly**. Confirm against the actual loop during implementation.
- The follow behavior is preserved, just relocated: if the Pi sends
  `target_angle = yaw + bearing_to_tag`, holding that heading is mathematically identical to
  pointing at the tag. The *decision* moved to the Pi; the *execution* stayed on the ESP32.
- **Trap to avoid:** do not leave the loop's PV wired to the now-idle fusion output while
  feeding `target_angle` as the setpoint — it would compare against stale data and steer
  wrong. The PV source must move to `yaw`.

### Why absolute, not relative
Absolute is idempotent (a dropped/duplicated serial frame does no harm; nothing accumulates),
matches how the Pi's nav/follow controllers already think (they compute a desired absolute
heading), and recovers cleanly after a comms gap. Its only cost — needing a shared reference
frame — is free here because `target_angle` is *defined* in the `yaw` frame the Pi already
receives. Relative ("turn N° from now") only pays off for discrete human jog commands, which
is not the streamed-setpoint use case.

## 5. Cmd-timeout failsafe

- Track `last_cmd_ms` = device time of the last valid setpoint frame.
- If `now − last_cmd_ms > CMD_TIMEOUT`, enter failsafe: **drive steering and ESC directly to
  0.0** (servo centered, ESC neutral).
- **CMD_TIMEOUT default: 300 ms** (~15 missed frames at the 50 Hz stream). Tunable; pick the
  value against the slowest expected command cadence once the Pi streams commands.
- **Auto-recovering**, not latching (unlike the cogging flag): the next valid setpoint frame
  exits failsafe and resumes normal loop execution.
- **Failsafe acts below the loops.** "Steering 0.0 / ESC 0.0" means commanding the actuators
  to neutral *directly*, bypassing both PIDs — not "setpoint heading 0" (which would swerve
  toward heading 0) nor "target_speed 0" (which would route through the speed loop). This
  actuator-level neutral path is the same plumbing the later raw-actuator mode will use.
- On boot the ESP32 starts in failsafe (neutral) and stays there until the first valid
  setpoint arrives — no autonomous fallback.

## 6. Mode arbitration — Pi-commanded only

- The ESP32 runs in **one mode: Pi-commanded**. There is no standalone autonomous drive mode
  and no fallback to one; a comms loss falls back to failsafe neutral (§5), never to
  self-driving.
- **This phase disables autonomous *actuation***: the ESP32 no longer moves the car from its
  own fusion/nav outputs. Only `target_speed` / `target_angle` from the Pi actuate.
- **Fusion *computation* stays for now.** `fused_angle` / `fused_dist` / `fused_unc` continue
  to be computed and streamed as telemetry, because the Pi currently depends on them to know
  the tag's bearing (there is no Pi-side fusion yet). This is the data dependency behind the
  staging in §1.
- **Later:** once Pi-side fusion (Phase 5) supplies the tag bearing, delete `fusion.cpp` /
  `nav.cpp` / the autonomous decision code entirely. Tracked as deferred, not this phase.

## 7. Observability — telemetry echo

Add three fields to the outbound telemetry frame so the Pi can *see* the command state rather
than infer it:

| Field | Meaning |
|---|---|
| `cmd_speed` | Last accepted `target_speed` (mph), or the failsafe value while in failsafe. |
| `cmd_angle` | Last accepted `target_angle` (deg), or the failsafe value while in failsafe. |
| `failsafe` | `0` normal, `1` while in cmd-timeout failsafe. |

Cheap, and it makes failsafe entry/exit and command round-trip directly observable in a
rosbag / Foxglove without guessing. Additive to the existing JSON — no consumer breaks.

## 8. Forward compatibility

The command path is designed so the later migration (control loops → Pi, PROJECT_PLAN Phase 8)
is a config change, not a re-architecture:

```
Now:   {"target_speed": <mph>,  "target_angle": <deg abs heading>}   ← loops on ESP32
Later: {"throttle": <-1..1>,    "steering": <-1..1>}                 ← loops on Pi, raw HAL
```

- Key-dispatch parsing (§2) accepts either form; adding the raw keys touches only the dispatch,
  not the framing or failsafe.
- The failsafe's actuator-level neutral path (§5) is exactly the write path raw mode uses.
- The steering field name changes (`target_angle` → `steering`) because the *quantity* changes
  (heading setpoint → raw position) when the steering loop leaves the ESP32. `target_speed` →
  `throttle` likewise, and only if/when the speed loop migrates (PROJECT_PLAN flags that as the
  risky, latency-sensitive one to defer).

## 9. Pi-side (bridge) responsibilities

The command path also needs the bridge node (`serial_bridge.py`) extended — currently
uplink-only. Not the focus of this spec, but the contract it must honor:

- Subscribe to a command topic (setpoint source: follow/nav controllers later; a manual
  publisher for bring-up).
- Convert **SI → native** on the way down: m/s → mph, rad → deg. Heading uses the **same sign
  as the incoming `yaw`** (no negation), so `target_angle` lands in the frame the ESP32's loop
  expects.
- Serialize `{"target_speed","target_angle"}` as one newline-delimited JSON line and write it
  to the same serial handle the read loop uses (guard the handle for concurrent read/write).

## 10. Open items to confirm against firmware

1. The existing steering loop really does null `fused_angle` today (assumed in §4) — confirm,
   and confirm the gains transfer to a `yaw`-referenced error.
2. Speed-loop behavior for `target_speed ≤ 0` (stop vs. reverse), given reverse is invisible to
   odometry.
3. cJSON is available/used on the RX path; USB-CDC line reads don't starve the 50 Hz telemetry
   task.
4. CMD_TIMEOUT final value once the Pi's command cadence is known.
