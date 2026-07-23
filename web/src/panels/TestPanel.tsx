// "test" tab: a manual drive-command sender + live actuator PWM readout.
// The three shape buttons (Setpoint/Direct/Inactive) pick the command SHAPE published to
// cmd_drive — NOT the ESP mode. Publishes only while nav_mode == "stopped" (so it can't fight
// nav_controller); Inactive stops publishing -> the ESP's 500 ms staleness failsafe neutralizes.
import { useEffect, useRef, useState } from "react";
import { useLive } from "../ros/live";
import { usePublish } from "../ros/foxglove";
import { ns } from "../ros/topics";

const MPH_TO_MPS = 0.44704;
const MAX_MPH = 5;
const PAN_MAX_DEG = 90;
const TX_PERIOD_MS = 100;  // dashboard heartbeat; must stay under the bridge's 500 ms staleness

const CMD_TOPIC = ns("cmd_drive");
const CMD_SCHEMA_NAME = "follow_me_interfaces/msg/DriveCommand";
const CMD_SCHEMA = `std_msgs/Header header
uint8 SETPOINT=0
uint8 DIRECT=1
uint8 shape
float32 speed
float32 heading
float32 throttle
float32 steering
float32 pan_deg
================================================================================
MSG: std_msgs/Header
builtin_interfaces/Time stamp
string frame_id
================================================================================
MSG: builtin_interfaces/Time
int32 sec
uint32 nanosec`;

type Shape = "setpoint" | "direct" | "inactive";
const SHAPES: { key: Shape; label: string }[] = [
  { key: "setpoint", label: "Setpoint" },
  { key: "direct", label: "Direct" },
  { key: "inactive", label: "Inactive" },
];

// One labeled slider row: label + right-aligned value over a full-width range input.
function Slider({ label, value, display, min, max, step, disabled, onChange }: {
  label: string; value: number; display: string; min: number; max: number; step: number;
  disabled: boolean; onChange: (v: number) => void;
}) {
  return (
    <div className={`sliderrow ${disabled ? "locked" : ""}`}>
      <div className="sliderhead">
        <span className="slabel">{label}</span>
        <span className="sval">{display}</span>
      </div>
      <input type="range" min={min} max={max} step={step} value={value} disabled={disabled}
             title={disabled ? "🚫 locked in this mode" : undefined}
             onChange={(e) => onChange(parseFloat(e.target.value))} />
    </div>
  );
}

export function TestPanel() {
  const { statusRef, treeRef } = useLive();
  const publish = usePublish();

  const [shape, setShape] = useState<Shape>("inactive");
  const [speed, setSpeed] = useState(0);        // mph
  const [throttle, setThrottle] = useState(0);  // -1..1
  const [steering, setSteering] = useState(0);  // -1..1
  const [pan, setPan] = useState(0);            // -1..1 of PAN_MAX_DEG
  const headingRef = useRef(0);                 // captured odom heading (rad) for setpoint

  // Re-render at a low rate to reflect live nav_mode + actuator PWM held in the shared ref.
  const [, bump] = useState(0);
  useEffect(() => {
    const id = setInterval(() => bump((n) => n + 1), 200);
    return () => clearInterval(id);
  }, []);

  const s = statusRef.current;
  const navTest = s.navMode === "test";

  // Switch command shape: zero the sliders locked in the new shape; capture heading for setpoint.
  const selectShape = (next: Shape) => {
    if (next === "inactive") { setSpeed(0); setThrottle(0); setSteering(0); setPan(0); }
    else if (next === "setpoint") {
      setThrottle(0); setSteering(0); setPan(0);
      headingRef.current = treeRef.current.get("base_link")?.yaw ?? 0;
    } else if (next === "direct") { setSpeed(0); }
    setShape(next);
  };

  // Latest command snapshot for the heartbeat, so the interval never reads stale closures.
  const latest = useRef({ shape, speed, throttle, steering, pan, navTest });
  latest.current = { shape, speed, throttle, steering, pan, navTest };

  // Heartbeat: while a shape is active AND nav is stopped, republish cmd_drive every TX_PERIOD_MS
  // to keep it fresh. Inactive or nav != stopped -> silence -> the ESP failsafe neutralizes.
  useEffect(() => {
    const id = setInterval(() => {
      const c = latest.current;
      if (c.shape === "inactive" || !c.navTest) return;
      const direct = c.shape === "direct";
      publish(CMD_TOPIC, CMD_SCHEMA_NAME, CMD_SCHEMA, {
        header: { stamp: { sec: 0, nanosec: 0 }, frame_id: "" },
        shape: direct ? 1 : 0,
        speed: direct ? 0 : c.speed * MPH_TO_MPS,
        heading: direct ? 0 : headingRef.current,
        throttle: direct ? c.throttle : 0,
        steering: direct ? c.steering : 0,
        pan_deg: direct ? c.pan * PAN_MAX_DEG : 0,
      });
    }, TX_PERIOD_MS);
    return () => clearInterval(id);
  }, [publish]);

  const pct = (v: number) => `${Math.round(v * 100)}%`;
  const lockSpeed = shape !== "setpoint";
  const lockDirect = shape !== "direct";

  return (
    <div className="testview">
      <div className="card">
        <div className="cardtitle">Direct Control</div>

        <div className="shaperow">
          {SHAPES.map(({ key, label }) => (
            <button key={key} className={`shapebtn ${shape === key ? "active" : ""}`}
                    onClick={() => selectShape(key)}>{label}</button>
          ))}
        </div>

        {!navTest && (
          <div className="cardnote">nav is “{s.navMode || "…"}” — set nav_mode to <b>test</b> to send commands</div>
        )}

        <Slider label="Target Speed" value={speed} display={`${speed.toFixed(1)} mph`}
                min={0} max={MAX_MPH} step={0.1} disabled={lockSpeed} onChange={setSpeed} />
        <Slider label="Throttle" value={throttle} display={pct(throttle)}
                min={-1} max={1} step={0.01} disabled={lockDirect} onChange={setThrottle} />
        <Slider label="Steering" value={steering} display={pct(steering)}
                min={-1} max={1} step={0.01} disabled={lockDirect} onChange={setSteering} />
        <Slider label="Pan" value={pan} display={pct(pan)}
                min={-1} max={1} step={0.01} disabled={lockDirect} onChange={setPan} />
      </div>

      <div className="card">
        <div className="cardtitle">Actuator PWM</div>
        <div className="pwmgrid">
          <div className="htile"><span className="mlabel">ESC</span><span className="hval">{s.escPwm} µs</span></div>
          <div className="htile"><span className="mlabel">Steering</span><span className="hval">{s.steerPwm} µs</span></div>
          <div className="htile"><span className="mlabel">Pan</span><span className="hval">{s.panPwm} µs</span></div>
        </div>
      </div>
    </div>
  );
}
