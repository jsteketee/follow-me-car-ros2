// Horizontal metrics strip across the top: connection, ESP32 mode (read-only), command
// health, steering, UWB angles, tag range, and tag 1-sigma uncertainty (color-graded bars).
// Wraps to multiple rows on narrow / mobile viewports. Polls the shared live refs at 5 Hz.
import { useEffect, useState, CSSProperties } from "react";
import { useConnStatus } from "../ros/foxglove";
import { useLive } from "../ros/live";
import { resolve } from "../ros/tf2d";

const MODES = ["SETPOINT", "DIRECT", "STOPPED"];
const DEG = 180 / Math.PI;
const RANGE_SIGMA_MAX = 0.5;       // m — full bar (reasonable default)
const BEARING_SIGMA_MAX_DEG = 20;  // deg — full bar

// Fraction 0..1 -> green (good) through amber to red (bad).
function sigColor(t: number): string {
  const x = Math.max(0, Math.min(1, t));
  const lerp = (a: number, b: number, k: number) => Math.round(a + (b - a) * k);
  const green = [70, 199, 106], amber = [230, 195, 74], red = [214, 75, 62];
  const [c0, c1, k] = x < 0.5 ? [green, amber, x * 2] : [amber, red, (x - 0.5) * 2];
  return `rgb(${lerp(c0[0], c1[0], k)},${lerp(c0[1], c1[1], k)},${lerp(c0[2], c1[2], k)})`;
}

type Snap = {
  mode: string; cmdAgeMs: number; cmdRejects: number; cmdSpeed: number;
  steering: number; hasStatus: boolean; hasCmd: boolean;
  panDeg: number | null; uwbBearingDeg: number | null; tagDist: number | null;
  rangeSigma: number; bearingSigmaDeg: number; coasting: boolean; hasTagEst: boolean;
};

export function ControlPanel() {
  const status = useConnStatus();
  const { treeRef, statusRef } = useLive();
  const [s, setS] = useState<Snap>({
    mode: "", cmdAgeMs: -1, cmdRejects: 0, cmdSpeed: 0, steering: 0,
    hasStatus: false, hasCmd: false, panDeg: null, uwbBearingDeg: null, tagDist: null,
    rangeSigma: 0, bearingSigmaDeg: 0, coasting: false, hasTagEst: false,
  });

  useEffect(() => {
    const id = setInterval(() => {
      const st = statusRef.current, tree = treeRef.current;
      const car = resolve(tree, "base_link");
      const tag = resolve(tree, "tag_est_link");
      const uwb = resolve(tree, "uwb_link");

      let tagDist: number | null = null, uwbBearingDeg: number | null = null;
      if (car && tag) tagDist = Math.hypot(tag.x - car.x, tag.y - car.y);
      if (uwb && tag) {
        const dx = tag.x - uwb.x, dy = tag.y - uwb.y;
        const c = Math.cos(-uwb.yaw), sn = Math.sin(-uwb.yaw);
        uwbBearingDeg = Math.atan2(sn * dx + c * dy, c * dx - sn * dy) * DEG;
      }
      setS({
        mode: st.mode, cmdAgeMs: st.cmdAgeMs, cmdRejects: st.cmdRejects, cmdSpeed: st.cmdSpeed,
        steering: st.steering, hasStatus: st.hasStatus, hasCmd: st.hasCmd,
        panDeg: st.hasStatus ? st.pan * DEG : null, uwbBearingDeg, tagDist,
        rangeSigma: st.tagRangeSigma, bearingSigmaDeg: st.tagBearingSigma * DEG,
        coasting: st.tagCoasting, hasTagEst: st.hasTagEst,
      });
    }, 200);
    return () => clearInterval(id);
  }, [treeRef, statusRef]);

  const failsafe = s.hasCmd && s.cmdAgeMs > 300;
  const rangeRatio = s.hasTagEst ? s.rangeSigma / RANGE_SIGMA_MAX : 0;
  const bearingRatio = s.hasTagEst ? s.bearingSigmaDeg / BEARING_SIGMA_MAX_DEG : 0;

  return (
    <>
    {/* Mode selector on its own row — display-only for now; will accept input once a
        mode-set path exists (see PROJECT_PLAN: the ESP32 dashboard is the mode authority). */}
    <div className="moderow">
      <span className="mlabel">mode</span>
      <div className="modebtns">
        {MODES.map((m) => <span key={m} className={`modebtn ${s.mode === m ? "active" : ""}`}>{m}</span>)}
      </div>
    </div>

    <div className="metrics">
      <div className="metric">
        <span className="mlabel">link</span>
        <span className="mval"><span className={`dot ${status}`} />{status}</span>
      </div>

      <Metric label="cmd age" value={s.hasCmd && s.cmdAgeMs >= 0 ? `${s.cmdAgeMs} ms` : "—"} warn={failsafe} />
      <Metric label="rejects" value={s.hasCmd ? String(s.cmdRejects) : "—"} warn={s.hasCmd && s.cmdRejects > 0} />
      <Metric label="cmd spd" value={s.hasCmd ? `${s.cmdSpeed.toFixed(2)} m/s` : "—"} />

      <MetricBar label="steering" value={s.hasStatus ? s.steering.toFixed(2) : "—"} bar={s.hasStatus ? s.steering : 0} bipolar />
      <Metric label="uwb pan" value={s.panDeg == null ? "—" : `${s.panDeg.toFixed(1)}°`} />
      <Metric label="uwb→tag" value={s.uwbBearingDeg == null ? "—" : `${s.uwbBearingDeg.toFixed(1)}°`} />
      <Metric label="tag rng" value={s.tagDist == null ? "—" : `${s.tagDist.toFixed(2)} m`} />

      <MetricBar label="range σ" value={s.hasTagEst ? `± ${s.rangeSigma.toFixed(2)} m` : "—"} bar={rangeRatio} color={sigColor(rangeRatio)} />
      <MetricBar label="bearing σ" value={s.hasTagEst ? `± ${s.bearingSigmaDeg.toFixed(1)}°` : "—"} bar={bearingRatio} color={sigColor(bearingRatio)} />

      {s.coasting && <Metric label="tag" value="coasting" warn />}
    </div>
    </>
  );
}

// One label/value metric cell.
function Metric({ label, value, warn }: { label: string; value: string; warn?: boolean }) {
  return (
    <div className="metric">
      <span className="mlabel">{label}</span>
      <span className={`mval ${warn ? "warn" : ""}`}>{value}</span>
    </div>
  );
}

// Metric cell with a mini bar under the value.
function MetricBar({ label, value, bar, bipolar, color }: {
  label: string; value: string; bar: number; bipolar?: boolean; color?: string;
}) {
  return (
    <div className="metric">
      <span className="mlabel">{label}</span>
      <span className="mval">{value}</span>
      <Bar value={bar} bipolar={bipolar} color={color} />
    </div>
  );
}

// Small horizontal meter; bipolar centers zero (for signed steering). Optional fill color.
function Bar({ value, bipolar, color }: { value: number; bipolar?: boolean; color?: string }) {
  const v = Math.max(-1, Math.min(1, value));
  const pct = bipolar ? Math.abs(v) * 50 : v * 100;
  const left = bipolar ? (v >= 0 ? 50 : 50 - pct) : 0;
  const style: CSSProperties = { left: `${left}%`, width: `${pct}%` };
  if (color) style.background = color;
  return (
    <div className="bar">
      {bipolar && <div className="bar-mid" />}
      <div className="bar-fill" style={style} />
    </div>
  );
}
