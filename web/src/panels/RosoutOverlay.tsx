// Error/log readout overlaid on the canvas panel: /rosout records from every ROS2 node
// (serial_bridge re-logs ESP32 "[esp32] ..." event frames here too). A corner chip cycles
// the severity threshold. Ring buffer in a ref + low-rate repaint.
import { useEffect, useRef, useState } from "react";
import { useRosTopic } from "../ros/foxglove";
import { RosoutLog } from "../ros/types";

// Per-severity display toggles; buffering floor is INFO (nodes don't publish DEBUG to
// /rosout by default), so enabling a severity reveals recently buffered lines too.
const BUFFER_MIN = 20;
const MAX_LINES = 100;     // ring-buffer cap (also bounds the transient_local connect burst)
const MAX_AGE_S = 300;     // lines older than this fade out of the overlay
const REPAINT_MS = 250;

type Severity = "info" | "warn" | "error";
const SEVERITIES: Severity[] = ["info", "warn", "error"];
const severityOf = (level: number): Severity =>
  level >= 40 ? "error" : level >= 30 ? "warn" : "info";

type Line = { key: number; level: number; name: string; msg: string; count: number; wall: number };

export function RosoutOverlay() {
  const bufRef = useRef<Line[]>([]);
  const keyRef = useRef(0);
  const [shown, setShown] = useState<Record<Severity, boolean>>({
    info: false, warn: true, error: true,
  });
  const [, bump] = useState(0);

  useRosTopic("/rosout", (m: RosoutLog) => {  // GLOBAL topic — never namespaced
    if (m.level < BUFFER_MIN) return;
    const buf = bufRef.current;
    const last = buf[buf.length - 1];
    if (last && last.name === m.name && last.msg === m.msg) {
      last.count += 1;                        // collapse consecutive repeats into (xN)
      last.wall = Date.now();
    } else {
      buf.push({ key: keyRef.current++, level: m.level, name: m.name, msg: m.msg, count: 1, wall: Date.now() });
      if (buf.length > MAX_LINES) buf.splice(0, buf.length - MAX_LINES);
    }
  });

  useEffect(() => {
    const id = setInterval(() => bump((n) => n + 1), REPAINT_MS);
    return () => clearInterval(id);
  }, []);

  const lines = bufRef.current.filter(
    (l) => shown[severityOf(l.level)] && Date.now() - l.wall < MAX_AGE_S * 1000);
  return (
    <div className="rosout-overlay">
      {lines.map((l) => (
        <div key={l.key} className={`rosout-line ${severityOf(l.level) === "error" ? "err" : severityOf(l.level)}`}>
          [{l.name}] {l.msg}{l.count > 1 ? ` (x${l.count})` : ""}
        </div>
      ))}
      <div className="rosout-levels">
        {SEVERITIES.map((sev) => (
          <button
            key={sev}
            className={`rosout-level ${sev} ${shown[sev] ? "on" : ""}`}
            onClick={() => setShown((s) => ({ ...s, [sev]: !s[sev] }))}
          >
            {sev}
          </button>
        ))}
      </div>
    </div>
  );
}
