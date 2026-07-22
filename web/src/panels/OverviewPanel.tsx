// Vehicle status view: per-sensor update rates from the ESP32's ~1-2 Hz health frames.
// Tiles grey out when the report itself goes stale (the reporter is unhealthy), and a
// 0 Hz sensor reads as dead. Polls the shared live ref at the overlay repaint rate.
import { useEffect, useState } from "react";
import { useLive } from "../ros/live";

const STALE_S = 3;         // no health frame for this long -> distrust the numbers
const POLL_MS = 250;

// Worst-case loop gap (us) -> readable string; switch to ms once past a millisecond.
function fmtLoop(us: number): string {
  return us >= 1000 ? `${(us / 1000).toFixed(1)} ms` : `${us} µs`;
}

export function OverviewPanel() {
  const { statusRef } = useLive();
  const [, bump] = useState(0);

  useEffect(() => {
    const id = setInterval(() => bump((n) => n + 1), POLL_MS);
    return () => clearInterval(id);
  }, []);

  const s = statusRef.current;
  const stale = !s.hasHealth || performance.now() / 1000 - s.healthWall > STALE_S;

  return (
    <div className="overview">
      <div className="ovsection">
        sensor health{s.hasHealth && stale ? " — stale, last report distrusted" : ""}
      </div>
      {!s.hasHealth && (
        <div className="ovempty">
          no health frames received — ESP32 {"{"}"type":"health"{"}"} reporting offline
        </div>
      )}
      <div className="healthgrid">
        {s.health.map(({ name, hz }) => (
          <div key={name} className={`htile ${stale ? "stale" : hz === 0 ? "dead" : ""}`}>
            <span className="mlabel">{name}</span>
            <span className="hval">{stale ? "—" : `${hz >= 100 ? hz.toFixed(0) : hz.toFixed(1)} Hz`}</span>
          </div>
        ))}
        {s.hasHealth && (
          <div className={`htile ${stale ? "stale" : ""}`}>
            <span className="mlabel">telem rx</span>
            <span className="hval">{stale ? "—" : `${s.telemFrames1s} /s`}</span>
          </div>
        )}
        {s.hasHealth && (
          <div className={`htile ${stale ? "stale" : ""}`}>
            <span className="mlabel">max loop</span>
            <span className="hval">{stale || s.maxLoopUs === 0 ? "—" : fmtLoop(s.maxLoopUs)}</span>
          </div>
        )}
      </div>
    </div>
  );
}
