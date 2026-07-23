// System health, grouped by layer along the data path: Sensors -> ESP -> Link -> Pi.
// A red/amber tile in the leftmost column is the likely root. sensor_health drives the first
// three columns; pi_health drives the Pi column, each with its own staleness. Polls the live ref.
import { useEffect, useState, ReactNode } from "react";
import { useLive } from "../ros/live";

const STALE_S = 3;         // no fresh source frame for this long -> distrust the column
const POLL_MS = 250;

// Worst-case loop gap (us) -> readable string; switch to ms once past a millisecond.
function fmtLoop(us: number): string {
  return us >= 1000 ? `${(us / 1000).toFixed(1)} ms` : `${us} µs`;
}

// Update rate -> "NN Hz", one decimal below 100 Hz.
function fmtHz(hz: number): string {
  return `${hz >= 100 ? hz.toFixed(0) : hz.toFixed(1)} Hz`;
}

// One metric tile: label over value, tinted by the passed status class (stale/dead/warn/crit).
function Tile({ label, value, cls = "" }: { label: string; value: string; cls?: string }) {
  return (
    <div className={`htile ${cls}`}>
      <span className="mlabel">{label}</span>
      <span className="hval">{value}</span>
    </div>
  );
}

// One labeled column of tiles.
function Col({ title, children }: { title: string; children: ReactNode }) {
  return (
    <div className="healthcol">
      <div className="ovsection">{title}</div>
      <div className="coltiles">{children}</div>
    </div>
  );
}

export function OverviewPanel() {
  const { statusRef } = useLive();
  const [, bump] = useState(0);

  useEffect(() => {
    const id = setInterval(() => bump((n) => n + 1), POLL_MS);
    return () => clearInterval(id);
  }, []);

  const s = statusRef.current;
  const now = performance.now() / 1000;
  const hStale = !s.hasHealth || now - s.healthWall > STALE_S;  // Sensors / ESP / Link source
  const pStale = !s.hasPi || now - s.piWall > STALE_S;          // Pi source

  const sensors = s.health.filter((h) => h.name !== "loop");
  const loopHz = s.health.find((h) => h.name === "loop")?.hz ?? 0;
  const warn = (b: boolean) => (b ? "warn" : "");
  const ncpu = s.piNcpu || 1;
  const memFrac = s.piMemTotalMb ? s.piMemUsedMb / s.piMemTotalMb : 0;

  return (
    <div className="overview">
      <div className="healthcols">

        <Col title="sensors">
          {sensors.length === 0 && <Tile label="sensors" value="—" cls="stale" />}
          {sensors.map(({ name, hz }) => (
            <Tile key={name} label={name} value={hStale ? "—" : fmtHz(hz)}
                  cls={hStale ? "stale" : hz === 0 ? "dead" : ""} />
          ))}
        </Col>

        <Col title="esp">
          <Tile label="loop" value={hStale ? "—" : fmtHz(loopHz)}
                cls={hStale ? "stale" : loopHz === 0 ? "dead" : ""} />
          <Tile label="max loop" value={hStale || s.maxLoopUs === 0 ? "—" : fmtLoop(s.maxLoopUs)}
                cls={hStale ? "stale" : ""} />
        </Col>

        <Col title="link">
          <Tile label="telem rx" value={hStale ? "—" : `${s.telemFrames1s} /s`}
                cls={hStale ? "stale" : warn(s.telemFrames1s < 40)} />
          <Tile label="tx drops" value={hStale ? "—" : `${s.txDrops}`}
                cls={hStale ? "stale" : warn(s.txDrops > 0)} />
          <Tile label="seq gaps" value={hStale ? "—" : `${s.seqGaps}`}
                cls={hStale ? "stale" : warn(s.seqGaps > 0)} />
          <Tile label="parse fails" value={hStale ? "—" : `${s.parseFails}`}
                cls={hStale ? "stale" : warn(s.parseFails > 0)} />
          <Tile label="max gap" value={hStale ? "—" : `${s.maxFrameGapMs.toFixed(0)} ms`}
                cls={hStale ? "stale" : warn(s.maxFrameGapMs > 100)} />
        </Col>

        <Col title="pi">
          <Tile label="cpu" value={pStale ? "—" : `${s.piCpu.toFixed(0)}%`}
                cls={pStale ? "stale" : s.piCpu > 90 ? "crit" : warn(s.piCpu > 75)} />
          <Tile label="load" value={pStale ? "—" : s.piLoad1m.toFixed(2)}
                cls={pStale ? "stale" : s.piLoad1m >= ncpu * 1.5 ? "crit" : warn(s.piLoad1m >= ncpu * 0.9)} />
          <Tile label="bridge cpu" value={pStale || s.piBridgeCpu < 0 ? "—" : `${s.piBridgeCpu.toFixed(0)}%`}
                cls={pStale ? "stale" : warn(s.piBridgeCpu > 90)} />
          <Tile label="mem" value={pStale || !s.piMemTotalMb ? "—" : `${(memFrac * 100).toFixed(0)}%`}
                cls={pStale ? "stale" : memFrac > 0.9 ? "crit" : warn(memFrac > 0.8)} />
          <Tile label="temp" value={pStale || Number.isNaN(s.piTempC) ? "—" : `${s.piTempC.toFixed(0)} °C`}
                cls={pStale ? "stale" : s.piTempC > 80 ? "crit" : warn(s.piTempC > 70)} />
        </Col>

      </div>
    </div>
  );
}
