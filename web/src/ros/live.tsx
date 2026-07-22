// Shared live-data store: subscribes the TF tree plus the ESP32 actuator/command status,
// and holds them in refs (no React re-render on data). Canvas reads refs in its rAF loop;
// the control panel polls them at a low rate. Keeps high-rate telemetry off React state.
import { createContext, useContext, useRef, MutableRefObject, ReactNode } from "react";
import { useRosTopic } from "./foxglove";
import { Edge, yawFromQuat } from "./tf2d";
import { TFMessage, ActuatorStatus, CommandStatus, NavMode, SensorHealthMsg, TagEstimate } from "./types";
import { ns } from "./topics";

export type LiveStatus = {
  steering: number; throttle: number; pan: number;
  commandMode: string; cmdSpeed: number; cmdHeading: number; cmdAgeMs: number; cmdRejects: number;
  navMode: string;
  tagRangeSigma: number; tagBearingSigma: number; tagCoasting: boolean; tagAgeMs: number;
  health: { name: string; hz: number }[]; healthWall: number; maxLoopUs: number; telemFrames1s: number;
  hasStatus: boolean; hasCmd: boolean; hasNavMode: boolean; hasTagEst: boolean; hasHealth: boolean;
};

export type LiveRefs = {
  treeRef: MutableRefObject<Map<string, Edge>>;
  statusRef: MutableRefObject<LiveStatus>;
};

const LiveCtx = createContext<LiveRefs | null>(null);

export function LiveProvider({ children }: { children: ReactNode }) {
  const treeRef = useRef<Map<string, Edge>>(new Map());
  const statusRef = useRef<LiveStatus>({
    steering: 0, throttle: 0, pan: 0,
    commandMode: "", cmdSpeed: 0, cmdHeading: 0, cmdAgeMs: -1, cmdRejects: 0,
    navMode: "",
    tagRangeSigma: 0, tagBearingSigma: 0, tagCoasting: false, tagAgeMs: -1,
    health: [], healthWall: 0, maxLoopUs: 0, telemFrames1s: 0,
    hasStatus: false, hasCmd: false, hasNavMode: false, hasTagEst: false, hasHealth: false,
  });

  // Fold each transform into the tree, keyed by child frame, stamped with arrival time.
  const onTf = (m: TFMessage) => {
    const now = performance.now() / 1000;
    for (const t of m.transforms ?? []) {
      treeRef.current.set(t.child_frame_id, {
        parent: t.header.frame_id,
        x: t.transform.translation.x,
        y: t.transform.translation.y,
        yaw: yawFromQuat(t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w),
        wall: now,
      });
    }
  };
  useRosTopic("/tf", onTf);
  useRosTopic("/tf_static", onTf);

  useRosTopic(ns("actuator/status"), (m: ActuatorStatus) => {
    const s = statusRef.current;
    s.steering = m.steering; s.throttle = m.throttle; s.pan = m.pan_angle; s.hasStatus = true;
  });
  useRosTopic(ns("command/status"), (m: CommandStatus) => {
    const s = statusRef.current;
    s.commandMode = m.command_mode; s.cmdSpeed = m.cmd_speed; s.cmdHeading = m.cmd_heading;
    s.cmdAgeMs = m.cmd_age_ms; s.cmdRejects = m.cmd_rejects; s.hasCmd = true;
  });
  useRosTopic(ns("nav_mode"), (m: NavMode) => {
    const s = statusRef.current;
    s.navMode = m.mode; s.hasNavMode = true;
  });
  useRosTopic(ns("sensor_health"), (m: SensorHealthMsg) => {
    const s = statusRef.current;
    s.health = Array.from(m.names, (n, i) => ({ name: n, hz: m.rates_hz[i] ?? 0 }));
    s.maxLoopUs = m.max_loop_us ?? 0;
    s.telemFrames1s = m.telem_frames_1s ?? 0;
    s.healthWall = performance.now() / 1000; s.hasHealth = true;
  });
  useRosTopic(ns("fused/tag_pose"), (m: TagEstimate) => {
    const s = statusRef.current;
    s.tagRangeSigma = m.range_sigma; s.tagBearingSigma = m.bearing_sigma;
    s.tagCoasting = m.coasting; s.tagAgeMs = m.age_ms; s.hasTagEst = true;
  });

  return <LiveCtx.Provider value={{ treeRef, statusRef }}>{children}</LiveCtx.Provider>;
}

// Access the shared TF tree and status refs (stable across renders).
export function useLive(): LiveRefs {
  const c = useContext(LiveCtx);
  if (!c) throw new Error("useLive must be used within LiveProvider");
  return c;
}
