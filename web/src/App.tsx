// Dashboard shell: title bar with view tabs, then the active view — "control" (metrics
// strip + top-down canvas) or "overview" (vehicle status). The rosout overlay and all
// topic subscriptions stay mounted across tab switches.
import { useState } from "react";
import { FoxgloveProvider, useConnStatus } from "./ros/foxglove";
import { LiveProvider } from "./ros/live";
import { ControlPanel } from "./panels/ControlPanel";
import { TopDown2D } from "./panels/TopDown2D";
import { OverviewPanel } from "./panels/OverviewPanel";
import { RosoutOverlay } from "./panels/RosoutOverlay";

const VIEWS = ["control", "overview"] as const;
type View = (typeof VIEWS)[number];

// Connection status dot + label, right-aligned in the top bar.
function ConnStatus() {
  const status = useConnStatus();
  return <span className="conn"><span className={`dot ${status}`} />{status}</span>;
}

// Restore the view from the URL hash so a refresh keeps the tab.
function initialView(): View {
  const h = location.hash.replace("#", "");
  return (VIEWS as readonly string[]).includes(h) ? (h as View) : "control";
}

export function App() {
  const [view, setView] = useState<View>(initialView);
  const select = (v: View) => { setView(v); location.hash = v; };

  return (
    <FoxgloveProvider>
      <LiveProvider>
        <div className="app">
          <div className="topbar">
            <span className="title">Follow-Me Car</span>
            <div className="tabs">
              {VIEWS.map((v) => (
                <button key={v} className={`tab ${view === v ? "active" : ""}`} onClick={() => select(v)}>
                  {v}
                </button>
              ))}
            </div>
            <ConnStatus />
          </div>
          {view === "control" && <ControlPanel />}
          <div className="panel">
            {view === "control" ? <TopDown2D /> : <OverviewPanel />}
            <RosoutOverlay />
          </div>
        </div>
      </LiveProvider>
    </FoxgloveProvider>
  );
}
