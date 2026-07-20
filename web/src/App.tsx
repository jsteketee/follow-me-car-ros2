// Dashboard shell: title bar, a horizontal metrics strip (wraps on mobile), and the
// top-down canvas filling the rest.
import { FoxgloveProvider } from "./ros/foxglove";
import { LiveProvider } from "./ros/live";
import { ControlPanel } from "./panels/ControlPanel";
import { TopDown2D } from "./panels/TopDown2D";

export function App() {
  return (
    <FoxgloveProvider>
      <LiveProvider>
        <div className="app">
          <div className="topbar">
            <span className="title">Follow-Me Car</span>
          </div>
          <ControlPanel />
          <div className="panel">
            <TopDown2D />
          </div>
        </div>
      </LiveProvider>
    </FoxgloveProvider>
  );
}
