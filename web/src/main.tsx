// App entry: mount React (no StrictMode, so the single WebSocket connection isn't double-opened in dev).
import { createRoot } from "react-dom/client";
import { App } from "./App";
import "./styles.css";

createRoot(document.getElementById("root")!).render(<App />);
