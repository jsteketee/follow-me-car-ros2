import { defineConfig } from "vite";
import react from "@vitejs/plugin-react";

// Static SPA served on the LAN. host:true binds all interfaces; allowedHosts:true disables
// Vite's Host-header allowlist so followme-pi.local / the Pi's IP / phones can all load it.
// (That check guards against DNS-rebinding; fine to drop for a trusted-LAN robot dashboard.)
export default defineConfig({
  plugins: [react()],
  server: { host: true, port: 5173, allowedHosts: true },
  preview: { host: true, port: 8080, allowedHosts: true },
  build: { outDir: "dist" },
});
