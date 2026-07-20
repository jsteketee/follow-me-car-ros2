# Follow-Me Car — Web Dashboard

Static React SPA that talks directly to the car's `foxglove_bridge` over the Foxglove
ws-protocol. No backend of its own — the Pi's bridge is the data source. Excluded from
the colcon build via `COLCON_IGNORE`.

**Features:** a top-down 2D view of the car (`base_link`), the fused tag estimate
(`tag_est_link`), and the follow controller's committed goal (`nav_goal`, a green target
reticle), all resolved into the `odom` frame from `/tf` + `/tf_static`, with an odometry
trail. Mirrors what the Foxglove 3D panel shows, flattened to a bird's-eye canvas.

## Requires on the Pi

- `foxglove_bridge` running (it's already in `bringup.launch.py`, port **8765**).
- Full bringup for live data — `/tf` needs the ESP32 + `serial_bridge` + `pose_estimator`
  (`odom → base_link`) and `tag_estimator` (`base_link → tag_est_link`). The `nav_goal`
  marker appears only while `nav_controller` is running (it is not in `bringup.launch.py`).

## Develop

```bash
cd web
npm install                     # first time
npm run dev                     # Vite dev server, reachable on the LAN (host: true)
```

The dashboard auto-targets `ws://<page-host>:8765`. When developing off-box (e.g. from a
Mac), point it at the car explicitly:

- URL query: `http://localhost:5173/?bridge=followme-pi.local`
- or env: `VITE_BRIDGE_URL=ws://followme-pi.local:8765 npm run dev`

## Build & serve from the Pi

```bash
cd web
npm run build                   # -> dist/  (gitignored)
npm run serve                   # static-serve dist/ on :8080 (or use any static server)
```

Then load `http://followme-pi.local:8080/` from a phone/laptop on the same network.

## Verify the data path without a browser

```bash
npm run probe                   # or: node scripts/probe.mjs ws://followme-pi.local:8765
```

Connects to the bridge, lists advertised topics, and decodes a few `/tf` transforms —
confirms the ws-protocol + CDR decode path works against live data.
