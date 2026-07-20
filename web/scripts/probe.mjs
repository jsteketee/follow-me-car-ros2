// Headless connection + decode check against foxglove_bridge (no browser).
// Lists advertised topics and decodes the first few /tf transforms to prove the
// ws-protocol + CDR path works end-to-end. Usage: node scripts/probe.mjs [ws://host:8765]
// These packages ship as CommonJS; require() sidesteps Node's ESM named-export detection.
// (The Vite app imports them normally — esbuild handles the interop there.)
import { createRequire } from "node:module";
const require = createRequire(import.meta.url);
const { FoxgloveClient } = require("@foxglove/ws-protocol");
const { parse: parseRos2 } = require("@foxglove/rosmsg");
const { MessageReader } = require("@foxglove/rosmsg2-serialization");
// The `ws` package (not Node's native WebSocket) negotiates the subprotocol the bridge
// requires. Browsers do this natively, so this is a probe-only concern.
const WebSocket = require("ws");

const url = process.argv[2] || "ws://followme-pi.local:8765";
console.log(`connecting to ${url} …`);

// New Rust SDK bridge requires "foxglove.sdk.v1"; classic bridge wants the old token. Offer both.
const ws = new WebSocket(url, ["foxglove.sdk.v1", FoxgloveClient.SUPPORTED_SUBPROTOCOL]);
const client = new FoxgloveClient({ ws });

const channels = new Map();
const readers = new Map();
const subToTopic = new Map();
let tfSeen = 0;

client.on("open", () => console.log("open"));
client.on("error", (e) => console.error("error:", e?.message ?? e));
client.on("close", () => { console.log("closed"); process.exit(0); });

client.on("advertise", (chs) => {
  for (const ch of chs) {
    channels.set(ch.id, ch);
    console.log(`  advertised: ${ch.topic}  [${ch.schemaName}]`);
    try { readers.set(ch.id, new MessageReader(parseRos2(ch.schema, { ros2: true }))); }
    catch (e) { console.warn(`  parse fail ${ch.topic}:`, e.message); }
    if (ch.topic === "/tf" || ch.topic === "/tf_static") {
      subToTopic.set(client.subscribe(ch.id), ch);
    }
  }
});

client.on("message", ({ subscriptionId, data }) => {
  const ch = subToTopic.get(subscriptionId);
  if (!ch) return;
  const reader = readers.get(ch.id);
  if (!reader) return;
  const msg = reader.readMessage(new Uint8Array(data.buffer, data.byteOffset, data.byteLength));
  for (const t of msg.transforms ?? []) {
    console.log(`  ${ch.topic}: ${t.header.frame_id} -> ${t.child_frame_id}  (${t.transform.translation.x.toFixed(3)}, ${t.transform.translation.y.toFixed(3)})`);
  }
  if (++tfSeen >= 6) { console.log("decode OK — exiting"); client.close(); }
});

setTimeout(() => { console.log("timeout (no tf in 10s — is bringup running?)"); process.exit(1); }, 10000);
