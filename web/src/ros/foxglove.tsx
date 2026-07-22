// Foxglove ws-protocol connection to foxglove_bridge: one WebSocket, topic subscribe with
// on-the-fly CDR decode (schema parsed from the channel advert), and ROS service calls
// (CDR-encoded from the service advert's schema). Exposes subscribe + callService hooks.
import { createContext, useCallback, useContext, useEffect, useRef, useState, ReactNode } from "react";
import { FoxgloveClient } from "@foxglove/ws-protocol";
import { parse as parseRos2 } from "@foxglove/rosmsg";
import { MessageReader, MessageWriter } from "@foxglove/rosmsg2-serialization";

export type ConnStatus = "connecting" | "connected" | "disconnected";
type MsgCb = (msg: any) => void;
type Unsub = () => void;
type CallService = (name: string, request: any) => Promise<any>;
type PendingCall = {
  svc: any;
  resolve: (v: any) => void;
  reject: (e: Error) => void;
  timer: ReturnType<typeof setTimeout>;
};

const SERVICE_CALL_TIMEOUT_MS = 3000;

// Offer both handshake tokens: the new Rust SDK bridge (libfoxglove.so) requires
// "foxglove.sdk.v1" and rejects the classic token; older foxglove_bridge wants the classic
// one. The message protocol is identical, so @foxglove/ws-protocol decodes either.
const SUBPROTOCOLS = ["foxglove.sdk.v1", FoxgloveClient.SUPPORTED_SUBPROTOCOL];

const SubscribeCtx = createContext<(topic: string, cb: MsgCb) => Unsub>(() => () => {});
const StatusCtx = createContext<ConnStatus>("disconnected");
const CallServiceCtx = createContext<CallService>(() => Promise.reject(new Error("no bridge")));

// Resolve the bridge WebSocket URL: ?bridge= query, then VITE_BRIDGE_URL, else this host:8765.
export function bridgeUrl(): string {
  const q = new URLSearchParams(location.search).get("bridge");
  if (q) return q.startsWith("ws") ? q : `ws://${q}:8765`;
  const env = (import.meta as any).env?.VITE_BRIDGE_URL as string | undefined;
  if (env) return env;
  const host = location.hostname || "followme-pi.local";
  return `ws://${host}:8765`;
}

export function FoxgloveProvider({ children }: { children: ReactNode }) {
  const [status, setStatus] = useState<ConnStatus>("connecting");

  // Caller subscriptions, kept across reconnects: topic -> set of callbacks.
  const subsRef = useRef<Map<string, Set<MsgCb>>>(new Map());
  // Per-connection bridge state, rebuilt on every (re)connect.
  const clientRef = useRef<any>(null);
  const channelByTopic = useRef<Map<string, any>>(new Map());
  const readerByChannel = useRef<Map<number, MessageReader>>(new Map());
  const subIdToTopic = useRef<Map<number, string>>(new Map());
  const activeSubByTopic = useRef<Map<string, number>>(new Map());
  // Service-call state: adverts by name, in-flight calls by callId.
  const servicesByName = useRef<Map<string, any>>(new Map());
  const pendingCalls = useRef<Map<number, PendingCall>>(new Map());
  const nextCallId = useRef(1);

  // Subscribe to a topic on the wire once its channel is advertised (idempotent).
  const wireSubscribe = useCallback((topic: string) => {
    if (activeSubByTopic.current.has(topic)) return;
    const ch = channelByTopic.current.get(topic);
    if (!ch || !clientRef.current) return;
    const subId = clientRef.current.subscribe(ch.id);
    activeSubByTopic.current.set(topic, subId);
    subIdToTopic.current.set(subId, topic);
  }, []);

  // Call a ROS service by name: CDR-encode the request from the advertised schema,
  // resolve with the decoded response (or reject on timeout/disconnect/no advert).
  const callService = useCallback((name: string, request: any): Promise<any> => {
    const client = clientRef.current;
    const svc = servicesByName.current.get(name);
    if (!client || !svc) return Promise.reject(new Error(`service not advertised: ${name}`));
    let data: Uint8Array;
    try {
      const writer = new MessageWriter(parseRos2(svc.request?.schema ?? svc.requestSchema, { ros2: true }));
      data = writer.writeMessage(request);
    } catch (e) {
      return Promise.reject(new Error(`request encode failed for ${name}: ${e}`));
    }
    const callId = nextCallId.current++;
    return new Promise((resolve, reject) => {
      const timer = setTimeout(() => {
        pendingCalls.current.delete(callId);
        reject(new Error(`service call timed out: ${name}`));
      }, SERVICE_CALL_TIMEOUT_MS);
      pendingCalls.current.set(callId, { svc, resolve, reject, timer });
      client.sendServiceCallRequest({ serviceId: svc.id, callId, encoding: "cdr", data });
    });
  }, []);

  // Stable public subscribe: register the callback, bind the wire subscription if possible.
  const subscribe = useCallback((topic: string, cb: MsgCb): Unsub => {
    let set = subsRef.current.get(topic);
    if (!set) { set = new Set(); subsRef.current.set(topic, set); }
    set.add(cb);
    wireSubscribe(topic);
    return () => {
      const s = subsRef.current.get(topic);
      if (s) { s.delete(cb); if (s.size === 0) subsRef.current.delete(topic); }
    };
  }, [wireSubscribe]);

  useEffect(() => {
    let closed = false;
    let retry: ReturnType<typeof setTimeout> | undefined;

    const connect = () => {
      setStatus("connecting");
      const ws = new WebSocket(bridgeUrl(), SUBPROTOCOLS);
      const client: any = new FoxgloveClient({ ws });
      clientRef.current = client;

      client.on("open", () => { if (!closed) setStatus("connected"); });
      client.on("error", () => { /* close handler drives reconnect */ });

      client.on("close", () => {
        if (closed) return;
        setStatus("disconnected");
        channelByTopic.current.clear();
        readerByChannel.current.clear();
        subIdToTopic.current.clear();
        activeSubByTopic.current.clear();
        servicesByName.current.clear();
        for (const p of pendingCalls.current.values()) {
          clearTimeout(p.timer);
          p.reject(new Error("bridge disconnected"));
        }
        pendingCalls.current.clear();
        clientRef.current = null;
        retry = setTimeout(connect, 1500);
      });

      client.on("advertiseServices", (svcs: any[]) => {
        for (const s of svcs) servicesByName.current.set(s.name, s);
      });

      client.on("unadvertiseServices", (ids: number[]) => {
        for (const [n, s] of servicesByName.current) if (ids.includes(s.id)) servicesByName.current.delete(n);
      });

      // Decode a service response with the advert's response schema, settle the pending call.
      client.on("serviceCallResponse", (res: any) => {
        const p = pendingCalls.current.get(res.callId);
        if (!p) return;
        pendingCalls.current.delete(res.callId);
        clearTimeout(p.timer);
        try {
          const defs = parseRos2(p.svc.response?.schema ?? p.svc.responseSchema, { ros2: true });
          p.resolve(new MessageReader(defs).readMessage(res.data));
        } catch (e) {
          p.reject(new Error(`response decode failed: ${e}`));
        }
      });

      // Each advertised channel carries its schema text; build a CDR reader from it.
      client.on("advertise", (channels: any[]) => {
        for (const ch of channels) {
          channelByTopic.current.set(ch.topic, ch);
          try {
            const defs = parseRos2(ch.schema, { ros2: true });
            readerByChannel.current.set(ch.id, new MessageReader(defs));
          } catch (e) {
            console.warn(`schema parse failed for ${ch.topic} (${ch.schemaName})`, e);
          }
        }
        for (const topic of subsRef.current.keys()) wireSubscribe(topic);
      });

      client.on("unadvertise", (ids: number[]) => {
        for (const id of ids) {
          for (const [t, ch] of channelByTopic.current) if (ch.id === id) channelByTopic.current.delete(t);
        }
      });

      client.on("message", (ev: any) => {
        const topic = subIdToTopic.current.get(ev.subscriptionId);
        if (!topic) return;
        const ch = channelByTopic.current.get(topic);
        const reader = ch && readerByChannel.current.get(ch.id);
        if (!reader) return;
        let msg: any;
        try { msg = reader.readMessage(ev.data); } catch { return; }
        const cbs = subsRef.current.get(topic);
        if (cbs) for (const cb of cbs) cb(msg);
      });
    };

    connect();
    return () => {
      closed = true;
      if (retry) clearTimeout(retry);
      try { clientRef.current?.close(); } catch { /* already closing */ }
    };
  }, [wireSubscribe]);

  return (
    <StatusCtx.Provider value={status}>
      <CallServiceCtx.Provider value={callService}>
        <SubscribeCtx.Provider value={subscribe}>{children}</SubscribeCtx.Provider>
      </CallServiceCtx.Provider>
    </StatusCtx.Provider>
  );
}

// Call a ROS service through the bridge: useCallService()(name, request) -> Promise<response>.
export function useCallService(): CallService {
  return useContext(CallServiceCtx);
}

// Subscribe to a ROS topic for the component's lifetime; the latest callback is always used.
export function useRosTopic(topic: string, cb: MsgCb) {
  const subscribe = useContext(SubscribeCtx);
  const cbRef = useRef(cb);
  cbRef.current = cb;
  useEffect(() => subscribe(topic, (m) => cbRef.current(m)), [subscribe, topic]);
}

// Current connection status for UI indicators.
export function useConnStatus(): ConnStatus {
  return useContext(StatusCtx);
}
