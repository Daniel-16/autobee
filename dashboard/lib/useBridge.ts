"use client";

import { useCallback, useEffect, useRef, useState } from "react";
import { BridgeState, EMPTY_STATE } from "./types";

const BRIDGE =
  process.env.NEXT_PUBLIC_BRIDGE_URL || "http://localhost:8080";

export interface IntentResult {
  ok: boolean;
  text?: string;
  error?: string;
}

/**
 * Connects to the ROS bridge: pulls the initial snapshot, then keeps a live
 * EventSource (SSE) stream open. Reconnects automatically on drop.
 */
export function useBridge() {
  const [state, setState] = useState<BridgeState>(EMPTY_STATE);
  const [connected, setConnected] = useState(false);
  const esRef = useRef<EventSource | null>(null);

  useEffect(() => {
    let cancelled = false;

    fetch(`${BRIDGE}/state`)
      .then((r) => r.json())
      .then((s) => !cancelled && setState(s))
      .catch(() => {});

    const es = new EventSource(`${BRIDGE}/events`);
    esRef.current = es;
    es.onopen = () => setConnected(true);
    es.onerror = () => setConnected(false); // EventSource auto-retries
    es.onmessage = (e) => {
      try {
        setState(JSON.parse(e.data) as BridgeState);
      } catch {
        /* ignore malformed frame */
      }
    };

    return () => {
      cancelled = true;
      es.close();
    };
  }, []);

  const sendIntent = useCallback(
    async (text: string): Promise<IntentResult> => {
      try {
        const r = await fetch(`${BRIDGE}/intent`, {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({ text }),
        });
        return (await r.json()) as IntentResult;
      } catch {
        return { ok: false, error: "network error" };
      }
    },
    []
  );

  const sendAbort = useCallback(async (): Promise<boolean> => {
    try {
      const r = await fetch(`${BRIDGE}/abort`, { method: "POST" });
      return r.ok;
    } catch {
      return false;
    }
  }, []);

  return { state, connected, sendIntent, sendAbort, bridgeUrl: BRIDGE };
}
