"use client";

import { useState } from "react";
import { IntentResult } from "@/lib/useBridge";

const SUGGESTIONS = [
  "do a quick check of the area",
  "check if the convocation ground is overcrowded",
  "survey the library and tell me how busy it is",
  "scan the main gate",
  "look around me",
];

export function CommandBar({
  onSend,
}: {
  onSend: (text: string) => Promise<IntentResult>;
}) {
  const [text, setText] = useState("");
  const [msg, setMsg] = useState<{ text: string; ok: boolean } | null>(null);
  const [busy, setBusy] = useState(false);

  async function submit(value?: string) {
    const t = (value ?? text).trim();
    if (!t || busy) return;
    setBusy(true);
    const r = await onSend(t);
    setBusy(false);
    if (r.ok) {
      setMsg({ text: `✓ sent: ${r.text}`, ok: true });
      setText("");
    } else {
      setMsg({ text: `✗ ${r.error || "failed"}`, ok: false });
    }
  }

  return (
    <div className="mb-5 rounded-2xl border border-line bg-panel p-5">
      <h2 className="mb-3 text-xs font-semibold uppercase tracking-wide text-slate-400">
        Give the drone a command
      </h2>
      <div className="flex gap-3">
        <input
          value={text}
          onChange={(e) => setText(e.target.value)}
          onKeyDown={(e) => e.key === "Enter" && submit()}
          placeholder="e.g. check if the convocation ground is overcrowded"
          spellCheck={false}
          autoComplete="off"
          className="flex-1 rounded-xl border border-line bg-bg px-4 py-3 text-[15px] outline-none transition focus:border-accent2"
        />
        <button
          onClick={() => submit()}
          disabled={busy}
          className="rounded-xl bg-accent px-6 text-[15px] font-bold text-slate-900 transition active:translate-y-px disabled:opacity-60"
        >
          {busy ? "…" : "Send ▸"}
        </button>
      </div>

      <div className="mt-3 flex flex-wrap gap-2">
        {SUGGESTIONS.map((s) => (
          <button
            key={s}
            onClick={() => {
              setText(s);
              submit(s);
            }}
            className="rounded-full border border-line bg-panel2 px-3 py-1.5 text-[13px] text-slate-400 transition hover:border-accent2 hover:text-slate-100"
          >
            {s}
          </button>
        ))}
      </div>

      {msg && (
        <div
          className={`mt-3 min-h-[18px] text-sm ${
            msg.ok ? "text-emerald-400" : "text-red-400"
          }`}
        >
          {msg.text}
        </div>
      )}
    </div>
  );
}
