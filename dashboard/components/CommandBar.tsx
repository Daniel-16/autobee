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
    <div className="relative overflow-hidden border border-line bg-panel/70 p-6 backdrop-blur-sm sm:p-8">
      <span className="pointer-events-none absolute inset-x-0 top-0 h-px bg-gradient-to-r from-transparent via-white/30 to-transparent" />
      <h2 className="eyebrow mb-5 text-[10px] text-white/45">
        Command Input · Natural Language
      </h2>
      <div className="flex flex-col gap-3 sm:flex-row">
        <input
          value={text}
          onChange={(e) => setText(e.target.value)}
          onKeyDown={(e) => e.key === "Enter" && submit()}
          placeholder="Speak the mission — e.g. check if the convocation ground is overcrowded"
          spellCheck={false}
          autoComplete="off"
          className="flex-1 border border-line bg-black/40 px-4 py-3.5 text-[15px] font-light text-white outline-none transition placeholder:text-white/30 focus:border-white/40"
        />
        <button
          onClick={() => submit()}
          disabled={busy}
          className="eyebrow border border-white/70 px-8 py-3.5 text-[11px] text-white transition-colors duration-300 hover:bg-white hover:text-black disabled:opacity-40"
        >
          {busy ? "Sending…" : "Engage"}
        </button>
      </div>

      <div className="mt-4 flex flex-wrap gap-2">
        {SUGGESTIONS.map((s) => (
          <button
            key={s}
            onClick={() => {
              setText(s);
              submit(s);
            }}
            className="border border-line px-3 py-1.5 text-[12px] font-light text-white/45 transition-colors duration-300 hover:border-white/40 hover:text-white"
          >
            {s}
          </button>
        ))}
      </div>

      {msg && (
        <div
          className={`mt-4 min-h-[18px] text-sm ${
            msg.ok ? "text-ok" : "text-danger"
          }`}
        >
          {msg.text}
        </div>
      )}
    </div>
  );
}
