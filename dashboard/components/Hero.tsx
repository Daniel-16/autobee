"use client";

import { DroneModel } from "./DroneModel";

/**
 * Full-bleed cinematic hero — SpaceX/Starlink open every page on the product
 * against black with a wide-tracked wordmark and minimal supporting copy.
 */
export function Hero({
  connected,
  phase,
}: {
  connected: boolean;
  phase: string;
}) {
  return (
    <section className="relative flex min-h-screen flex-col items-center justify-center overflow-hidden px-6 pb-24 pt-24 text-center">
      {/* slow telemetry sweep */}
      <div className="pointer-events-none absolute inset-x-0 top-0 h-px bg-gradient-to-r from-transparent via-white/40 to-transparent hero-scan" />

      {/* eyebrow */}
      <p className="eyebrow mb-6 text-[11px] text-accent2 sm:text-xs">
        Autonomous Aerial Surveillance
      </p>

      {/* wordmark */}
      <h1 className="display text-[18vw] leading-[0.82] text-white sm:text-[14vw] lg:text-[11rem]">
        AUTO<span className="text-white/55">BEE</span>
      </h1>

      <p className="mt-5 max-w-xl text-sm font-light leading-relaxed text-white/55 sm:text-base">
        Speak the mission. The drone plans, flies and reports on its own.
        Three layers of closed-loop autonomy — the human stays in plain English.
      </p>

      {/* live status chip */}
      <div className="mt-6 flex items-center gap-3 text-xs">
        <span className="relative flex h-2.5 w-2.5">
          {connected && (
            <span className="absolute inline-flex h-full w-full rounded-full bg-ok/60 animate-pulse-ring" />
          )}
          <span
            className={`relative inline-flex h-2.5 w-2.5 rounded-full ${
              connected ? "bg-ok" : "bg-danger"
            }`}
          />
        </span>
        <span className="eyebrow text-[10px] tracking-wider2 text-white/70">
          {connected ? `LINK ESTABLISHED · ${phase || "STANDBY"}` : "ACQUIRING LINK…"}
        </span>
      </div>

      {/* the aircraft */}
      <div className="relative mt-4 w-full max-w-2xl">
        <DroneModel className="mx-auto w-full max-w-[520px]" />
      </div>

      {/* scroll cue */}
      <a
        href="#control"
        className="absolute bottom-8 left-1/2 -translate-x-1/2 flex flex-col items-center gap-2 text-white/40 transition hover:text-white"
      >
        <span className="eyebrow text-[10px]">Mission Control</span>
        <svg width="20" height="28" viewBox="0 0 20 28" className="scroll-cue">
          <path d="M10 2 L10 20 M3 13 L10 21 L17 13" stroke="currentColor" strokeWidth="1.5" fill="none" />
        </svg>
      </a>
    </section>
  );
}
