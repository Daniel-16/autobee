"use client";

import { useBridge } from "@/lib/useBridge";
import { Hero } from "@/components/Hero";
import { Reveal } from "@/components/Reveal";
import { CommandBar } from "@/components/CommandBar";
import { MissionStatus } from "@/components/MissionStatus";
import { Telemetry } from "@/components/Telemetry";
import { Camera } from "@/components/Camera";
import { CrowdAnalytics } from "@/components/CrowdAnalytics";
import { Anomalies } from "@/components/Anomalies";
import { MissionLog } from "@/components/MissionLog";
import { Report } from "@/components/Report";
import { Plan } from "@/components/Plan";

export default function Page() {
  const { state, connected, sendIntent, sendAbort, bridgeUrl } = useBridge();
  const hasCam = state.camera_available;

  async function onAbort() {
    if (confirm("Emergency land the drone now?")) await sendAbort();
  }

  return (
    <>
      {/* ── Fixed top nav (transparent, wide uppercase — SpaceX house style) */}
      <header className="fixed inset-x-0 top-0 z-50 flex items-center justify-between px-5 py-4 sm:px-8">
        <a href="#" className="display text-lg tracking-wider2 text-white">
          AUTOBEE
        </a>
        <div className="flex items-center gap-5">
          <div className="hidden items-center gap-2 text-xs sm:flex">
            <span
              className={`h-2 w-2 rounded-full ${
                connected ? "bg-ok shadow-[0_0_10px] shadow-ok" : "bg-danger shadow-[0_0_10px] shadow-danger"
              }`}
            />
            <span className="eyebrow text-[10px] text-white/60">
              {connected ? "Live" : "Reconnecting"}
            </span>
          </div>
          <button
            onClick={onAbort}
            className="eyebrow border border-danger/60 px-4 py-2 text-[10px] text-danger transition-colors duration-300 hover:bg-danger hover:text-black"
            title="Stop the mission and land the drone immediately"
          >
            Emergency Land
          </button>
        </div>
      </header>

      {/* ── Cinematic hero ───────────────────────────────────────────── */}
      <Hero connected={connected} phase={state.mission.phase} />

      {/* ── Mission control ──────────────────────────────────────────── */}
      <main
        id="control"
        className="mx-auto max-w-7xl scroll-mt-20 px-5 pb-28 sm:px-8"
      >
        <Reveal className="mb-10 border-t border-line pt-8">
          <p className="eyebrow text-[11px] text-accent2">Section 01</p>
          <h2 className="display mt-1 text-4xl text-white sm:text-5xl">
            Mission Control
          </h2>
        </Reveal>

        <Reveal className="mb-6">
          <CommandBar onSend={sendIntent} />
        </Reveal>

        <div className="grid grid-cols-12 gap-4">
          <Reveal className="col-span-12 lg:col-span-8">
            <MissionStatus mission={state.mission} className="h-full" />
          </Reveal>
          <Reveal className="col-span-12 lg:col-span-4" delay={80}>
            <Telemetry t={state.telemetry} className="h-full" />
          </Reveal>

          {hasCam && (
            <Reveal className="col-span-12 lg:col-span-6">
              <Camera url={bridgeUrl} className="h-full" />
            </Reveal>
          )}
          <Reveal
            className={`col-span-12 ${hasCam ? "lg:col-span-6" : ""}`}
            delay={80}
          >
            <CrowdAnalytics
              a={state.analytics}
              fallbackCount={state.person_count}
              className="h-full"
            />
          </Reveal>

          <Reveal className="col-span-12 lg:col-span-6">
            <Anomalies items={state.anomalies} className="h-full" />
          </Reveal>
          <Reveal className="col-span-12 lg:col-span-6" delay={80}>
            <MissionLog items={state.events} className="h-full" />
          </Reveal>

          <Reveal className="col-span-12">
            <Report text={state.report} />
          </Reveal>
          <Reveal className="col-span-12">
            <Plan plan={state.plan} />
          </Reveal>
        </div>

        <footer className="mt-20 border-t border-line pt-8 text-center">
          <p className="eyebrow text-[10px] text-white/30">
            AutoBee · Autonomous Aerial Surveillance System
          </p>
        </footer>
      </main>
    </>
  );
}
