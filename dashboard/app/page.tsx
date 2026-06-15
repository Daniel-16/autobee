"use client";

import { useBridge } from "@/lib/useBridge";
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
      <header className="flex items-center justify-between border-b border-line bg-panel px-6 py-4">
        <div className="flex items-center gap-2 text-xl font-bold">
          <span className="text-2xl">🐝</span> AutoBee
          <span className="ml-1.5 text-sm font-normal text-slate-400">
            Mission Control
          </span>
        </div>
        <div className="flex items-center gap-4">
          <button
            onClick={onAbort}
            className="rounded-lg bg-red-600 px-4 py-2 text-sm font-bold text-white transition hover:bg-red-500 active:translate-y-px"
            title="Stop the mission and land the drone immediately"
          >
            ⬇ EMERGENCY LAND
          </button>
          <div className="flex items-center gap-2 text-sm text-slate-400">
            <span
              className={`h-2.5 w-2.5 rounded-full transition ${
                connected
                  ? "bg-emerald-500 shadow-[0_0_8px] shadow-emerald-500"
                  : "bg-red-500 shadow-[0_0_8px] shadow-red-500"
              }`}
            />
            {connected ? "live" : "reconnecting…"}
          </div>
        </div>
      </header>

      <main className="mx-auto max-w-7xl px-5 py-6">
        <CommandBar onSend={sendIntent} />

        <div className="grid grid-cols-12 gap-4">
          <MissionStatus
            mission={state.mission}
            className="col-span-12 lg:col-span-8"
          />
          <Telemetry
            t={state.telemetry}
            className="col-span-12 lg:col-span-4"
          />

          {hasCam && (
            <Camera url={bridgeUrl} className="col-span-12 lg:col-span-6" />
          )}
          <CrowdAnalytics
            a={state.analytics}
            fallbackCount={state.person_count}
            className="col-span-12 lg:col-span-6"
          />

          <Anomalies
            items={state.anomalies}
            className="col-span-12 lg:col-span-6"
          />
          <MissionLog
            items={state.events}
            className="col-span-12 lg:col-span-6"
          />

          <Report text={state.report} className="col-span-12" />
          <Plan plan={state.plan} className="col-span-12" />
        </div>
      </main>
    </>
  );
}
