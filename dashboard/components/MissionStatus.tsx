import { Card, Row } from "./Card";
import { Mission } from "@/lib/types";

function eventBadge(event: string): { cls: string; label: string } {
  const e = (event || "").toLowerCase();
  if (e === "completed")
    return { cls: "bg-emerald-500/15 text-emerald-400", label: "completed" };
  if (e === "aborted" || e === "failed")
    return { cls: "bg-red-500/15 text-red-400", label: e };
  if (e === "idle" || e === "")
    return { cls: "bg-slate-500/15 text-slate-400", label: "idle" };
  return { cls: "bg-blue-500/15 text-accent2", label: e };
}

export function MissionStatus({
  mission,
  className = "",
}: {
  mission: Mission;
  className?: string;
}) {
  const idle = !mission.event || mission.event === "idle";
  const badge = eventBadge(mission.event);
  const dash = (v: string | null) => (v ? v : "—");

  return (
    <Card title="Mission status" className={className}>
      <div className="mb-4 flex items-center gap-4">
        <div
          className={`text-3xl font-extrabold tracking-wide ${
            idle ? "text-slate-500" : "text-slate-100"
          }`}
        >
          {mission.phase || "IDLE"}
        </div>
        <span
          className={`rounded-full px-3 py-1 text-xs font-bold uppercase tracking-wide ${badge.cls}`}
        >
          {badge.label}
        </span>
      </div>
      <Row label="Intent" value={dash(mission.intent)} />
      <Row label="Landmark" value={dash(mission.landmark)} />
      <Row label="Note" value={dash(mission.reason)} />
    </Card>
  );
}
