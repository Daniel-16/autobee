import { Card, Row } from "./Card";
import { Mission } from "@/lib/types";

function eventBadge(event: string): { cls: string; label: string } {
  const e = (event || "").toLowerCase();
  if (e === "completed")
    return { cls: "border-ok/40 text-ok", label: "completed" };
  if (e === "aborted" || e === "failed")
    return { cls: "border-danger/40 text-danger", label: e };
  if (e === "idle" || e === "")
    return { cls: "border-line2 text-white/40", label: "idle" };
  return { cls: "border-accent2/50 text-accent2", label: e };
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
    <Card title="Mission Status" className={className}>
      <div className="mb-5 flex items-center gap-4">
        <div
          className={`display text-5xl leading-none ${
            idle ? "text-white/30" : "text-white"
          }`}
        >
          {mission.phase && mission.phase !== "—" ? mission.phase : "IDLE"}
        </div>
        <span
          className={`eyebrow rounded-none border px-3 py-1 text-[10px] ${badge.cls}`}
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
