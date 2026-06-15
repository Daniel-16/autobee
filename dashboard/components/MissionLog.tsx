import { Card } from "./Card";
import { EventLine } from "@/lib/types";

export function MissionLog({
  items,
  className = "",
}: {
  items: EventLine[];
  className?: string;
}) {
  return (
    <Card title="Mission log" className={className}>
      <div className="scroll-thin flex max-h-60 flex-col gap-2 overflow-auto">
        {items.length === 0 && (
          <div className="text-sm italic text-slate-500">No events yet.</div>
        )}
        {items.map((e, i) => (
          <div key={i} className="rounded-lg bg-panel2 px-3 py-2">
            <b className="text-sm">{e.text || "—"}</b>
            {e.reason && (
              <div className="mt-1 text-xs text-slate-400">{e.reason}</div>
            )}
          </div>
        ))}
      </div>
    </Card>
  );
}
