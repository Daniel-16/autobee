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
    <Card title="Mission Log" className={className}>
      <div className="scroll-thin flex max-h-60 flex-col gap-2 overflow-auto pr-1">
        {items.length === 0 && (
          <div className="text-sm font-light italic text-white/30">
            No events yet.
          </div>
        )}
        {items.map((e, i) => (
          <div
            key={i}
            className="border-l-2 border-l-white/15 bg-white/[0.02] px-4 py-2.5"
          >
            <b className="text-sm font-semibold text-white/90">{e.text || "—"}</b>
            {e.reason && (
              <div className="mt-1 text-xs font-light text-white/45">{e.reason}</div>
            )}
          </div>
        ))}
      </div>
    </Card>
  );
}
