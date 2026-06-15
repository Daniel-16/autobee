import { Card } from "./Card";
import { Anomaly } from "@/lib/types";

const SEV_BORDER: Record<string, string> = {
  low: "border-l-accent2",
  medium: "border-l-amber-500",
  high: "border-l-red-500",
};

export function Anomalies({
  items,
  className = "",
}: {
  items: Anomaly[];
  className?: string;
}) {
  return (
    <Card title="Anomaly alerts" className={className}>
      <div className="scroll-thin flex max-h-60 flex-col gap-2 overflow-auto">
        {items.length === 0 && (
          <div className="text-sm italic text-slate-500">No anomalies.</div>
        )}
        {items.map((x, i) => {
          const sev = x.severity ?? "low";
          return (
            <div
              key={i}
              className={`rounded-lg border-l-[3px] bg-panel2 px-3 py-2 ${
                SEV_BORDER[sev] ?? SEV_BORDER.low
              }`}
            >
              <div className="flex items-center justify-between">
                <b className="text-sm">{x.type ?? "ANOMALY"}</b>
                <span className="rounded-full bg-blue-500/15 px-2 py-0.5 text-xs font-bold uppercase text-accent2">
                  {sev}
                </span>
              </div>
              {x.description && (
                <div className="mt-1 text-xs text-slate-400">
                  {x.description}
                </div>
              )}
            </div>
          );
        })}
      </div>
    </Card>
  );
}
