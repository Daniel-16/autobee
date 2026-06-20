import { Card } from "./Card";
import { Anomaly } from "@/lib/types";

const SEV: Record<string, { border: string; chip: string }> = {
  low: { border: "border-l-accent2", chip: "border-accent2/50 text-accent2" },
  medium: { border: "border-l-warn", chip: "border-warn/50 text-warn" },
  high: { border: "border-l-danger", chip: "border-danger/50 text-danger" },
};

export function Anomalies({
  items,
  className = "",
}: {
  items: Anomaly[];
  className?: string;
}) {
  return (
    <Card title="Anomaly Alerts" className={className}>
      <div className="scroll-thin flex max-h-60 flex-col gap-2 overflow-auto pr-1">
        {items.length === 0 && (
          <div className="text-sm font-light italic text-white/30">
            No anomalies detected.
          </div>
        )}
        {items.map((x, i) => {
          const sev = SEV[x.severity ?? "low"] ?? SEV.low;
          return (
            <div
              key={i}
              className={`border-l-2 bg-white/[0.02] px-4 py-2.5 ${sev.border}`}
            >
              <div className="flex items-center justify-between">
                <b className="text-sm font-semibold tracking-wide text-white/90">
                  {x.type ?? "ANOMALY"}
                </b>
                <span className={`eyebrow border px-2 py-0.5 text-[9px] ${sev.chip}`}>
                  {x.severity ?? "low"}
                </span>
              </div>
              {x.description && (
                <div className="mt-1 text-xs font-light text-white/45">
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
