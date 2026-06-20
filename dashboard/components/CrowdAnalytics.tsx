import { Card } from "./Card";
import { Analytics } from "@/lib/types";

const DENSITY: Record<string, string> = {
  low: "border-ok/40 text-ok",
  medium: "border-warn/40 text-warn",
  high: "border-danger/40 text-danger",
};

function Stat({ n, label }: { n: React.ReactNode; label: string }) {
  return (
    <div className="flex flex-col">
      <span className="display text-4xl leading-none text-white tabular-nums">
        {n}
      </span>
      <span className="eyebrow mt-2 text-[10px] text-white/40">{label}</span>
    </div>
  );
}

export function CrowdAnalytics({
  a,
  fallbackCount,
  className = "",
}: {
  a: Analytics | null;
  fallbackCount: number;
  className?: string;
}) {
  const density = a?.density_label ?? "low";
  const trend = a?.trend ?? "stable";
  const arrow = trend === "rising" ? "▲" : trend === "falling" ? "▼" : "▬";

  return (
    <Card title="Crowd Analytics" className={className}>
      <div className="grid grid-cols-2 gap-6">
        <Stat n={a?.current_count ?? fallbackCount ?? 0} label="Current" />
        <Stat n={a?.peak_count ?? 0} label="Peak" />
        <Stat n={a?.mean_count ?? 0} label="Mean" />
        <div className="flex flex-col">
          <span
            className={`eyebrow self-start border px-3 py-1 text-[10px] ${
              DENSITY[density] ?? DENSITY.low
            }`}
          >
            {density}
          </span>
          <span className="eyebrow mt-2 text-[10px] text-white/40">Density</span>
        </div>
      </div>
      <div className="mt-5 border-t border-line pt-3 text-xs font-light text-white/45">
        Trend {arrow} {trend} · {a?.rate_per_sec ?? 0} /s
      </div>
    </Card>
  );
}
