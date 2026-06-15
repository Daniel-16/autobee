import { Card } from "./Card";
import { Analytics } from "@/lib/types";

const DENSITY: Record<string, string> = {
  low: "bg-emerald-500/15 text-emerald-400",
  medium: "bg-amber-500/20 text-amber-400",
  high: "bg-red-500/15 text-red-400",
};

function Stat({ n, label }: { n: React.ReactNode; label: string }) {
  return (
    <div className="flex flex-col">
      <span className="text-3xl font-extrabold">{n}</span>
      <span className="text-xs uppercase tracking-wide text-slate-400">
        {label}
      </span>
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
    <Card title="Crowd analytics" className={className}>
      <div className="grid grid-cols-2 gap-4">
        <Stat n={a?.current_count ?? fallbackCount ?? 0} label="Current" />
        <Stat n={a?.peak_count ?? 0} label="Peak" />
        <Stat n={a?.mean_count ?? 0} label="Mean" />
        <div className="flex flex-col">
          <span
            className={`self-start rounded-full px-3 py-1 text-xs font-bold ${
              DENSITY[density] ?? DENSITY.low
            }`}
          >
            {density}
          </span>
          <span className="mt-1.5 text-xs uppercase tracking-wide text-slate-400">
            Density
          </span>
        </div>
      </div>
      <div className="mt-4 text-sm text-slate-400">
        trend: {arrow} {trend} · {a?.rate_per_sec ?? 0} /s
      </div>
    </Card>
  );
}
