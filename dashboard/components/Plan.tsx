import { Card } from "./Card";

export function Plan({
  plan,
  className = "",
}: {
  plan: unknown;
  className?: string;
}) {
  return (
    <Card title="LLM plan (raw)" className={className}>
      <pre className="scroll-thin max-h-64 overflow-auto whitespace-pre-wrap break-words font-mono text-xs text-slate-400">
        {plan ? JSON.stringify(plan, null, 2) : "—"}
      </pre>
    </Card>
  );
}
