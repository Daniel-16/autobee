import { Card } from "./Card";

export function Plan({
  plan,
  className = "",
}: {
  plan: unknown;
  className?: string;
}) {
  return (
    <Card title="LLM Plan · Raw" className={className}>
      <pre className="scroll-thin max-h-64 overflow-auto whitespace-pre-wrap break-words font-mono text-xs leading-relaxed text-white/50">
        {plan ? JSON.stringify(plan, null, 2) : "—"}
      </pre>
    </Card>
  );
}
