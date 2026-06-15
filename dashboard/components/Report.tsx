import { Card } from "./Card";

export function Report({
  text,
  className = "",
}: {
  text: string;
  className?: string;
}) {
  return (
    <Card title="Mission report" className={className}>
      {text ? (
        <p className="whitespace-pre-wrap text-[15px] leading-relaxed">{text}</p>
      ) : (
        <span className="text-sm italic text-slate-500">
          The report appears here when the mission finishes.
        </span>
      )}
    </Card>
  );
}
