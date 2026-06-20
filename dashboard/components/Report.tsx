import { Card } from "./Card";

export function Report({
  text,
  className = "",
}: {
  text: string;
  className?: string;
}) {
  return (
    <Card title="Mission Report" className={className}>
      {text ? (
        <p className="max-w-3xl whitespace-pre-wrap text-[15px] font-light leading-relaxed text-white/80">
          {text}
        </p>
      ) : (
        <span className="text-sm font-light italic text-white/30">
          The report appears here when the mission finishes.
        </span>
      )}
    </Card>
  );
}
