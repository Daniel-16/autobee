export function Card({
  title,
  className = "",
  children,
}: {
  title?: string;
  className?: string;
  children: React.ReactNode;
}) {
  return (
    <section
      className={`rounded-2xl border border-line bg-panel p-5 ${className}`}
    >
      {title && (
        <h3 className="mb-3 text-xs font-semibold uppercase tracking-wide text-slate-400">
          {title}
        </h3>
      )}
      {children}
    </section>
  );
}

export function Row({ label, value }: { label: string; value: React.ReactNode }) {
  return (
    <div className="flex items-center justify-between border-b border-line py-1.5 text-sm last:border-none">
      <span className="text-slate-400">{label}</span>
      <span className="font-semibold">{value}</span>
    </div>
  );
}
