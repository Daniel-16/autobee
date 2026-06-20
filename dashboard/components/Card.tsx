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
      className={`group relative overflow-hidden rounded-none border border-line bg-panel/70 p-6 backdrop-blur-sm transition-colors duration-500 hover:border-line2 ${className}`}
    >
      {/* hairline top-edge accent, brightens on hover */}
      <span className="pointer-events-none absolute inset-x-0 top-0 h-px bg-gradient-to-r from-transparent via-white/20 to-transparent opacity-60 transition-opacity duration-500 group-hover:opacity-100" />
      {title && (
        <h3 className="eyebrow mb-4 text-[10px] text-white/45">{title}</h3>
      )}
      {children}
    </section>
  );
}

export function Row({ label, value }: { label: string; value: React.ReactNode }) {
  return (
    <div className="flex items-center justify-between border-b border-line py-2 text-sm last:border-none">
      <span className="font-light text-white/45">{label}</span>
      <span className="font-medium tabular-nums text-white/90">{value}</span>
    </div>
  );
}
