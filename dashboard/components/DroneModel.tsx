/**
 * Hero aircraft — the real DJI Tello (transparent PNG). It reveals with a
 * blur-in / scale-up / lift, an expanding sonar ring and a soft underglow,
 * then settles into a continuous hover. Pure CSS so there's no asset pipeline
 * beyond the static image.
 */
export function DroneModel({ className = "" }: { className?: string }) {
  return (
    <div className={`relative aspect-square ${className}`}>
      {/* soft underglow */}
      <div className="drone-glow pointer-events-none absolute left-1/2 top-1/2 h-[70%] w-[70%] -translate-x-1/2 -translate-y-1/2 rounded-full bg-white/10 blur-3xl" />

      {/* expanding sonar rings (one-shot on reveal) */}
      <span className="drone-ring pointer-events-none absolute left-1/2 top-1/2 h-2/3 w-2/3 -translate-x-1/2 -translate-y-1/2 rounded-full border border-white/20" />
      <span className="drone-ring drone-ring-2 pointer-events-none absolute left-1/2 top-1/2 h-2/3 w-2/3 -translate-x-1/2 -translate-y-1/2 rounded-full border border-white/15" />

      {/* the aircraft */}
      <div className="drone-hover absolute inset-0">
        {/* eslint-disable-next-line @next/next/no-img-element */}
        <img
          src="/tello.png"
          alt="DJI Tello quadcopter"
          className="h-full w-full object-contain drop-shadow-[0_30px_60px_rgba(0,0,0,0.7)]"
          draggable={false}
        />
      </div>
    </div>
  );
}
