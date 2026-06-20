"use client";

import { Card } from "./Card";

export function Camera({
  url,
  className = "",
}: {
  url: string;
  className?: string;
}) {
  return (
    <Card title="Live Camera · Annotated" className={className}>
      <div className="relative overflow-hidden border border-line">
        {/* corner framing reticle — sci-fi viewport feel */}
        <span className="pointer-events-none absolute left-2 top-2 z-10 h-4 w-4 border-l border-t border-white/40" />
        <span className="pointer-events-none absolute right-2 top-2 z-10 h-4 w-4 border-r border-t border-white/40" />
        <span className="pointer-events-none absolute bottom-2 left-2 z-10 h-4 w-4 border-b border-l border-white/40" />
        <span className="pointer-events-none absolute bottom-2 right-2 z-10 h-4 w-4 border-b border-r border-white/40" />
        <span className="eyebrow pointer-events-none absolute right-3 top-3 z-10 flex items-center gap-1.5 text-[9px] text-danger">
          <span className="inline-block h-1.5 w-1.5 rounded-full bg-danger" /> REC
        </span>
        {/* MJPEG stream straight from the bridge; <img> needs no CORS. */}
        {/* eslint-disable-next-line @next/next/no-img-element */}
        <img
          src={`${url}/camera.mjpg`}
          alt="annotated camera feed"
          className="block aspect-video w-full bg-black object-contain"
        />
      </div>
    </Card>
  );
}
