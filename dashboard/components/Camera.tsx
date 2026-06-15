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
    <Card title="Camera (live)" className={className}>
      {/* MJPEG stream straight from the bridge; <img> needs no CORS. */}
      {/* eslint-disable-next-line @next/next/no-img-element */}
      <img
        src={`${url}/camera.mjpg`}
        alt="annotated camera feed"
        className="block aspect-video w-full rounded-lg bg-black object-contain"
      />
    </Card>
  );
}
