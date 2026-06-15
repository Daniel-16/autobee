import { Card, Row } from "./Card";
import { Telemetry as T } from "@/lib/types";

export function Telemetry({
  t,
  className = "",
}: {
  t: T;
  className?: string;
}) {
  const v = (n: number, unit: string) => (t.have_odom ? `${n}${unit}` : "—");
  return (
    <Card title="Telemetry" className={className}>
      <Row label="Position X" value={v(t.x, " m")} />
      <Row label="Position Y" value={v(t.y, " m")} />
      <Row label="Altitude Z" value={v(t.z, " m")} />
      <Row label="Heading" value={v(t.yaw_deg, "°")} />
      <Row label="Speed" value={v(t.speed, " m/s")} />
    </Card>
  );
}
