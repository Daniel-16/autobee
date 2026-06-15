// Shape of the snapshot served by the ROS bridge (autobee_ui/web_ui_node.py).

export interface Mission {
  event: string;          // idle | started | phase | completed | aborted | failed | ...
  phase: string;          // CLIMB | TRANSIT_OUT | SURVEY | DONE | ...
  intent: string;
  landmark: string | null;
  reason: string;
  wall_time: number;
}

export interface Telemetry {
  x: number;
  y: number;
  z: number;
  yaw_deg: number;
  speed: number;
  have_odom: boolean;
}

export interface Analytics {
  current_count?: number;
  peak_count?: number;
  mean_count?: number;
  median_count?: number;
  density_label?: "low" | "medium" | "high" | string;
  trend?: "rising" | "falling" | "stable" | string;
  rate_per_sec?: number;
}

export interface Anomaly {
  type?: string;
  severity?: "low" | "medium" | "high" | string;
  description?: string;
  timestamp?: number;
}

export interface EventLine {
  t: number;
  text: string;
  reason: string;
}

export interface BridgeState {
  camera_available: boolean;
  mission: Mission;
  plan: unknown;
  report: string;
  analytics: Analytics | null;
  anomalies: Anomaly[];
  events: EventLine[];
  telemetry: Telemetry;
  person_count: number;
  last_intent: string;
}

export const EMPTY_STATE: BridgeState = {
  camera_available: false,
  mission: {
    event: "idle", phase: "—", intent: "", landmark: null,
    reason: "", wall_time: 0,
  },
  plan: null,
  report: "",
  analytics: null,
  anomalies: [],
  events: [],
  telemetry: { x: 0, y: 0, z: 0, yaw_deg: 0, speed: 0, have_odom: false },
  person_count: 0,
  last_intent: "",
};
