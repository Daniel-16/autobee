"""Anomaly detection — surveillance-relevant patterns on /crowd_analytics.

Implements the four anomaly classes called out in the AutoBee thesis:
    SURGE              -- count rises rapidly (e.g. crowd forming)
    DISPERSAL          -- count drops rapidly (e.g. crowd scattering)
    SUSTAINED_HIGH     -- high density persists past a threshold duration
    GATHERING          -- rising trend with mean above a threshold

Designed to run downstream of crowd_analytics_node. Stateful: tracks per-type
"active" status with hysteresis so a single sample doesn't flap alerts.

Subscribes:
    /crowd_analytics    std_msgs/String (JSON)

Publishes:
    /anomaly_alert      std_msgs/String (JSON)

Alert schema:
    {
      "timestamp": <unix>,
      "type": "SURGE" | "DISPERSAL" | "SUSTAINED_HIGH" | "GATHERING",
      "severity": "low" | "medium" | "high",
      "description": "<human-readable single line>",
      "metrics": {<the analytics snapshot that triggered>}
    }
"""

import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class AnomalyDetectionNode(Node):
    def __init__(self):
        super().__init__("anomaly_detection_node")

        self.declare_parameter("surge_rate_per_sec", 0.4)
        self.declare_parameter("dispersal_rate_per_sec", -0.4)
        self.declare_parameter("sustained_high_seconds", 8.0)
        self.declare_parameter("gathering_mean_threshold", 5.0)
        self.declare_parameter("rearm_silence_s", 10.0)

        self.surge_rate = float(self.get_parameter("surge_rate_per_sec").value)
        self.dispersal_rate = float(self.get_parameter("dispersal_rate_per_sec").value)
        self.sustained_high_s = float(self.get_parameter("sustained_high_seconds").value)
        self.gathering_mean = float(self.get_parameter("gathering_mean_threshold").value)
        self.rearm_silence_s = float(self.get_parameter("rearm_silence_s").value)

        self.alert_pub = self.create_publisher(String, "/anomaly_alert", 10)
        self.create_subscription(String, "/crowd_analytics", self._on_analytics, 10)

        # Per-type "last fired" timestamps for rearm-after-silence dedupe
        self._last_alert = {
            "SURGE": 0.0,
            "DISPERSAL": 0.0,
            "SUSTAINED_HIGH": 0.0,
            "GATHERING": 0.0,
        }
        # Tracking continuous-high-density window
        self._high_since = None

        self.get_logger().info(
            f"anomaly_detection_node up; surge>={self.surge_rate} p/s, "
            f"dispersal<={self.dispersal_rate} p/s, "
            f"sustained_high>={self.sustained_high_s}s, gathering_mean>={self.gathering_mean}"
        )

    def _on_analytics(self, msg: String):
        try:
            a = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        rate = float(a.get("rate_per_sec", 0.0))
        density = str(a.get("density_label", "low"))
        mean_count = float(a.get("mean_count", 0.0))
        trend = str(a.get("trend", "stable"))
        samples = int(a.get("samples", 0))
        now = time.time()

        # Update sustained-high tracking
        if density == "high":
            if self._high_since is None:
                self._high_since = now
        else:
            self._high_since = None

        # Need a minimum sample count before any inference is meaningful
        if samples < 4:
            return

        if rate >= self.surge_rate:
            self._fire(
                "SURGE",
                self._severity_from_rate(rate),
                f"Crowd is surging: +{rate:.2f} persons/sec, current density {density}.",
                a, now,
            )

        if rate <= self.dispersal_rate:
            self._fire(
                "DISPERSAL",
                self._severity_from_rate(-rate),
                f"Crowd is dispersing: {rate:.2f} persons/sec.",
                a, now,
            )

        if self._high_since is not None:
            held = now - self._high_since
            if held >= self.sustained_high_s:
                self._fire(
                    "SUSTAINED_HIGH",
                    "high",
                    f"High density sustained for {held:.0f}s "
                    f"(mean count {mean_count:.1f}).",
                    a, now,
                )

        if trend == "rising" and mean_count >= self.gathering_mean and rate > 0:
            self._fire(
                "GATHERING",
                "medium",
                f"Crowd gathering detected: trend rising, mean {mean_count:.1f} persons.",
                a, now,
            )

    def _fire(self, anomaly_type, severity, description, snapshot, now):
        if now - self._last_alert.get(anomaly_type, 0.0) < self.rearm_silence_s:
            return
        self._last_alert[anomaly_type] = now

        payload = {
            "timestamp": now,
            "type": anomaly_type,
            "severity": severity,
            "description": description,
            "metrics": snapshot,
        }
        out = String()
        out.data = json.dumps(payload)
        self.alert_pub.publish(out)
        self.get_logger().warn(f"[{severity.upper()}] {anomaly_type}: {description}")

    @staticmethod
    def _severity_from_rate(rate):
        absrate = abs(rate)
        if absrate >= 1.0:
            return "high"
        if absrate >= 0.6:
            return "medium"
        return "low"


def main(args=None):
    rclpy.init(args=args)
    node = AnomalyDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
