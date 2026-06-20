"""Crowd analytics — temporal aggregation of /perception/event.

Maintains a sliding time window over the per-frame detection stream and
publishes derived crowd statistics at a fixed rate. Decoupled from the
detection source (yolov8 today, Tello camera tomorrow) -- consumes the
abstract /perception/event JSON contract.

Subscribes:
    /perception/event   std_msgs/String (JSON) -- per-frame detection summary

Publishes:
    /crowd_analytics    std_msgs/String (JSON) -- aggregated stats at rate_hz Hz

Output schema:
    {
      "timestamp": <unix>,
      "window_s": <float>,
      "current_count": <int>,           latest person count
      "peak_count": <int>,              max in window
      "mean_count": <float>,
      "median_count": <float>,
      "stddev_count": <float>,
      "samples": <int>,                 events seen in window
      "trend": "rising" | "falling" | "stable",
      "rate_per_sec": <float>,          linear-regression slope, persons/s
      "density_label": "low" | "medium" | "high",
      "density_thresholds": {{"low_max": ..., "medium_max": ...}}
    }
"""

import collections
import json
import statistics
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class CrowdAnalyticsNode(Node):
    def __init__(self):
        super().__init__("crowd_analytics_node")

        self.declare_parameter("window_s", 30.0)
        self.declare_parameter("rate_hz", 1.0)
        self.declare_parameter("low_max", 3)
        self.declare_parameter("medium_max", 8)
        self.declare_parameter("trend_eps", 0.05)

        self.window_s = float(self.get_parameter("window_s").value)
        rate = float(self.get_parameter("rate_hz").value)
        self.low_max = int(self.get_parameter("low_max").value)
        self.medium_max = int(self.get_parameter("medium_max").value)
        self.trend_eps = float(self.get_parameter("trend_eps").value)

        # Each sample: (wall_time, count)
        self._samples = collections.deque()

        self.analytics_pub = self.create_publisher(String, "/crowd_analytics", 10)
        self.create_subscription(String, "/perception/event", self._on_event, 10)
        self.create_timer(1.0 / rate, self._tick)

        self.get_logger().info(
            f"crowd_analytics_node up; window={self.window_s}s rate={rate}Hz "
            f"thresholds=(low<={self.low_max}, medium<={self.medium_max})"
        )

    def _on_event(self, msg: String):
        try:
            event = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        try:
            count = int(event.get("person_count", 0))
        except (TypeError, ValueError):
            return
        self._samples.append((time.time(), count))

    def _prune(self, now):
        cutoff = now - self.window_s
        while self._samples and self._samples[0][0] < cutoff:
            self._samples.popleft()

    def _tick(self):
        now = time.time()
        self._prune(now)

        if not self._samples:
            payload = {
                "timestamp": now,
                "window_s": self.window_s,
                "current_count": 0,
                "peak_count": 0,
                "mean_count": 0.0,
                "median_count": 0.0,
                "stddev_count": 0.0,
                "samples": 0,
                "trend": "stable",
                "rate_per_sec": 0.0,
                "density_label": "low",
                "density_thresholds": {"low_max": self.low_max, "medium_max": self.medium_max},
            }
        else:
            counts = [c for _, c in self._samples]
            current = counts[-1]
            peak = max(counts)
            mean = statistics.fmean(counts)
            median = statistics.median(counts)
            stddev = statistics.pstdev(counts) if len(counts) > 1 else 0.0
            slope = self._linreg_slope(self._samples)
            if slope > self.trend_eps:
                trend = "rising"
            elif slope < -self.trend_eps:
                trend = "falling"
            else:
                trend = "stable"

            if current <= self.low_max:
                density = "low"
            elif current <= self.medium_max:
                density = "medium"
            else:
                density = "high"

            payload = {
                "timestamp": now,
                "window_s": self.window_s,
                "current_count": current,
                "peak_count": peak,
                "mean_count": round(mean, 2),
                "median_count": round(median, 2),
                "stddev_count": round(stddev, 2),
                "samples": len(counts),
                "trend": trend,
                "rate_per_sec": round(slope, 3),
                "density_label": density,
                "density_thresholds": {"low_max": self.low_max, "medium_max": self.medium_max},
            }

        out = String()
        out.data = json.dumps(payload)
        self.analytics_pub.publish(out)

    @staticmethod
    def _linreg_slope(samples):
        """OLS slope of count(t) over the window. Returns persons/s."""
        n = len(samples)
        if n < 2:
            return 0.0
        t0 = samples[0][0]
        xs = [t - t0 for t, _ in samples]
        ys = [c for _, c in samples]
        x_mean = sum(xs) / n
        y_mean = sum(ys) / n
        num = sum((x - x_mean) * (y - y_mean) for x, y in zip(xs, ys))
        den = sum((x - x_mean) ** 2 for x in xs)
        if den < 1e-9:
            return 0.0
        return num / den


def main(args=None):
    rclpy.init(args=args)
    node = CrowdAnalyticsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
