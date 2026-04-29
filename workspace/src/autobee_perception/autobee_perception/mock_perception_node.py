"""Mock perception — synthesizes plausible person-detection events during a mission.

Use when humanoid meshes aren't loaded into the world (or when a deterministic
demo is preferred over real YOLOv8 inference). Topics match yolov8_perception_node
exactly so report consumers don't care which is the source.

Flow:
    1. Wait for /mission_command (LLM agent published a mission JSON)
    2. Skip the climb/settle phase (~22s) -- camera is mostly sky
    3. Emit synthetic detections at rate_hz Hz, with counts following a bell
       curve peaking in the middle of the survey (drone is "over the crowd").
    4. Stop after survey_window_s elapses; reset on next mission_command.

Published topics:
    /perception/person_count   std_msgs/Int32   (matches yolov8 schema)
    /perception/anomaly        std_msgs/String  (threshold breach -- first occurrence)
    /perception/event          std_msgs/String  (JSON: timestamp, count, density, confidence, intent)
"""

import json
import math
import random

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String


class MockPerceptionNode(Node):
    def __init__(self):
        super().__init__("mock_perception_node")

        self.declare_parameter("rate_hz", 1.0)
        self.declare_parameter("max_persons", 12)
        self.declare_parameter("anomaly_threshold", 10)
        self.declare_parameter("survey_window_s", 180.0)
        self.declare_parameter("climb_phase_s", 22.0)
        self.declare_parameter("seed", 42)

        rate = float(self.get_parameter("rate_hz").value)
        self.max_persons = int(self.get_parameter("max_persons").value)
        self.anomaly_threshold = int(self.get_parameter("anomaly_threshold").value)
        self.survey_window = float(self.get_parameter("survey_window_s").value)
        self.climb_phase = float(self.get_parameter("climb_phase_s").value)
        self._rng = random.Random(int(self.get_parameter("seed").value))

        self.count_pub = self.create_publisher(Int32, "/perception/person_count", 10)
        self.anomaly_pub = self.create_publisher(String, "/perception/anomaly", 10)
        self.event_pub = self.create_publisher(String, "/perception/event", 10)
        self.create_subscription(String, "/mission_command", self._on_command, 10)

        self._mission_start = None
        self._mission_intent = ""
        self._anomaly_active = False

        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f"mock_perception_node up; rate={rate}Hz max_persons={self.max_persons} "
            f"anomaly_threshold={self.anomaly_threshold}"
        )

    def _on_command(self, msg: String):
        try:
            cmd = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if cmd.get("action") != "survey_local_area":
            return
        self._mission_intent = str(cmd.get("intent", ""))
        self._mission_start = self.get_clock().now()
        self._anomaly_active = False
        self.get_logger().info(
            f"mission active (intent={self._mission_intent!r}); will emit detections"
        )

    def _tick(self):
        if self._mission_start is None:
            return
        elapsed = (self.get_clock().now() - self._mission_start).nanoseconds / 1e9
        if elapsed > self.survey_window:
            return
        if elapsed < self.climb_phase:
            return

        survey_span = max(self.survey_window - self.climb_phase, 1.0)
        progress = max(0.0, min(1.0, (elapsed - self.climb_phase) / survey_span))
        bell = math.exp(-((progress - 0.5) ** 2) / 0.04)
        baseline = self.max_persons * bell
        noise = self._rng.gauss(0, 1.2)
        count = max(0, int(round(baseline + noise)))

        if count <= 3:
            density = "low"
        elif count <= 8:
            density = "medium"
        else:
            density = "high"
        confidence = round(self._rng.uniform(0.68, 0.92), 2)

        count_msg = Int32()
        count_msg.data = count
        self.count_pub.publish(count_msg)

        event = {
            "timestamp": self.get_clock().now().nanoseconds / 1e9,
            "frame_id": "drone/camera",
            "person_count": count,
            "density": density,
            "confidence": confidence,
            "mission_elapsed_s": round(elapsed, 1),
            "intent": self._mission_intent,
        }
        event_msg = String()
        event_msg.data = json.dumps(event)
        self.event_pub.publish(event_msg)

        if count >= self.anomaly_threshold and not self._anomaly_active:
            self._anomaly_active = True
            alert = String()
            alert.data = (
                f"CROWD_DENSITY_HIGH person_count={count} "
                f"threshold={self.anomaly_threshold}"
            )
            self.anomaly_pub.publish(alert)
            self.get_logger().warn(alert.data)


def main(args=None):
    rclpy.init(args=args)
    node = MockPerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
