"""Mission logger — durable per-mission record on disk.

For every mission, opens a fresh log directory and streams JSONL files for:
    intent.json        original user intent + LLM-emitted command
    perception.jsonl   one line per /perception/event
    analytics.jsonl    one line per /crowd_analytics
    anomaly.jsonl      one line per /anomaly_alert
    status.jsonl       one line per /mission_status event
    report.txt         final natural-language report
    summary.json       overall mission outcome (success/failed/aborted, counts)

Designed so post-flight forensics or evaluation harnesses (precision/recall
on real Tello footage, latency analysis, anomaly false-positive rate) have
a stable artifact to load. One mission == one folder; folders are named
<UTC_timestamp>_<intent_slug> for sortability.

Subscribes:
    /user_intent       std_msgs/String
    /mission_command   std_msgs/String (JSON)
    /mission_status    std_msgs/String (JSON)
    /perception/event  std_msgs/String (JSON)
    /crowd_analytics   std_msgs/String (JSON)
    /anomaly_alert     std_msgs/String (JSON)
    /mission_report    std_msgs/String
"""

import json
import os
import re
import time
from datetime import datetime, timezone

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def _slug(text, maxlen=40):
    s = re.sub(r"[^a-zA-Z0-9]+", "_", text or "").strip("_").lower()
    return (s[:maxlen] or "mission")


class MissionLoggerNode(Node):
    def __init__(self):
        super().__init__("mission_logger_node")

        self.declare_parameter("log_root", "/tmp/autobee_missions")
        self.log_root = str(self.get_parameter("log_root").value)
        os.makedirs(self.log_root, exist_ok=True)

        self.create_subscription(String, "/user_intent", self._on_intent, 10)
        self.create_subscription(String, "/mission_command", self._on_command, 10)
        self.create_subscription(String, "/mission_status", self._on_status, 10)
        self.create_subscription(String, "/perception/event", self._on_event, 10)
        self.create_subscription(String, "/crowd_analytics", self._on_analytics, 10)
        self.create_subscription(String, "/anomaly_alert", self._on_anomaly_alert, 10)
        self.create_subscription(String, "/mission_report", self._on_report, 10)

        self._mission_dir = None
        self._fh = {}
        self._latest_intent = ""
        self._counters = {"events": 0, "analytics": 0, "anomalies": 0}
        self._outcome = None

        self.get_logger().info(f"mission_logger_node up; log_root={self.log_root}")

    # ---- Topic handlers ------------------------------------------------

    def _on_intent(self, msg: String):
        # Intent arrives BEFORE /mission_command. Stash it for use when the
        # command lands and we open the log directory.
        self._latest_intent = msg.data.strip()

    def _on_command(self, msg: String):
        self._open_mission_dir(msg.data)

    def _on_status(self, msg: String):
        self._append_jsonl("status.jsonl", msg.data)
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        event = payload.get("event")
        if event in ("completed", "aborted", "failed"):
            self._outcome = event
            self._finalize(payload)

    def _on_event(self, msg: String):
        if self._mission_dir is None:
            return
        self._counters["events"] += 1
        self._append_jsonl("perception.jsonl", msg.data)

    def _on_analytics(self, msg: String):
        if self._mission_dir is None:
            return
        self._counters["analytics"] += 1
        self._append_jsonl("analytics.jsonl", msg.data)

    def _on_anomaly_alert(self, msg: String):
        if self._mission_dir is None:
            return
        self._counters["anomalies"] += 1
        self._append_jsonl("anomaly.jsonl", msg.data)

    def _on_report(self, msg: String):
        if self._mission_dir is None:
            return
        self._write_text("report.txt", msg.data)

    # ---- Internals -----------------------------------------------------

    def _open_mission_dir(self, command_raw):
        # Close any prior mission's handles
        self._close_handles()
        self._counters = {"events": 0, "analytics": 0, "anomalies": 0}
        self._outcome = None

        try:
            command = json.loads(command_raw)
        except json.JSONDecodeError:
            command = {"raw": command_raw}

        intent_slug = _slug(command.get("intent") or self._latest_intent or "mission")
        ts = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        dirname = f"{ts}_{intent_slug}"
        self._mission_dir = os.path.join(self.log_root, dirname)
        os.makedirs(self._mission_dir, exist_ok=True)

        bundle = {
            "user_intent": self._latest_intent,
            "mission_command": command,
            "started_at_unix": time.time(),
            "started_at_iso": datetime.now(timezone.utc).isoformat(),
        }
        self._write_text("intent.json", json.dumps(bundle, indent=2))

        self.get_logger().info(f"logging mission to {self._mission_dir}")

    def _finalize(self, status_payload):
        if self._mission_dir is None:
            return
        summary = {
            "outcome": self._outcome,
            "phase": status_payload.get("phase"),
            "reason": status_payload.get("reason", ""),
            "intent": status_payload.get("intent", ""),
            "duration_s": status_payload.get("elapsed_s"),
            "perception_events": self._counters["events"],
            "analytics_samples": self._counters["analytics"],
            "anomaly_alerts": self._counters["anomalies"],
            "ended_at_unix": time.time(),
            "ended_at_iso": datetime.now(timezone.utc).isoformat(),
            "mission_dir": self._mission_dir,
        }
        self._write_text("summary.json", json.dumps(summary, indent=2))
        self.get_logger().info(
            f"mission {self._outcome} in {self._mission_dir} | "
            f"{self._counters['events']} events, "
            f"{self._counters['anomalies']} anomalies"
        )
        self._close_handles()
        self._mission_dir = None

    def _append_jsonl(self, name, line):
        if self._mission_dir is None:
            return
        fh = self._fh.get(name)
        if fh is None:
            fh = open(os.path.join(self._mission_dir, name), "a", buffering=1)
            self._fh[name] = fh
        fh.write(line.rstrip("\n") + "\n")

    def _write_text(self, name, content):
        if self._mission_dir is None:
            return
        with open(os.path.join(self._mission_dir, name), "w") as f:
            f.write(content)

    def _close_handles(self):
        for fh in self._fh.values():
            try:
                fh.close()
            except Exception:
                pass
        self._fh = {}


def main(args=None):
    rclpy.init(args=args)
    node = MissionLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._close_handles()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
