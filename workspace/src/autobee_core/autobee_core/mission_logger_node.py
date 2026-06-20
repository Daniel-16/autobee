"""Mission logger — durable per-mission record on disk.

For every mission, opens a fresh log directory and streams JSONL files for:
    intent.json        original user intent + LLM-emitted command
    perception.jsonl   one line per /perception/event
    analytics.jsonl    one line per /crowd_analytics
    anomaly.jsonl      one line per /anomaly_alert
    status.jsonl       one line per /mission_status event
    report.txt         final natural-language report
    summary.json       overall mission outcome (success/failed/aborted, counts)
    frames/            periodic PNGs of the annotated camera feed + one per
                       anomaly (when save_frames=true)

On the terminal mission event the whole folder is zipped to
<UTC_timestamp>_<intent_slug>.zip (when zip_on_finish=true) so each mission
leaves a single shareable file with pictures + logs.

Designed so post-flight forensics or evaluation harnesses (precision/recall
on real Tello footage, latency analysis, anomaly false-positive rate) have
a stable artifact to load. One mission == one folder; folders are named
<UTC_timestamp>_<intent_slug> for sortability.

Frames are decoded directly from sensor_msgs/Image (numpy + cv2, no cv_bridge)
so the logger is immune to the cv_bridge/numpy ABI coupling.

Subscribes:
    /user_intent             std_msgs/String
    /mission_command         std_msgs/String (JSON)
    /mission_status          std_msgs/String (JSON)
    /perception/event        std_msgs/String (JSON)
    /crowd_analytics         std_msgs/String (JSON)
    /anomaly_alert           std_msgs/String (JSON)
    /mission_report          std_msgs/String
    /perception/image_annotated  sensor_msgs/Image (frame_topic, when save_frames)
"""

import json
import os
import re
import time
import zipfile
from datetime import datetime, timezone

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image

try:
    import numpy as _np
    import cv2 as _cv2
    _HAS_CV = True
except Exception:  # numpy/cv2 missing or broken -> logging still works
    _HAS_CV = False


def _slug(text, maxlen=40):
    s = re.sub(r"[^a-zA-Z0-9]+", "_", text or "").strip("_").lower()
    return (s[:maxlen] or "mission")


class MissionLoggerNode(Node):
    def __init__(self):
        super().__init__("mission_logger_node")

        self.declare_parameter("log_root", "/tmp/autobee_missions")
        self.declare_parameter("save_frames", True)
        self.declare_parameter("frame_topic", "/perception/image_annotated")
        self.declare_parameter("frame_interval_s", 1.0)
        self.declare_parameter("max_frames", 120)
        self.declare_parameter("zip_on_finish", True)

        self.log_root = str(self.get_parameter("log_root").value)
        os.makedirs(self.log_root, exist_ok=True)
        self.frame_interval_s = float(self.get_parameter("frame_interval_s").value)
        self.max_frames = int(self.get_parameter("max_frames").value)
        self.zip_on_finish = bool(self.get_parameter("zip_on_finish").value)
        want_frames = bool(self.get_parameter("save_frames").value)
        self.save_frames = want_frames and _HAS_CV

        self.create_subscription(String, "/user_intent", self._on_intent, 10)
        self.create_subscription(String, "/mission_command", self._on_command, 10)
        self.create_subscription(String, "/mission_status", self._on_status, 10)
        self.create_subscription(String, "/perception/event", self._on_event, 10)
        self.create_subscription(String, "/crowd_analytics", self._on_analytics, 10)
        self.create_subscription(String, "/anomaly_alert", self._on_anomaly_alert, 10)
        self.create_subscription(String, "/mission_report", self._on_report, 10)
        if self.save_frames:
            self.create_subscription(
                Image, str(self.get_parameter("frame_topic").value), self._on_frame, 10)
        elif want_frames:
            self.get_logger().warn(
                "save_frames requested but numpy/cv2 unavailable; saving logs only")

        self._mission_dir = None
        self._mission_start = None
        self._frames_dir = None
        self._frame_count = 0
        self._last_frame_save = 0.0
        self._latest_frame = None
        self._fh = {}
        self._latest_intent = ""
        self._counters = {"events": 0, "analytics": 0, "anomalies": 0}
        self._outcome = None

        self.get_logger().info(
            f"mission_logger_node up; log_root={self.log_root} "
            f"save_frames={self.save_frames} zip_on_finish={self.zip_on_finish}")

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
        # Grab the frame at the moment of the alert — the "result" picture.
        if self.save_frames and self._frames_dir is not None and self._latest_frame is not None:
            path = os.path.join(self._frames_dir, f"anomaly_{self._counters['anomalies']:03d}.png")
            try:
                _cv2.imwrite(path, self._latest_frame)
            except Exception as e:
                self.get_logger().warn(f"anomaly frame write failed: {e}")

    def _on_report(self, msg: String):
        if self._mission_dir is None:
            return
        self._write_text("report.txt", msg.data)

    def _on_frame(self, msg: Image):
        img = self._decode_image(msg)
        if img is None:
            return
        self._latest_frame = img
        if self._mission_dir is None or self._frames_dir is None:
            return
        now = time.time()
        if now - self._last_frame_save < self.frame_interval_s:
            return
        if self._frame_count >= self.max_frames:
            return
        self._last_frame_save = now
        path = os.path.join(self._frames_dir, f"frame_{self._frame_count:04d}.png")
        try:
            _cv2.imwrite(path, img)
            self._frame_count += 1
        except Exception as e:
            self.get_logger().warn(f"frame write failed: {e}")

    @staticmethod
    def _decode_image(msg: Image):
        """sensor_msgs/Image -> BGR ndarray, without cv_bridge."""
        try:
            buf = _np.frombuffer(msg.data, dtype=_np.uint8)
            if msg.encoding in ("bgr8", "rgb8"):
                img = buf.reshape(msg.height, msg.step)[:, : msg.width * 3].reshape(
                    msg.height, msg.width, 3)
                if msg.encoding == "rgb8":
                    img = img[:, :, ::-1]
                return _np.ascontiguousarray(img)
            if msg.encoding == "mono8":
                return _np.ascontiguousarray(
                    buf.reshape(msg.height, msg.step)[:, : msg.width])
        except Exception:
            return None
        return None

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
        self._mission_start = time.time()
        os.makedirs(self._mission_dir, exist_ok=True)
        if self.save_frames:
            self._frames_dir = os.path.join(self._mission_dir, "frames")
            os.makedirs(self._frames_dir, exist_ok=True)
            self._frame_count = 0
            self._last_frame_save = 0.0

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
            "duration_s": (
                round(time.time() - self._mission_start, 1)
                if self._mission_start else None
            ),
            "perception_events": self._counters["events"],
            "analytics_samples": self._counters["analytics"],
            "anomaly_alerts": self._counters["anomalies"],
            "frames_saved": self._frame_count,
            "ended_at_unix": time.time(),
            "ended_at_iso": datetime.now(timezone.utc).isoformat(),
            "mission_dir": self._mission_dir,
        }
        self._write_text("summary.json", json.dumps(summary, indent=2))
        self._close_handles()  # flush jsonl before zipping
        bundle_path = self._zip_mission_dir() if self.zip_on_finish else None
        self.get_logger().info(
            f"mission {self._outcome} in {self._mission_dir} | "
            f"{self._counters['events']} events, "
            f"{self._counters['anomalies']} anomalies, "
            f"{self._frame_count} frames"
            + (f" | bundle: {bundle_path}" if bundle_path else "")
        )
        self._mission_dir = None
        self._frames_dir = None

    def _zip_mission_dir(self):
        if self._mission_dir is None:
            return None
        zip_path = self._mission_dir.rstrip("/") + ".zip"
        parent = os.path.dirname(self._mission_dir)
        try:
            with zipfile.ZipFile(zip_path, "w", zipfile.ZIP_DEFLATED) as zf:
                for root, _dirs, files in os.walk(self._mission_dir):
                    for fn in files:
                        full = os.path.join(root, fn)
                        zf.write(full, os.path.relpath(full, parent))
            return zip_path
        except Exception as e:
            self.get_logger().warn(f"bundle zip failed: {e}")
            return None

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
