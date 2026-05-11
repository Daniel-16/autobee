"""Phase-3 perception: YOLOv8n on the drone camera.

Publishes:
  /perception/detections        vision_msgs/Detection2DArray (all classes, raw)
  /perception/image_annotated   sensor_msgs/Image (debug visual)
  /perception/person_count      std_msgs/Int32 (count of class 0)
  /perception/event             std_msgs/String JSON (per-frame person summary)
  /perception/anomaly           std_msgs/String (legacy single-frame breach)
  /perception/obstacles         std_msgs/String JSON — the most threatening
      non-person detection per frame (large + centred), used by mission_executor
      as a collision-imminent backstop. Only published when a detection clears
      the size + centredness thresholds; silent otherwise.

YOLO is run unfiltered (the previous `classes=[0]` filter is removed) so
non-person classes can feed obstacle detection. Person semantics on the
existing channels are preserved by filtering at publish time.
"""

import json
import os

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, String
from vision_msgs.msg import (
    Detection2D,
    Detection2DArray,
    ObjectHypothesisWithPose,
)
import cv2
from cv_bridge import CvBridge
from ultralytics import YOLO

PERSON_CLASS_COCO = 0


def _draw_crowd_hud(frame, person_count: int, density: str):
    """Overlay a crowd-analytics headline strip on the YOLO-annotated frame.

    Frames published to /perception/image_annotated are the operator's primary
    visual signal during a mission, and "PEOPLE: N" plus density is the
    headline for crowd-analytics demos.
    """
    colour = {"low": (0, 200, 0), "medium": (0, 200, 230), "high": (40, 60, 230)}.get(
        density, (200, 200, 200)
    )
    h, w = frame.shape[:2]
    bar_h = max(28, int(h * 0.07))
    cv2.rectangle(frame, (0, 0), (w, bar_h), (20, 20, 20), thickness=-1)
    cv2.putText(
        frame, f"PEOPLE: {person_count}    density: {density.upper()}",
        (10, int(bar_h * 0.72)),
        cv2.FONT_HERSHEY_SIMPLEX, 0.7, colour, 2, cv2.LINE_AA,
    )
    return frame


class YoloPerceptionNode(Node):
    def __init__(self):
        super().__init__("yolov8_perception_node")

        self.declare_parameter("model", "yolov8n.pt")
        self.declare_parameter("conf", 0.35)
        self.declare_parameter("image_topic", "/drone/camera/image_raw")
        self.declare_parameter("anomaly_threshold", 10)
        self.declare_parameter("yolo_config_dir", "/tmp/Ultralytics")
        # Obstacle-channel thresholds: a non-person detection is published as
        # a collision-imminent obstacle only if it meets ALL three.
        self.declare_parameter("obstacle_min_size_frac", 0.06)   # >6% of frame area
        self.declare_parameter("obstacle_max_centre_frac", 0.35) # |dx_centre|/W <= 0.35
        self.declare_parameter("obstacle_min_score", 0.40)

        model_name = self.get_parameter("model").value
        self.conf = float(self.get_parameter("conf").value)
        image_topic = self.get_parameter("image_topic").value
        self.threshold = int(self.get_parameter("anomaly_threshold").value)
        self.obs_min_size = float(self.get_parameter("obstacle_min_size_frac").value)
        self.obs_max_centre = float(self.get_parameter("obstacle_max_centre_frac").value)
        self.obs_min_score = float(self.get_parameter("obstacle_min_score").value)
        os.environ.setdefault("YOLO_CONFIG_DIR", self.get_parameter("yolo_config_dir").value)

        self.get_logger().info(f"Loading YOLO model: {model_name}")
        self.model = YOLO(model_name)
        self.bridge = CvBridge()
        # Cache class id -> name mapping for human-readable obstacle alerts.
        self._class_names = (
            getattr(self.model, "names", None)
            or getattr(self.model.model, "names", {})
            or {}
        )

        self.sub = self.create_subscription(Image, image_topic, self._on_image, 10)
        self.det_pub = self.create_publisher(Detection2DArray, "/perception/detections", 10)
        self.ann_pub = self.create_publisher(Image, "/perception/image_annotated", 10)
        self.count_pub = self.create_publisher(Int32, "/perception/person_count", 10)
        self.anomaly_pub = self.create_publisher(String, "/perception/anomaly", 10)
        self.event_pub = self.create_publisher(String, "/perception/event", 10)
        self.obstacle_pub = self.create_publisher(String, "/perception/obstacles", 10)

        self._frames = 0
        self._anomaly_active = False
        self._log_timer = self.create_timer(5.0, self._log_rate)
        self.get_logger().info(f"Subscribed to {image_topic}")

    def _log_rate(self):
        rate = self._frames / 5.0
        self.get_logger().info(f"perception FPS: {rate:.1f}")
        self._frames = 0

    def _on_image(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        # YOLO is run UNFILTERED so non-person classes feed the obstacle
        # backstop channel. Person semantics below are preserved by class id.
        results = self.model.predict(frame, conf=self.conf, verbose=False)
        r = results[0]

        height, width = frame.shape[:2]
        frame_area = float(max(1, height * width))

        detections = Detection2DArray()
        detections.header = msg.header
        person_count = 0
        person_confs = []
        # Track the most threatening non-person detection for the obstacle
        # channel: threat = score * size_frac * centredness, cleared by
        # min thresholds.
        best_obstacle = None
        best_threat = 0.0
        for box in r.boxes:
            cls_id = int(box.cls[0])
            conf = float(box.conf[0])
            x1, y1, x2, y2 = box.xyxy[0].tolist()

            d = Detection2D()
            d.header = msg.header
            d.bbox.center.position.x = (x1 + x2) / 2.0
            d.bbox.center.position.y = (y1 + y2) / 2.0
            d.bbox.size_x = x2 - x1
            d.bbox.size_y = y2 - y1

            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = str(cls_id)
            hyp.hypothesis.score = conf
            d.results.append(hyp)

            detections.detections.append(d)
            if cls_id == PERSON_CLASS_COCO:
                person_count += 1
                person_confs.append(conf)
                continue

            # Non-person: evaluate as obstacle threat.
            cx = (x1 + x2) / 2.0
            size_frac = ((x2 - x1) * (y2 - y1)) / frame_area
            centre_offset_frac = abs(cx - width / 2.0) / max(1.0, width / 2.0)
            if (
                conf >= self.obs_min_score
                and size_frac >= self.obs_min_size
                and centre_offset_frac <= self.obs_max_centre
            ):
                centredness = 1.0 - centre_offset_frac
                threat = conf * size_frac * centredness
                if threat > best_threat:
                    best_threat = threat
                    best_obstacle = {
                        "class_id": cls_id,
                        "class": self._class_names.get(cls_id, f"class_{cls_id}"),
                        "score": round(conf, 3),
                        "bbox": [round(x1, 1), round(y1, 1), round(x2, 1), round(y2, 1)],
                        "size_frac": round(size_frac, 4),
                        "centre_offset_frac": round(centre_offset_frac, 3),
                        "threat": round(threat, 4),
                        "frame_id": msg.header.frame_id or "drone/camera",
                        "timestamp": self.get_clock().now().nanoseconds / 1e9,
                    }

        if best_obstacle is not None:
            obs_msg = String()
            obs_msg.data = json.dumps(best_obstacle)
            self.obstacle_pub.publish(obs_msg)

        self.det_pub.publish(detections)

        if person_count <= 3:
            density = "low"
        elif person_count <= 8:
            density = "medium"
        else:
            density = "high"

        annotated = r.plot()
        annotated = _draw_crowd_hud(annotated, person_count, density)
        ann_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
        ann_msg.header = msg.header
        self.ann_pub.publish(ann_msg)

        count_msg = Int32()
        count_msg.data = person_count
        self.count_pub.publish(count_msg)
        avg_conf = round(sum(person_confs) / len(person_confs), 2) if person_confs else 0.0

        event = {
            "timestamp": self.get_clock().now().nanoseconds / 1e9,
            "frame_id": msg.header.frame_id or "drone/camera",
            "person_count": person_count,
            "density": density,
            "confidence": avg_conf,
            "source": "yolov8",
        }
        event_msg = String()
        event_msg.data = json.dumps(event)
        self.event_pub.publish(event_msg)

        if person_count >= self.threshold and not self._anomaly_active:
            self._anomaly_active = True
            alert = String()
            alert.data = (
                f"CROWD_DENSITY_HIGH person_count={person_count} threshold={self.threshold}"
            )
            self.anomaly_pub.publish(alert)
        elif person_count < int(self.threshold * 0.7):
            self._anomaly_active = False

        self._frames += 1


def main(args=None):
    rclpy.init(args=args)
    node = YoloPerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
