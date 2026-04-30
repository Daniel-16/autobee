"""Phase-3 perception: YOLOv8n on the drone camera, publishing detections, person count, anomaly flags."""

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
from cv_bridge import CvBridge
from ultralytics import YOLO

PERSON_CLASS_COCO = 0


class YoloPerceptionNode(Node):
    def __init__(self):
        super().__init__("yolov8_perception_node")

        self.declare_parameter("model", "yolov8n.pt")
        self.declare_parameter("conf", 0.35)
        self.declare_parameter("image_topic", "/drone/camera/image_raw")
        self.declare_parameter("anomaly_threshold", 10)
        self.declare_parameter("yolo_config_dir", "/tmp/Ultralytics")

        model_name = self.get_parameter("model").value
        self.conf = float(self.get_parameter("conf").value)
        image_topic = self.get_parameter("image_topic").value
        self.threshold = int(self.get_parameter("anomaly_threshold").value)
        os.environ.setdefault("YOLO_CONFIG_DIR", self.get_parameter("yolo_config_dir").value)

        self.get_logger().info(f"Loading YOLO model: {model_name}")
        self.model = YOLO(model_name)
        self.bridge = CvBridge()

        self.sub = self.create_subscription(Image, image_topic, self._on_image, 10)
        self.det_pub = self.create_publisher(Detection2DArray, "/perception/detections", 10)
        self.ann_pub = self.create_publisher(Image, "/perception/image_annotated", 10)
        self.count_pub = self.create_publisher(Int32, "/perception/person_count", 10)
        self.anomaly_pub = self.create_publisher(String, "/perception/anomaly", 10)
        self.event_pub = self.create_publisher(String, "/perception/event", 10)

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
        results = self.model.predict(
            frame, conf=self.conf, verbose=False, classes=[PERSON_CLASS_COCO]
        )
        r = results[0]

        detections = Detection2DArray()
        detections.header = msg.header
        person_count = 0
        person_confs = []
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

        self.det_pub.publish(detections)

        annotated = r.plot()
        ann_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
        ann_msg.header = msg.header
        self.ann_pub.publish(ann_msg)

        count_msg = Int32()
        count_msg.data = person_count
        self.count_pub.publish(count_msg)

        if person_count <= 3:
            density = "low"
        elif person_count <= 8:
            density = "medium"
        else:
            density = "high"
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
