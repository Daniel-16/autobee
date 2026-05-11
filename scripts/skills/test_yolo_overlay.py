#!/usr/bin/env python3
"""M3 acceptance gate: /perception/image_annotated publishes with crowd HUD.

Prereq: `ros2 launch autobee_core demo.launch.py` running.

This is a topic-rate + frame-shape check; the HUD strip itself is verified
by eye via `rqt_image_view /perception/image_annotated` (a dark bar across
the top with "PEOPLE: N    density: LOW/MED/HIGH").
"""

import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


WINDOW_S = 6.0
MIN_FPS = 3.0  # NFR-02 floor with sampling jitter
TOPIC = "/perception/image_annotated"


class OverlayWatcher(Node):
    def __init__(self):
        super().__init__("overlay_watcher")
        self.bridge = CvBridge()
        self.frames = 0
        self.size = None
        self.hud_pixel_seen = False
        self.create_subscription(Image, TOPIC, self._on_frame, 10)

    def _on_frame(self, msg: Image):
        self.frames += 1
        self.size = (msg.width, msg.height)
        # Sample the top-centre pixel; the HUD fills the top strip with
        # (20,20,20). If that pixel is darker than the camera background
        # threshold we can be confident the HUD is rendering.
        try:
            cv = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            b, g, r = cv[2, msg.width // 2]
            if int(b) < 50 and int(g) < 50 and int(r) < 50:
                self.hud_pixel_seen = True
        except Exception:
            pass


def main():
    rclpy.init()
    node = OverlayWatcher()
    print(f"sampling {TOPIC} for {WINDOW_S}s...")
    deadline = time.time() + WINDOW_S
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    fps = node.frames / WINDOW_S
    print(f"  frames: {node.frames}  fps: {fps:.2f}  size: {node.size}")
    print(f"  HUD top-strip dark pixel observed: {node.hud_pixel_seen}")

    failed = []
    if fps < MIN_FPS:
        failed.append(f"fps {fps:.2f} < {MIN_FPS}")
    if node.size is None:
        failed.append("no frames received")
    if not node.hud_pixel_seen:
        failed.append("HUD strip not detected (top-centre pixel was not dark)")

    node.destroy_node()
    rclpy.shutdown()
    if failed:
        print("FAIL: " + "; ".join(failed))
        sys.exit(1)
    print("PASS")
    sys.exit(0)


if __name__ == "__main__":
    main()
