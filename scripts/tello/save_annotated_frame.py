#!/usr/bin/env python3
"""View / save frames from /perception/image_annotated (YOLO boxes + crowd HUD).

For capturing a thesis figure of the Tello's onboard camera with detections
overlaid. Run it alongside demo_tello.launch.py (or the sim demo) while a
person is in frame.

    # live window: watch the feed, press 's' to grab a frame, 'q' to quit
    python3 scripts/tello/save_annotated_frame.py --show

    # headless: auto-save full-res PNGs on a timer (no window needed)
    python3 scripts/tello/save_annotated_frame.py --count 40 --interval 0.4 \
        --outdir docs/figures/tello_frames

The window needs X on the host:  xhost +local:docker  (run once per boot).
Saving to disk gives a full-resolution, chrome-free image for the thesis.
"""

import argparse
import os
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


WINDOW = "AutoBee onboard feed + YOLO"


class FrameSaver(Node):
    def __init__(self, topic, outdir, count, interval, show):
        super().__init__("annotated_frame_saver")
        self.bridge = CvBridge()
        self.outdir = outdir
        self.count = count
        self.interval = interval
        self.show = show
        self.saved = 0
        self._last = 0.0
        os.makedirs(outdir, exist_ok=True)
        self.create_subscription(Image, topic, self._on_image, 10)
        if show:
            cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
            self.get_logger().info(
                f"live view of {topic}. Press 's' to save a frame, 'q' to quit."
            )
        else:
            self.get_logger().info(
                f"saving up to {count} frame(s) from {topic} -> {outdir}/ "
                f"(one every {interval}s). Ctrl+C to stop early."
            )

    def _save(self, frame):
        path = os.path.join(self.outdir, f"frame_{self.saved:04d}.png")
        cv2.imwrite(path, frame)
        self.saved += 1
        self.get_logger().info(f"saved {path}")

    def _on_image(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        if self.show:
            cv2.imshow(WINDOW, frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("s"):
                self._save(frame)
            elif key == ord("q"):
                raise SystemExit(0)
            return

        now = time.time()
        if now - self._last < self.interval:
            return
        self._last = now
        self._save(frame)
        if self.count and self.saved >= self.count:
            self.get_logger().info("reached --count; stopping")
            raise SystemExit(0)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--topic", default="/perception/image_annotated")
    ap.add_argument("--outdir", default="docs/figures/tello_frames")
    ap.add_argument("--count", type=int, default=30, help="0 = unlimited (headless mode)")
    ap.add_argument("--interval", type=float, default=0.5, help="seconds between saves (headless mode)")
    ap.add_argument("--show", action="store_true", help="open a live window; 's' saves, 'q' quits")
    args = ap.parse_args()

    rclpy.init()
    node = FrameSaver(args.topic, args.outdir, args.count, args.interval, args.show)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        if args.show:
            cv2.destroyAllWindows()
        node.get_logger().info(f"done: {node.saved} frame(s) written to {args.outdir}/")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
