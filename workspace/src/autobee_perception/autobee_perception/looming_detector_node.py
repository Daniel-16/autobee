"""Optical-flow looming detector — collision imminent from monocular video.

Algorithm (classical, no NN):
    1. goodFeaturesToTrack on the previous frame's central region.
    2. Lucas-Kanade sparse flow to the current frame.
    3. For each tracked feature, compute its distance from the image centre
       in both frames. The ratio (distance_now / distance_prev) > 1 means
       the feature is moving outward in image space — i.e. the camera is
       approaching whatever the feature is on. (Pure rotation produces
       sideways motion that doesn't change the radial distance, so this
       implicitly suppresses some yaw false positives.)
    4. Use the median radial expansion ratio to estimate a time-to-contact
       (TTC) from the standard looming formula:
           TTC ≈ Δt / (ratio - 1)        for ratio > 1
       where Δt is the frame interval and ratio is median(d_now/d_prev).
    5. Publish on /perception/looming when TTC < ttc_threshold_s for at
       least N consecutive frames (debounce). The mission_executor abort
       handler does the rest.

Gating: subscribes to /mission_status. When the most recent phase contains
"YAW", "CLIMB", "DESCEND", "SETTLE", "STOP", "TRANSIT_*_YAW", or after a
phase change within `phase_grace_s`, detection is suppressed. Yaw-only
motion in particular produces lots of false positives.

Why not dense flow / why not MiDaS:
    Dense optical flow on CPU + a Whisper + YOLO already running would
    blow the FPS budget. MiDaS gives nicer signal but produces relative
    depth that needs metric calibration. Sparse LK is honest about being
    a backstop — primary avoidance is the geometric known-map planner in
    mission_executor.
"""

import json
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

import cv2


SUPPRESS_PHASE_PATTERNS = (
    "CLIMB", "SETTLE", "PRE_DESCEND", "DESCEND", "STOP",
    "EMERGENCY_LAND_HOVER", "EMERGENCY_LAND_DESCEND",
    "FAILED", "DONE",
)


class LoomingDetectorNode(Node):
    def __init__(self):
        super().__init__("looming_detector_node")

        self.declare_parameter("image_topic", "/drone/camera/image_raw")
        self.declare_parameter("max_features", 100)
        self.declare_parameter("feature_quality", 0.01)
        self.declare_parameter("min_feature_distance_px", 8.0)
        # Central-region mask: only features inside this box (as fractions of
        # frame width/height around centre) feed the looming estimate.
        self.declare_parameter("centre_frac", 0.6)
        # TTC threshold (seconds). When estimated time-to-contact drops below
        # this for `debounce_frames` consecutive frames, fire an alert.
        self.declare_parameter("ttc_threshold_s", 1.5)
        self.declare_parameter("debounce_frames", 3)
        # Quiet window after a phase transition to avoid yaw/climb transients.
        self.declare_parameter("phase_grace_s", 1.0)
        # Re-arm time after a fired alert (don't spam).
        self.declare_parameter("rearm_silence_s", 6.0)
        # Minimum tracked features required to trust the estimate.
        self.declare_parameter("min_tracked_features", 12)

        self.image_topic = str(self.get_parameter("image_topic").value)
        self.max_features = int(self.get_parameter("max_features").value)
        self.feature_quality = float(self.get_parameter("feature_quality").value)
        self.min_feature_dist = float(self.get_parameter("min_feature_distance_px").value)
        self.centre_frac = float(self.get_parameter("centre_frac").value)
        self.ttc_threshold_s = float(self.get_parameter("ttc_threshold_s").value)
        self.debounce_frames = int(self.get_parameter("debounce_frames").value)
        self.phase_grace_s = float(self.get_parameter("phase_grace_s").value)
        self.rearm_silence_s = float(self.get_parameter("rearm_silence_s").value)
        self.min_tracked = int(self.get_parameter("min_tracked_features").value)

        self.bridge = CvBridge()
        self.create_subscription(Image, self.image_topic, self._on_image, 10)
        self.create_subscription(String, "/mission_status", self._on_status, 10)
        self.alert_pub = self.create_publisher(String, "/perception/looming", 10)

        self._prev_gray = None
        self._prev_pts = None
        self._prev_stamp = None
        self._below_threshold_streak = 0
        self._last_alert_t = 0.0
        self._current_phase = None
        self._last_phase_change = 0.0

        self.get_logger().info(
            f"looming_detector up; ttc<{self.ttc_threshold_s}s for "
            f"{self.debounce_frames} frames triggers /perception/looming"
        )

    # ----- mission status gating -----

    def _on_status(self, msg: String):
        try:
            status = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        phase = status.get("phase")
        if phase and phase != self._current_phase:
            self._current_phase = phase
            self._last_phase_change = time.time()
            # Reset feature track on phase change; the visual context just
            # changed (yaw or altitude shift), starting fresh avoids stale
            # vectors faking a looming signal.
            self._prev_gray = None
            self._prev_pts = None
            self._below_threshold_streak = 0

    def _is_phase_suppressed(self):
        phase = self._current_phase or ""
        if not phase:
            return False
        if phase in SUPPRESS_PHASE_PATTERNS:
            return True
        if phase.endswith("_YAW") or phase.startswith("YAW"):
            return True
        if (time.time() - self._last_phase_change) < self.phase_grace_s:
            return True
        return False

    # ----- core flow loop -----

    def _on_image(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"cv_bridge: {e}")
            return
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        h, w = gray.shape
        cx_img, cy_img = w / 2.0, h / 2.0

        stamp = (msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9) if msg.header.stamp.sec else time.time()

        if self._prev_gray is None or self._prev_pts is None or len(self._prev_pts) < self.min_tracked:
            self._reseed_features(gray, w, h)
            self._prev_gray = gray
            self._prev_stamp = stamp
            return

        if self._is_phase_suppressed():
            # Refresh the feature track silently so we resume cleanly.
            self._reseed_features(gray, w, h)
            self._prev_gray = gray
            self._prev_stamp = stamp
            self._below_threshold_streak = 0
            return

        # Lucas-Kanade flow.
        next_pts, status, _err = cv2.calcOpticalFlowPyrLK(
            self._prev_gray, gray, self._prev_pts, None,
            winSize=(21, 21), maxLevel=3,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01),
        )
        if next_pts is None or status is None:
            self._reseed_features(gray, w, h)
            self._prev_gray = gray
            self._prev_stamp = stamp
            return

        good_now = next_pts[status.flatten() == 1]
        good_prev = self._prev_pts[status.flatten() == 1]

        if len(good_now) < self.min_tracked:
            self._reseed_features(gray, w, h)
            self._prev_gray = gray
            self._prev_stamp = stamp
            return

        # Radial expansion — only the central region counts.
        half_w = w * self.centre_frac / 2.0
        half_h = h * self.centre_frac / 2.0

        ratios = []
        for (xn, yn), (xp, yp) in zip(good_now.reshape(-1, 2), good_prev.reshape(-1, 2)):
            if not (cx_img - half_w <= xp <= cx_img + half_w and
                    cy_img - half_h <= yp <= cy_img + half_h):
                continue
            d_prev = float(np.hypot(xp - cx_img, yp - cy_img))
            d_now = float(np.hypot(xn - cx_img, yn - cy_img))
            if d_prev < 4.0:  # too close to centre — small numbers, noisy ratio
                continue
            ratios.append(d_now / d_prev)

        dt = max(1e-3, stamp - (self._prev_stamp or stamp))

        if len(ratios) >= max(8, self.min_tracked // 2):
            median_ratio = float(np.median(ratios))
            if median_ratio > 1.0001:
                ttc = dt / (median_ratio - 1.0)
                if ttc < self.ttc_threshold_s:
                    self._below_threshold_streak += 1
                else:
                    self._below_threshold_streak = 0

                if (
                    self._below_threshold_streak >= self.debounce_frames
                    and (time.time() - self._last_alert_t) >= self.rearm_silence_s
                ):
                    self._fire(ttc, median_ratio, len(ratios))
                    self._last_alert_t = time.time()
                    self._below_threshold_streak = 0
            else:
                self._below_threshold_streak = 0

        # Roll forward.
        self._prev_pts = good_now.reshape(-1, 1, 2).astype(np.float32)
        self._prev_gray = gray
        self._prev_stamp = stamp

        # Periodically reseed so we don't bleed features off-screen.
        if len(self._prev_pts) < self.max_features // 2:
            self._reseed_features(gray, w, h)

    def _reseed_features(self, gray, w, h):
        # Only seed inside the central region — features at the edges of
        # frame have huge image-space motion during yaw and aren't useful.
        half_w = int(w * self.centre_frac / 2.0)
        half_h = int(h * self.centre_frac / 2.0)
        cx, cy = w // 2, h // 2
        mask = np.zeros_like(gray, dtype=np.uint8)
        mask[cy - half_h:cy + half_h, cx - half_w:cx + half_w] = 255
        pts = cv2.goodFeaturesToTrack(
            gray,
            maxCorners=self.max_features,
            qualityLevel=self.feature_quality,
            minDistance=self.min_feature_dist,
            mask=mask,
        )
        self._prev_pts = pts

    def _fire(self, ttc, ratio, n_features):
        msg = {
            "timestamp": time.time(),
            "ttc_s": round(ttc, 3),
            "expansion_ratio": round(ratio, 4),
            "central_features": int(n_features),
            "phase_when_fired": self._current_phase,
        }
        out = String()
        out.data = json.dumps(msg)
        self.alert_pub.publish(out)
        self.get_logger().warn(
            f"LOOMING: ttc={ttc:.2f}s ratio={ratio:.3f} features={n_features} "
            f"phase={self._current_phase!r}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = LoomingDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
