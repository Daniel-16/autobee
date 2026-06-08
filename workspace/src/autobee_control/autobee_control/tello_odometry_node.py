"""Tello telemetry -> unified odometry contract.

Subscribes:
    /drone/state   std_msgs/String (JSON)   -- tello_driver_node telemetry snapshot

Publishes:
    /drone/odom    nav_msgs/Odometry        -- metric (drifty) pose + body twist

This is the REAL-drone half of the AutoBee feedback contract. The Tello has
no GPS and no externally published pose; its onboard estimator does give a
decent *metric velocity* (vgx/vgy/vgz, cm/s) plus a height (ToF/baro). We turn
that into a `nav_msgs/Odometry`:

    twist  <- body-frame velocity straight from the Tello estimator (reliable)
    pose   <- velocity dead-reckoned in the world (odom) frame (drifts over time)
    pose.z <- height sensor directly, NOT integrated vz (height is absolute-ish)

The drift in pose.xy is expected and is exactly why control is velocity-based
(see CLAUDE.md). When ORB-SLAM3 mono-inertial lands later it replaces the
dead-reckoned pose with a drift-corrected one feeding the same /drone/odom —
nothing downstream changes.

Sign convention follows tello_driver_node's REP-103 mapping (x forward, y left,
z up; Tello vgy is rightward-positive so y_left = -vgy). Verify on first flight;
a flipped sign is a one-line fix and milestone-2 closed-loop will surface it.

Deliberately has NO djitellopy import: it consumes the driver's already-published
JSON, so it runs in any environment (and is unit-testable without hardware).
"""

import json
import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String


def _euler_to_quat(roll, pitch, yaw):
    cr, sr = math.cos(roll * 0.5),  math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5),   math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,  # x
        cr * sp * cy + sr * cp * sy,  # y
        cr * cp * sy - sr * sp * cy,  # z
        cr * cp * cy + sr * sp * sy,  # w
    )


class TelloOdometryNode(Node):
    def __init__(self):
        super().__init__("tello_odometry_node")

        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("child_frame_id", "drone")
        # Ignore absurd dt jumps (e.g. first message, or a stalled telemetry
        # link) so a single huge dt doesn't fling the dead-reckoned pose away.
        self.declare_parameter("max_dt_s", 0.5)

        self.frame_id = str(self.get_parameter("frame_id").value)
        self.child_frame_id = str(self.get_parameter("child_frame_id").value)
        self.max_dt = float(self.get_parameter("max_dt_s").value)

        self.pub = self.create_publisher(Odometry, "/drone/odom", 10)
        self.create_subscription(String, "/drone/state", self._on_state, 10)

        # Dead-reckoned world pose accumulator.
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self._last_time = None
        self._last_yaw = None

        self.get_logger().info(
            "tello_odometry_node up: /drone/state -> /drone/odom "
            "(velocity twist + dead-reckoned pose)"
        )

    @staticmethod
    def _as_float(value, default=0.0):
        try:
            return float(value)
        except (TypeError, ValueError):
            return default

    def _on_state(self, msg: String):
        try:
            s = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        now = self.get_clock().now()
        if self._last_time is None:
            self._last_time = now
            self._last_yaw = math.radians(self._as_float(s.get("yaw_deg")))
            return
        dt = (now - self._last_time).nanoseconds / 1e9
        self._last_time = now
        if dt <= 0.0 or dt > self.max_dt:
            # Skip integration for this step but keep yaw fresh for rate calc.
            self._last_yaw = math.radians(self._as_float(s.get("yaw_deg")))
            return

        # Body-frame velocity (cm/s -> m/s), REP-103 axes.
        vx = self._as_float(s.get("vx_cm_s")) * 0.01           # forward
        vy = -self._as_float(s.get("vy_cm_s")) * 0.01          # left (vgy is right+)
        vz = self._as_float(s.get("vz_cm_s")) * 0.01           # up

        roll = math.radians(self._as_float(s.get("roll_deg")))
        pitch = math.radians(self._as_float(s.get("pitch_deg")))
        yaw = math.radians(self._as_float(s.get("yaw_deg")))

        # Rotate horizontal body velocity into the world (odom) frame and
        # integrate. Height comes from the sensor directly (more reliable than
        # integrating vz).
        cos_y, sin_y = math.cos(yaw), math.sin(yaw)
        world_vx = vx * cos_y - vy * sin_y
        world_vy = vx * sin_y + vy * cos_y
        self.x += world_vx * dt
        self.y += world_vy * dt
        height_cm = s.get("height_cm")
        if height_cm is not None:
            self.z = self._as_float(height_cm) * 0.01
        else:
            self.z += vz * dt

        yaw_rate = self._wrap(yaw - (self._last_yaw if self._last_yaw is not None else yaw)) / dt
        self._last_yaw = yaw

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_frame_id
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = self.z
        qx, qy, qz, qw = _euler_to_quat(roll, pitch, yaw)
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        # Twist is in the child (body) frame per REP-103.
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.linear.z = vz
        odom.twist.twist.angular.z = yaw_rate
        self.pub.publish(odom)

    @staticmethod
    def _wrap(angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = TelloOdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
