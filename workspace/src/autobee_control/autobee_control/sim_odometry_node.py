"""Sim ground-truth pose -> unified odometry contract.

Subscribes:
    /drone/pose    geometry_msgs/Pose   -- Gazebo PosePublisher (ground truth)

Publishes:
    /drone/odom    nav_msgs/Odometry    -- pose + body twist (Tello-like)

This is the SIM half of the AutoBee feedback contract, the mirror of
tello_odometry_node. Gazebo hands us a perfect pose but no velocity, so we:

    pose   <- ground-truth pose (optionally perturbed)
    twist  <- finite-difference of pose, rotated into the body frame

Crucially it injects optional Gaussian noise (``vel_noise_std`` / ``pos_noise_std``)
so the milestone-2 controller is tuned against a Tello-like feedback signal
rather than perfect data. Tuning against ground truth would not survive the
swap to the real drone (see CLAUDE.md sim-to-real note). Set the std params to
0.0 for a clean-feedback debug run.

geometry_msgs/Pose carries no timestamp, so dt comes from the node clock
(which honours use_sim_time when launched in sim).
"""

import math
import random

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry


def _quat_to_yaw(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class SimOdometryNode(Node):
    def __init__(self):
        super().__init__("sim_odometry_node")

        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("child_frame_id", "drone")
        # Tello-like feedback noise. Defaults approximate the real drone's
        # estimator jitter so the controller transfers. Set to 0.0 for a
        # perfect-feedback debug run.
        self.declare_parameter("vel_noise_std", 0.05)   # m/s, per axis
        self.declare_parameter("pos_noise_std", 0.0)    # m, per axis
        self.declare_parameter("max_dt_s", 0.5)

        self.frame_id = str(self.get_parameter("frame_id").value)
        self.child_frame_id = str(self.get_parameter("child_frame_id").value)
        self.vel_noise = float(self.get_parameter("vel_noise_std").value)
        self.pos_noise = float(self.get_parameter("pos_noise_std").value)
        self.max_dt = float(self.get_parameter("max_dt_s").value)

        self.pub = self.create_publisher(Odometry, "/drone/odom", 10)
        self.create_subscription(Pose, "/drone/pose", self._on_pose, 10)

        self._last_pos = None
        self._last_yaw = None
        self._last_time = None

        self.get_logger().info(
            "sim_odometry_node up: /drone/pose -> /drone/odom "
            f"(vel_noise={self.vel_noise} m/s, pos_noise={self.pos_noise} m)"
        )

    def _on_pose(self, msg: Pose):
        now = self.get_clock().now()
        px, py, pz = msg.position.x, msg.position.y, msg.position.z
        yaw = _quat_to_yaw(
            msg.orientation.x, msg.orientation.y,
            msg.orientation.z, msg.orientation.w,
        )

        if self._last_time is None:
            self._last_time = now
            self._last_pos = (px, py, pz)
            self._last_yaw = yaw
            return
        dt = (now - self._last_time).nanoseconds / 1e9
        if dt <= 0.0 or dt > self.max_dt:
            self._last_time = now
            self._last_pos = (px, py, pz)
            self._last_yaw = yaw
            return

        # World-frame velocity by finite difference.
        lx, ly, lz = self._last_pos
        world_vx = (px - lx) / dt
        world_vy = (py - ly) / dt
        world_vz = (pz - lz) / dt
        yaw_rate = self._wrap(yaw - self._last_yaw) / dt

        self._last_time = now
        self._last_pos = (px, py, pz)
        self._last_yaw = yaw

        # Rotate world velocity into the body (child) frame for the twist.
        cos_y, sin_y = math.cos(yaw), math.sin(yaw)
        body_vx = world_vx * cos_y + world_vy * sin_y
        body_vy = -world_vx * sin_y + world_vy * cos_y
        body_vz = world_vz

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_frame_id
        odom.pose.pose.position.x = px + self._noise(self.pos_noise)
        odom.pose.pose.position.y = py + self._noise(self.pos_noise)
        odom.pose.pose.position.z = pz + self._noise(self.pos_noise)
        odom.pose.pose.orientation = msg.orientation
        odom.twist.twist.linear.x = body_vx + self._noise(self.vel_noise)
        odom.twist.twist.linear.y = body_vy + self._noise(self.vel_noise)
        odom.twist.twist.linear.z = body_vz + self._noise(self.vel_noise)
        odom.twist.twist.angular.z = yaw_rate
        self.pub.publish(odom)

    @staticmethod
    def _noise(std):
        return random.gauss(0.0, std) if std > 0.0 else 0.0

    @staticmethod
    def _wrap(angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = SimOdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
