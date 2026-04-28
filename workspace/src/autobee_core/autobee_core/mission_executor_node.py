"""Mission executor — flies a survey pattern using ground-truth /drone/pose feedback.

State machine:
    INIT     wait for first pose, snapshot takeoff_pose, generate waypoints
    CLIMB    rise straight up to cruise_altitude_m
    SURVEY   visit each waypoint via yaw+forward control
    RETURN   fly back to takeoff XY at cruise altitude
    DESCEND  drop straight down to takeoff Z
    DONE     publish zero Twist forever

Hardcoded survey_local_area mission for now; LLM agent will emit this JSON later.
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped

from autobee_core.survey_pattern_generator import generate


class MissionExecutorNode(Node):
    def __init__(self):
        super().__init__("mission_executor_node")

        self.declare_parameter("cruise_altitude_m", 12.0)
        self.declare_parameter("survey_radius_m", 20.0)
        self.declare_parameter("survey_pattern", "spiral")
        self.declare_parameter("waypoint_tolerance_m", 1.5)
        self.declare_parameter("max_xy_speed", 2.5)
        self.declare_parameter("max_z_speed", 1.5)
        self.declare_parameter("max_yaw_rate", 0.8)

        self.cruise_altitude = float(self.get_parameter("cruise_altitude_m").value)
        self.survey_radius = float(self.get_parameter("survey_radius_m").value)
        self.survey_pattern = str(self.get_parameter("survey_pattern").value)
        self.waypoint_tol = float(self.get_parameter("waypoint_tolerance_m").value)
        self.max_xy_speed = float(self.get_parameter("max_xy_speed").value)
        self.max_z_speed = float(self.get_parameter("max_z_speed").value)
        self.max_yaw_rate = float(self.get_parameter("max_yaw_rate").value)

        self.kp_xy = 0.7
        self.kp_z = 1.0
        self.kp_yaw = 1.5

        self.state = "INIT"
        self.takeoff_pose = None
        self.current_pose = None
        self.current_yaw = 0.0
        self.waypoints = []
        self.wp_idx = 0

        self.cmd_pub = self.create_publisher(Twist, "/drone/cmd_vel", 10)
        self.create_subscription(PoseStamped, "/drone/pose", self._on_pose, 10)
        self.create_timer(0.05, self._tick)

        self.get_logger().info(
            f"mission_executor up: cruise={self.cruise_altitude}m "
            f"radius={self.survey_radius}m pattern={self.survey_pattern}"
        )

    def _on_pose(self, msg: PoseStamped):
        self.current_pose = (
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        )
        q = msg.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def _tick(self):
        if self.current_pose is None:
            return

        if self.state == "INIT":
            self.takeoff_pose = self.current_pose
            self.waypoints = generate(
                self.survey_pattern,
                (self.takeoff_pose[0], self.takeoff_pose[1]),
                self.survey_radius,
                self.cruise_altitude,
            )
            self.wp_idx = 0
            self.state = "CLIMB"
            self.get_logger().info(
                f"takeoff_pose snapshot: ({self.takeoff_pose[0]:.2f}, "
                f"{self.takeoff_pose[1]:.2f}, {self.takeoff_pose[2]:.2f}); "
                f"{len(self.waypoints)} waypoints generated; climbing to {self.cruise_altitude}m"
            )
            return

        if self.state == "CLIMB":
            cmd = Twist()
            cmd.linear.z = self.max_z_speed
            self.cmd_pub.publish(cmd)
            if self.current_pose[2] >= self.cruise_altitude - 0.3:
                self.state = "SURVEY"
                self.get_logger().info("reached cruise altitude; starting survey")
            return

        if self.state == "SURVEY":
            target = self.waypoints[self.wp_idx]
            self._fly_to(target)
            if self._xy_distance_to(target) < self.waypoint_tol:
                self.wp_idx += 1
                if self.wp_idx >= len(self.waypoints):
                    self.state = "RETURN"
                    self.get_logger().info("survey complete; returning home")
                else:
                    self.get_logger().info(
                        f"waypoint {self.wp_idx}/{len(self.waypoints)} reached"
                    )
            return

        if self.state == "RETURN":
            target = (self.takeoff_pose[0], self.takeoff_pose[1], self.cruise_altitude)
            self._fly_to(target)
            if self._xy_distance_to(target) < self.waypoint_tol:
                self.state = "DESCEND"
                self.get_logger().info("over takeoff_pose; descending")
            return

        if self.state == "DESCEND":
            cmd = Twist()
            cmd.linear.z = -0.5
            self.cmd_pub.publish(cmd)
            if self.current_pose[2] <= self.takeoff_pose[2] + 0.2:
                self.state = "DONE"
                self.get_logger().info("mission complete; holding zero velocity")
            return

        if self.state == "DONE":
            self.cmd_pub.publish(Twist())
            return

    def _fly_to(self, target):
        cx, cy, cz = self.current_pose
        tx, ty, tz = target
        dx, dy, dz = tx - cx, ty - cy, tz - cz
        dist_xy = math.hypot(dx, dy)

        heading_to_target = math.atan2(dy, dx)
        heading_err = self._wrap(heading_to_target - self.current_yaw)
        yaw_rate = self._clamp(
            heading_err * self.kp_yaw, -self.max_yaw_rate, self.max_yaw_rate
        )

        align = max(math.cos(heading_err), 0.0)
        fwd = self._clamp(dist_xy * self.kp_xy * align, 0.0, self.max_xy_speed)
        vz = self._clamp(dz * self.kp_z, -self.max_z_speed, self.max_z_speed)

        cmd = Twist()
        cmd.linear.x = fwd
        cmd.linear.z = vz
        cmd.angular.z = yaw_rate
        self.cmd_pub.publish(cmd)

    def _xy_distance_to(self, target):
        cx, cy, _ = self.current_pose
        tx, ty, _ = target
        return math.hypot(tx - cx, ty - cy)

    @staticmethod
    def _wrap(angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    @staticmethod
    def _clamp(v, lo, hi):
        return max(lo, min(hi, v))


def main(args=None):
    rclpy.init(args=args)
    node = MissionExecutorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop = Twist()
        for _ in range(5):
            node.cmd_pub.publish(stop)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
