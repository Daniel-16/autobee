"""Open-loop patrol: climb to cruise altitude, then orbit. Phase-3 perception scaffold."""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class PatrolNode(Node):
    def __init__(self):
        super().__init__("patrol_node")

        self.declare_parameter("takeoff_time", 4.0)
        self.declare_parameter("climb_speed", 1.2)
        self.declare_parameter("orbit_linear", 1.0)
        self.declare_parameter("orbit_angular", 0.3)
        self.declare_parameter("cmd_vel_topic", "/drone/cmd_vel")

        self.takeoff_time = float(self.get_parameter("takeoff_time").value)
        self.climb_speed = float(self.get_parameter("climb_speed").value)
        self.orbit_linear = float(self.get_parameter("orbit_linear").value)
        self.orbit_angular = float(self.get_parameter("orbit_angular").value)
        cmd_vel_topic = self.get_parameter("cmd_vel_topic").value

        self.pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(0.05, self._tick)

        self.get_logger().info(
            f"patrol_node up: takeoff={self.takeoff_time}s climb={self.climb_speed}m/s "
            f"orbit=({self.orbit_linear}m/s, {self.orbit_angular}rad/s) -> {cmd_vel_topic}"
        )

    def _tick(self):
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        cmd = Twist()
        if elapsed < self.takeoff_time:
            cmd.linear.z = self.climb_speed
        else:
            cmd.linear.x = self.orbit_linear
            cmd.angular.z = self.orbit_angular
        self.pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = PatrolNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
