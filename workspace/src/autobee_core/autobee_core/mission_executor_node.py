"""Mission executor — open-loop timed survey, command-driven.

Waits on /mission_command (std_msgs/String JSON) before flying. The LLM agent
publishes the JSON; this node parses it, pre-computes a timed twist schedule
from the spiral/orbit waypoints, and plays it back on /drone/cmd_vel.

Pose feedback through ros_gz_bridge is unreliable in this Fortress build, so
the executor runs open-loop. Demo-grade: no closed-loop correction. Trades
drift for reliability.

Schedule phases:
    CLIMB    rise at max_z_speed for cruise_altitude / max_z_speed seconds
    SETTLE   short pause at cruise altitude
    SURVEY   for each waypoint segment: yaw to target heading, then drive forward
    DESCEND  drop at max_z_speed back to ground
    DONE     publish zero Twist forever
"""

import json
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from autobee_core.survey_pattern_generator import generate


class MissionExecutorNode(Node):
    def __init__(self):
        super().__init__("mission_executor_node")

        self.declare_parameter("default_cruise_altitude_m", 30.0)
        self.declare_parameter("default_survey_radius_m", 15.0)
        self.declare_parameter("default_survey_pattern", "spiral")
        self.declare_parameter("max_xy_speed", 2.0)
        self.declare_parameter("max_z_speed", 1.5)
        self.declare_parameter("max_yaw_rate", 0.8)
        self.declare_parameter("settle_time_s", 1.0)
        self.declare_parameter("autostart_mission", "")

        self.max_xy_speed = float(self.get_parameter("max_xy_speed").value)
        self.max_z_speed = float(self.get_parameter("max_z_speed").value)
        self.max_yaw_rate = float(self.get_parameter("max_yaw_rate").value)
        self.settle_time = float(self.get_parameter("settle_time_s").value)

        self.cmd_pub = self.create_publisher(Twist, "/drone/cmd_vel", 10)
        self.create_subscription(String, "/mission_command", self._on_command, 10)
        self.create_timer(0.05, self._tick)

        self.schedule = None
        self.start_time = None
        self._last_phase = None

        self.get_logger().info(
            "mission_executor up (open-loop, command-driven); "
            "waiting for JSON on /mission_command"
        )

        autostart = str(self.get_parameter("autostart_mission").value).strip()
        if autostart:
            self.get_logger().info(f"autostart_mission set; running {autostart!r}")
            self._start_mission_from_json(autostart)

    def _on_command(self, msg: String):
        if self.schedule is not None:
            self.get_logger().warn("mission already in progress; ignoring new command")
            return
        self._start_mission_from_json(msg.data)

    def _start_mission_from_json(self, raw):
        try:
            command = json.loads(raw)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"invalid JSON on /mission_command: {e}")
            return

        if command.get("action") != "survey_local_area":
            self.get_logger().error(f"unsupported action: {command.get('action')!r}")
            return

        cruise = float(command.get(
            "altitude_m", self.get_parameter("default_cruise_altitude_m").value))
        radius = float(command.get(
            "radius_m", self.get_parameter("default_survey_radius_m").value))
        pattern = str(command.get(
            "pattern", self.get_parameter("default_survey_pattern").value))
        intent = str(command.get("intent", ""))

        self.cruise_altitude = cruise
        self.survey_radius = radius
        self.survey_pattern = pattern

        self.schedule, total = self._build_schedule()
        self.start_time = self.get_clock().now()

        self.get_logger().info(
            f"executing: pattern={pattern} radius={radius}m altitude={cruise}m "
            f"duration={total:.1f}s intent={intent!r}"
        )

    def _build_schedule(self):
        events = []
        t = 0.0

        climb_dur = self.cruise_altitude / self.max_z_speed
        cmd = Twist()
        cmd.linear.z = self.max_z_speed
        events.append((t, t + climb_dur, cmd, "CLIMB"))
        t += climb_dur

        events.append((t, t + self.settle_time, Twist(), "SETTLE"))
        t += self.settle_time

        waypoints = generate(
            self.survey_pattern,
            (0.0, 0.0),
            self.survey_radius,
            self.cruise_altitude,
        )

        prev_x, prev_y = 0.0, 0.0
        prev_heading = 0.0

        for i, (wx, wy, _wz) in enumerate(waypoints):
            dx = wx - prev_x
            dy = wy - prev_y
            distance = math.hypot(dx, dy)
            if distance < 1e-3:
                continue

            target_heading = math.atan2(dy, dx)
            heading_diff = self._wrap(target_heading - prev_heading)

            yaw_dur = abs(heading_diff) / self.max_yaw_rate
            if yaw_dur > 0.05:
                yaw_cmd = Twist()
                yaw_cmd.angular.z = self.max_yaw_rate * (1.0 if heading_diff > 0 else -1.0)
                events.append((t, t + yaw_dur, yaw_cmd, f"YAW->{i}"))
                t += yaw_dur

            fwd_dur = distance / self.max_xy_speed
            fwd_cmd = Twist()
            fwd_cmd.linear.x = self.max_xy_speed
            events.append((t, t + fwd_dur, fwd_cmd, f"FWD->{i}"))
            t += fwd_dur

            prev_x, prev_y, prev_heading = wx, wy, target_heading

        events.append((t, t + self.settle_time, Twist(), "PRE_DESCEND"))
        t += self.settle_time

        descend_dur = self.cruise_altitude / self.max_z_speed
        descend_cmd = Twist()
        descend_cmd.linear.z = -self.max_z_speed
        events.append((t, t + descend_dur, descend_cmd, "DESCEND"))
        t += descend_dur

        events.append((t, float("inf"), Twist(), "DONE"))

        return events, t

    def _tick(self):
        if self.schedule is None:
            return
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        for (t_start, t_end, cmd, label) in self.schedule:
            if t_start <= elapsed < t_end:
                self.cmd_pub.publish(cmd)
                if label != self._last_phase and not label.startswith(("YAW", "FWD")):
                    self.get_logger().info(f"phase: {label} (t={elapsed:.1f}s)")
                    self._last_phase = label
                return
        self.cmd_pub.publish(Twist())

    @staticmethod
    def _wrap(angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


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
