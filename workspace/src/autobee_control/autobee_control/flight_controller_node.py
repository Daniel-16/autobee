"""Closed-loop velocity/heading controller — the reactive inner loop.

Subscribes:
    /drone/odom           nav_msgs/Odometry    -- unified feedback contract
    /drone/goal           std_msgs/String JSON -- one goal at a time (executive)
    /perception/looming   std_msgs/String JSON -- optical-flow looming alert
    /perception/obstacles std_msgs/String JSON -- YOLO non-person threat

Publishes:
    /drone/cmd_vel        geometry_msgs/Twist  -- control output
    /drone/goal_status    std_msgs/String JSON -- per-goal handshake to executive

This replaces the old open-loop timed "tape" in mission_executor. It runs a
deterministic ~20 Hz loop, so the LLM is never in the inner loop (see CLAUDE.md
layered-autonomy design). The executive (mission_executor) hands it goals and
advances on the `reached` status; the agent never talks to this node directly.

Control law (velocity-based, Tello-friendly — Tello estimates velocity well,
position poorly):
    P on world position error -> body-frame velocity, damped by measured body
    velocity (the D term, straight off the odom twist). Heading either tracks a
    commanded yaw, or faces the direction of travel while transiting and holds
    on arrival. Tello is holonomic (can strafe), so we command vx AND vy rather
    than the old "yaw-then-forward".

Reactive avoidance (milestone 4 — steers, doesn't just abort):
    - looming (no bearing, central): brake forward + climb + back off.
    - obstacle (signed cx offset from YOLO): brake forward + push laterally to
      the *opposite* side + slight climb.
    Avoidance biases the velocity command; it does not kill the mission. If a
    threat persists past `block_timeout_s` without clearing, the controller
    publishes goal_status state=blocked so the executive can escalate to the
    agent (tactical decision) instead of silently grinding.

Goal schema (/drone/goal):
    {"goal_id","type":"goto|hover|land|takeoff","x","y","z","yaw":<rad|null>,
     "pos_tol_m","yaw_tol_rad","timeout_s","avoid":<bool>}
Status schema (/drone/goal_status):
    {"goal_id","state":"active|reached|timeout|blocked","dist_m","yaw_err_rad","avoiding"}
"""

import json
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String


def _quat_to_yaw(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _wrap(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


class FlightControllerNode(Node):
    def __init__(self):
        super().__init__("flight_controller_node")

        self.declare_parameter("rate_hz", 20.0)
        # Frame the velocity command is published in. "body" (REP-103, what the
        # Tello and gz VelocityControl expect) or "world" (insurance: flip this
        # if the sim drone spirals instead of converging — means the plugin
        # interprets cmd_vel in the world frame).
        self.declare_parameter("cmd_frame", "body")
        # Gains. P on position, D on measured body velocity (from odom twist).
        self.declare_parameter("kp_xy", 0.6)
        self.declare_parameter("kd_xy", 0.15)
        self.declare_parameter("kp_z", 0.8)
        self.declare_parameter("kp_yaw", 1.0)
        # Velocity envelope (match the old executor; demo_tello overrides these).
        self.declare_parameter("max_xy_speed", 1.0)
        self.declare_parameter("max_z_speed", 0.6)
        self.declare_parameter("max_yaw_rate", 0.6)
        # Face the direction of travel when farther than this from the goal;
        # hold heading once inside it (stable camera on station).
        self.declare_parameter("face_travel_dist_m", 1.0)
        # Arrival / settling.
        self.declare_parameter("default_pos_tol_m", 0.5)
        self.declare_parameter("default_yaw_tol_rad", 0.15)
        self.declare_parameter("settle_speed_mps", 0.2)
        self.declare_parameter("default_timeout_s", 30.0)
        # Failsafe: zero the command if odom goes stale.
        self.declare_parameter("odom_timeout_s", 1.0)
        # Reactive avoidance.
        self.declare_parameter("avoidance_enabled", True)
        self.declare_parameter("looming_react_s", 1.5)
        self.declare_parameter("obstacle_react_s", 1.0)
        self.declare_parameter("avoid_backoff_mps", 0.3)
        self.declare_parameter("avoid_climb_mps", 0.3)
        self.declare_parameter("avoid_lateral_mps", 0.4)
        # Suppress avoidance while turning hard (mirrors looming's yaw gating).
        self.declare_parameter("avoid_yaw_gate_rad", 0.6)
        # If avoidance can't clear within this, tell the executive we're blocked.
        self.declare_parameter("block_timeout_s", 6.0)

        g = self.get_parameter
        self.rate_hz = float(g("rate_hz").value)
        self.cmd_frame = str(g("cmd_frame").value).lower()
        self.kp_xy = float(g("kp_xy").value)
        self.kd_xy = float(g("kd_xy").value)
        self.kp_z = float(g("kp_z").value)
        self.kp_yaw = float(g("kp_yaw").value)
        self.max_xy = float(g("max_xy_speed").value)
        self.max_z = float(g("max_z_speed").value)
        self.max_yaw = float(g("max_yaw_rate").value)
        self.face_travel_dist = float(g("face_travel_dist_m").value)
        self.default_pos_tol = float(g("default_pos_tol_m").value)
        self.default_yaw_tol = float(g("default_yaw_tol_rad").value)
        self.settle_speed = float(g("settle_speed_mps").value)
        self.default_timeout = float(g("default_timeout_s").value)
        self.odom_timeout = float(g("odom_timeout_s").value)
        self.avoidance_enabled = bool(g("avoidance_enabled").value)
        self.looming_react_s = float(g("looming_react_s").value)
        self.obstacle_react_s = float(g("obstacle_react_s").value)
        self.avoid_backoff = float(g("avoid_backoff_mps").value)
        self.avoid_climb = float(g("avoid_climb_mps").value)
        self.avoid_lateral = float(g("avoid_lateral_mps").value)
        self.avoid_yaw_gate = float(g("avoid_yaw_gate_rad").value)
        self.block_timeout_s = float(g("block_timeout_s").value)

        self.cmd_pub = self.create_publisher(Twist, "/drone/cmd_vel", 10)
        self.status_pub = self.create_publisher(String, "/drone/goal_status", 10)
        self.create_subscription(Odometry, "/drone/odom", self._on_odom, 10)
        self.create_subscription(String, "/drone/goal", self._on_goal, 10)
        self.create_subscription(String, "/perception/looming", self._on_looming, 10)
        self.create_subscription(String, "/perception/obstacles", self._on_obstacle, 10)
        self.create_timer(1.0 / self.rate_hz, self._tick)

        self._odom = None
        self._odom_time = 0.0
        self._goal = None
        self._goal_start = 0.0
        self._terminal_state = None        # reached/timeout/blocked already sent
        self._avoiding_since = None        # wall time avoidance went continuously active
        self._looming_until = 0.0
        self._obstacle_until = 0.0
        self._obstacle_side = 0.0          # sign of cx offset (+1 right, -1 left)

        self.get_logger().info(
            f"flight_controller up @ {self.rate_hz:.0f} Hz "
            f"(avoidance={'on' if self.avoidance_enabled else 'off'}); "
            "goals on /drone/goal -> /drone/cmd_vel, handshake on /drone/goal_status"
        )

    # ---------- inputs ----------

    def _on_odom(self, msg: Odometry):
        self._odom = msg
        self._odom_time = time.time()

    def _on_goal(self, msg: String):
        try:
            goal = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error("invalid JSON on /drone/goal")
            return
        self._goal = goal
        self._goal_start = time.time()
        self._terminal_state = None
        self._avoiding_since = None
        self.get_logger().info(
            f"goal {goal.get('goal_id')!r} type={goal.get('type')} "
            f"-> ({goal.get('x')}, {goal.get('y')}, {goal.get('z')})"
        )

    def _on_looming(self, msg: String):
        if not self.avoidance_enabled:
            return
        self._looming_until = time.time() + self.looming_react_s

    def _on_obstacle(self, msg: String):
        if not self.avoidance_enabled:
            return
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        # Prefer the signed offset (positive = right of frame centre). Fall back
        # to 0 (treat as dead-ahead) if an older YOLO node only sends the
        # unsigned centre_offset_frac.
        off = payload.get("cx_offset_frac")
        self._obstacle_side = (1.0 if off > 0 else -1.0) if isinstance(off, (int, float)) and off != 0 else 0.0
        self._obstacle_until = time.time() + self.obstacle_react_s

    # ---------- control loop ----------

    def _tick(self):
        if self._goal is None:
            self.cmd_pub.publish(Twist())
            return
        # Failsafe: no fresh odom -> hold still.
        if self._odom is None or (time.time() - self._odom_time) > self.odom_timeout:
            self.cmd_pub.publish(Twist())
            return

        goal = self._goal
        gtype = goal.get("type", "goto")
        p = self._odom.pose.pose.position
        yaw = _quat_to_yaw(
            self._odom.pose.pose.orientation.x, self._odom.pose.pose.orientation.y,
            self._odom.pose.pose.orientation.z, self._odom.pose.pose.orientation.w,
        )
        tw = self._odom.twist.twist
        mvx, mvy = tw.linear.x, tw.linear.y       # measured body velocity

        gx = float(goal.get("x", p.x))
        gy = float(goal.get("y", p.y))
        gz = float(goal.get("z", p.z))
        ex, ey, ez = gx - p.x, gy - p.y, gz - p.z
        dist_h = math.hypot(ex, ey)
        dist_3d = math.sqrt(ex * ex + ey * ey + ez * ez)

        # World position error -> body frame.
        cy, sy = math.cos(yaw), math.sin(yaw)
        bx = ex * cy + ey * sy
        by = -ex * sy + ey * cy

        # Velocity command: P(position) - D(measured velocity).
        vx = self.kp_xy * bx - self.kd_xy * mvx
        vy = self.kp_xy * by - self.kd_xy * mvy
        vz = self.kp_z * ez

        # Heading: explicit yaw, else face travel while transiting, else hold.
        goal_yaw = goal.get("yaw")
        if isinstance(goal_yaw, (int, float)):
            yaw_err = _wrap(float(goal_yaw) - yaw)
        elif gtype in ("goto",) and dist_h > self.face_travel_dist:
            yaw_err = _wrap(math.atan2(ey, ex) - yaw)
        else:
            yaw_err = 0.0
        vyaw = self.kp_yaw * yaw_err

        # Reactive avoidance — only during horizontal travel, not climb/descend,
        # and not while turning hard (mirrors looming's own gating).
        avoiding = False
        now = time.time()
        avoid_ok = (
            self.avoidance_enabled
            and bool(goal.get("avoid", True))
            and gtype in ("goto", "hover")
            and abs(yaw_err) < self.avoid_yaw_gate
        )
        if avoid_ok and now < self._looming_until:
            vx = -self.avoid_backoff       # stop approaching, back off
            vz = max(vz, self.avoid_climb)  # climb over
            avoiding = True
        if avoid_ok and now < self._obstacle_until:
            vx = min(vx, 0.0)              # don't drive into it
            # Push to the side opposite the obstacle (side 0 -> default left).
            vy += -self.avoid_lateral if self._obstacle_side > 0 else self.avoid_lateral
            vz = max(vz, self.avoid_climb)
            avoiding = True

        # Optionally rotate the horizontal command into the world frame.
        if self.cmd_frame == "world":
            wvx = vx * cy - vy * sy
            wvy = vx * sy + vy * cy
            vx, vy = wvx, wvy

        # Clamp to the envelope and publish.
        cmd = Twist()
        cmd.linear.x = _clamp(vx, -self.max_xy, self.max_xy)
        cmd.linear.y = _clamp(vy, -self.max_xy, self.max_xy)
        cmd.linear.z = _clamp(vz, -self.max_z, self.max_z)
        cmd.angular.z = _clamp(vyaw, -self.max_yaw, self.max_yaw)
        self.cmd_pub.publish(cmd)

        self._update_status(goal, gtype, dist_3d, yaw_err, mvx, mvy, avoiding, now)

    def _update_status(self, goal, gtype, dist_3d, yaw_err, mvx, mvy, avoiding, now):
        # Track continuous avoidance for block detection.
        if avoiding:
            if self._avoiding_since is None:
                self._avoiding_since = now
        else:
            self._avoiding_since = None

        if self._terminal_state is not None:
            # Already reported terminal for this goal; keep station-keeping but
            # don't spam status (the executive has moved on / will send next goal).
            return

        pos_tol = float(goal.get("pos_tol_m", self.default_pos_tol))
        yaw_tol = float(goal.get("yaw_tol_rad", self.default_yaw_tol))
        timeout_s = float(goal.get("timeout_s", self.default_timeout))
        speed = math.hypot(mvx, mvy)

        if (
            dist_3d < pos_tol
            and abs(yaw_err) < yaw_tol
            and speed < self.settle_speed
            and not avoiding
        ):
            self._publish_status(goal, "reached", dist_3d, yaw_err, avoiding)
            self._terminal_state = "reached"
            return
        if self._avoiding_since is not None and (now - self._avoiding_since) > self.block_timeout_s:
            self._publish_status(goal, "blocked", dist_3d, yaw_err, avoiding)
            self._terminal_state = "blocked"
            return
        if (now - self._goal_start) > timeout_s:
            self._publish_status(goal, "timeout", dist_3d, yaw_err, avoiding)
            self._terminal_state = "timeout"
            return

    def _publish_status(self, goal, state, dist_3d, yaw_err, avoiding):
        payload = {
            "goal_id": goal.get("goal_id"),
            "state": state,
            "dist_m": round(dist_3d, 3),
            "yaw_err_rad": round(yaw_err, 3),
            "avoiding": bool(avoiding),
            "wall_time": time.time(),
        }
        self.status_pub.publish(String(data=json.dumps(payload)))
        if state != "active":
            self.get_logger().info(f"goal {goal.get('goal_id')!r}: {state} (dist={dist_3d:.2f}m)")


def main(args=None):
    rclpy.init(args=args)
    node = FlightControllerNode()
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
