"""Mission executor — closed-loop goal-sequencing state machine.

This is the EXECUTIVE layer of AutoBee's layered autonomy (see CLAUDE.md). It
no longer flies the drone directly: it pre-plans a list of goals (reusing the
geometric known-map avoidance) and hands them ONE AT A TIME to
flight_controller_node, advancing when the controller reports `reached`. The
controller (reactive layer, ~20 Hz) owns the velocity loop; the LLM agent
(deliberative layer) owns tactical decisions when the executive escalates.

Subscribes:
    /mission_command    std_msgs/String JSON  -- LLM-emitted plan
    /mission_abort      std_msgs/String       -- operator/watchdog kill
    /drone/odom         nav_msgs/Odometry     -- current pose (home capture, RTH)
    /drone/goal_status  std_msgs/String JSON  -- controller handshake
    /mission_directive  std_msgs/String JSON  -- agent's tactical decision
    /anomaly_alert      std_msgs/String JSON  -- typed anomalies (may escalate)
Publishes:
    /drone/goal         std_msgs/String JSON  -- one goal at a time
    /mission_event      std_msgs/String JSON  -- escalation to the LLM agent
    /mission_status     std_msgs/String JSON  -- lifecycle (UNCHANGED contract)

The /mission_status schema and phase labels (CLIMB/SETTLE/TRANSIT_OUT/SURVEY/
TRANSIT_HOME/PRE_DESCEND/DESCEND/DONE/EMERGENCY_LAND*/FAILED) are preserved so
report_node, looming gating, and the Tello driver's land-on-complete keep
working untouched.

Closed-loop notes:
  - Goals are in the odom frame. `home` (xy) and `ground_z` are captured from
    /drone/odom at mission start; cruise_z = ground_z + cruise_altitude.
  - Velocity control + reactive avoidance live in the controller; this node
    only sequences and reacts to reached/timeout/blocked.
  - On `blocked` or `timeout`, the executive escalates to the agent via
    /mission_event and holds position until a /mission_directive arrives.
"""

import json
import math
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from autobee_core.survey_pattern_generator import (
    avoid_obstacles,
    generate,
    is_obstacle_relevant_at_altitude,
)


# World XY of named landmarks in campus.sdf. Must stay in sync with SDF poses.
LANDMARKS = {
    "main_gate":          (-20.0, 0.0),
    "library":            (10.0, 15.0),
    "convocation_ground": (15.0, -15.0),
    "admin":              (-10.0, -18.0),
    "lecture_hall":       (28.0, 8.0),
    "chapel":             (-25.0, 18.0),
    "hostel":             (-18.0, 25.0),
    "dining_hall":        (25.0, -28.0),
}

# Known-map for geometric obstacle avoidance — mirrors campus.sdf.
# name -> (world_x, world_y, safety_radius_m, top_height_m).
KNOWN_OBSTACLES = {
    "tree_1":       (28.0, -22.0, 3.0, 4.6),
    "tree_2":       (28.0, -18.0, 3.0, 4.6),
    "tree_3":       (28.0, -14.0, 3.0, 5.1),
    "tree_4":       (28.0, -10.0, 3.0, 4.6),
    "tree_5":       (0.0,  10.0,  3.0, 5.1),
    "tree_6":       (-5.0, -5.0,  3.0, 4.6),
    "main_gate":    (-20.0, 0.0,  6.5, 4.0),
    "library":      (10.0,  15.0, 7.5, 8.0),
    "admin":        (-10.0, -18.0, 6.5, 6.0),
    "lecture_hall": (28.0,  8.0,  9.0, 6.0),
    "chapel":       (-25.0, 18.0, 7.5, 7.0),
    "hostel":       (-18.0, 25.0, 8.5, 9.0),
    "dining_hall":  (25.0,  -28.0, 6.5, 5.0),
}


class MissionExecutorNode(Node):
    def __init__(self):
        super().__init__("mission_executor_node")

        self.declare_parameter("default_cruise_altitude_m", 11.0)
        self.declare_parameter("cruise_altitude_max_m", 100.0)
        self.declare_parameter("default_survey_radius_m", 6.0)
        self.declare_parameter("survey_radius_max_m", 100.0)
        self.declare_parameter("default_survey_pattern", "orbit")
        self.declare_parameter("max_xy_speed", 1.0)
        self.declare_parameter("settle_time_s", 1.0)
        self.declare_parameter("autostart_mission", "")
        self.declare_parameter("takeoff_world_x", 0.0)
        self.declare_parameter("takeoff_world_y", 0.0)
        self.declare_parameter("avoid_known_obstacles", True)
        self.declare_parameter("landmarks_enabled", True)
        # When true (default), escalate path_blocked / goal_timeout / high-severity
        # anomalies to the LLM agent via /mission_event and wait for a directive.
        # When false, blocked/timeout abort the mission directly (no agent loop).
        self.declare_parameter("escalate_to_agent", True)
        self.declare_parameter("escalate_anomalies", True)
        # Arrival tolerance handed to the controller per goal.
        self.declare_parameter("pos_tol_m", 0.6)
        # If the agent never answers an escalation, give up after this long.
        self.declare_parameter("directive_timeout_s", 20.0)

        gp = self.get_parameter
        self.takeoff_world_xy = (
            float(gp("takeoff_world_x").value), float(gp("takeoff_world_y").value),
        )
        self.avoid_known = bool(gp("avoid_known_obstacles").value)
        self.landmarks_enabled = bool(gp("landmarks_enabled").value)
        self.escalate_to_agent = bool(gp("escalate_to_agent").value)
        self.escalate_anomalies = bool(gp("escalate_anomalies").value)
        self.max_xy_speed = float(gp("max_xy_speed").value)
        self.settle_time = float(gp("settle_time_s").value)
        self.pos_tol = float(gp("pos_tol_m").value)
        self.directive_timeout_s = float(gp("directive_timeout_s").value)

        self.goal_pub = self.create_publisher(String, "/drone/goal", 10)
        self.event_pub = self.create_publisher(String, "/mission_event", 10)
        self.status_pub = self.create_publisher(String, "/mission_status", 10)
        self.create_subscription(String, "/mission_command", self._on_command, 10)
        self.create_subscription(String, "/mission_abort", self._on_abort, 10)
        self.create_subscription(Odometry, "/drone/odom", self._on_odom, 10)
        self.create_subscription(String, "/drone/goal_status", self._on_goal_status, 10)
        self.create_subscription(String, "/mission_directive", self._on_directive, 10)
        self.create_subscription(String, "/anomaly_alert", self._on_anomaly, 10)
        self.create_timer(0.1, self._tick)

        # Mission state.
        self._odom_xyz = None
        self._takeoff_xyz = None
        self._steps = None          # list of step dicts
        self._idx = 0
        self._state = "IDLE"        # IDLE/RUNNING/DWELL/WAIT_DIRECTIVE/TERMINATING
        self._goal_seq = 0
        self._active_goal_id = None
        self._dwell_until = 0.0
        self._dwell_action = "advance"   # "advance" (next step) or "resume" (re-issue current)
        self._escalation_at = 0.0
        self._pending_event_id = None
        self._mission_intent = ""
        self._survey_landmark = None
        self._anomaly_escalated = False
        self._terminating = False
        self._abort_reason = ""

        self.get_logger().info(
            "mission_executor up (closed-loop goal sequencer); "
            "waiting for /mission_command. Hands goals to flight_controller_node."
        )

        autostart = str(gp("autostart_mission").value).strip()
        if autostart:
            self.get_logger().info(f"autostart_mission set; running {autostart!r}")
            self._start_mission_from_json(autostart)

    # ---------- feedback ----------

    def _on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        self._odom_xyz = (p.x, p.y, p.z)

    # ---------- command intake ----------

    def _on_command(self, msg: String):
        if self._state != "IDLE":
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
            reason = f"unsupported action: {command.get('action')!r}"
            self.get_logger().error(reason)
            self._publish_status("failed", phase="REJECTED", reason=reason)
            return
        if self._odom_xyz is None:
            reason = "no /drone/odom yet; cannot anchor goals (is odometry up?)"
            self.get_logger().error(reason)
            self._publish_status("failed", phase="PLANNING", reason=reason)
            return

        cruise = float(command.get("altitude_m", gpval(self, "default_cruise_altitude_m")))
        cruise = min(cruise, float(gpval(self, "cruise_altitude_max_m")))
        radius = float(command.get("radius_m", gpval(self, "default_survey_radius_m")))
        radius = min(radius, float(gpval(self, "survey_radius_max_m")))
        pattern = str(command.get("pattern", gpval(self, "default_survey_pattern")))
        intent = str(command.get("intent", ""))
        landmark = command.get("landmark")
        landmark = landmark.strip().lower() if isinstance(landmark, str) and landmark.strip() else None
        if landmark is not None and landmark not in LANDMARKS:
            self.get_logger().warn(f"unknown landmark {landmark!r}; local-area survey")
            landmark = None
        if landmark is not None and not self.landmarks_enabled:
            self.get_logger().warn(
                f"landmarks_enabled=false; ignoring {landmark!r}, surveying at takeoff origin"
            )
            landmark = None

        # Building target: widen orbit to clear its safety circle.
        if landmark and landmark in KNOWN_OBSTACLES and self.avoid_known:
            _, _, dst_r, dst_top = KNOWN_OBSTACLES[landmark]
            if is_obstacle_relevant_at_altitude(dst_top, cruise) and radius < dst_r + 1.5:
                radius = dst_r + 1.5
                self.get_logger().warn(f"widened radius to {radius:.1f}m to orbit {landmark!r}")

        self.cruise_altitude = cruise
        self.survey_radius = radius
        self.survey_pattern = pattern
        self._survey_landmark = landmark
        self._mission_intent = intent
        self._anomaly_escalated = False

        try:
            self._steps = self._build_steps()
        except Exception as e:
            reason = f"plan build failed: {e}"
            self.get_logger().error(reason)
            self._publish_status("failed", phase="PLANNING", reason=reason)
            self._reset()
            return

        self._takeoff_xyz = self._odom_xyz
        self._idx = 0
        self._state = "RUNNING"
        self._publish_status(
            "started", phase="CLIMB",
            extra={
                "pattern": pattern, "radius_m": radius, "altitude_m": cruise,
                "landmark": landmark, "n_goals": len(self._steps),
            },
        )
        self.get_logger().info(
            f"executing: pattern={pattern} radius={radius}m alt={cruise}m "
            f"landmark={landmark!r} goals={len(self._steps)} intent={intent!r}"
        )
        self._issue_current_goal()

    # ---------- goal-list construction (geometric pre-plan) ----------

    def _build_steps(self):
        hx, hy, hz = self._odom_xyz
        cruise_z = hz + self.cruise_altitude

        if self.survey_pattern == "yaw_only":
            return self._build_yaw_only(hx, hy, hz, cruise_z)
        if self.survey_pattern == "land_now":
            return self._build_land_now(hx, hy, hz, cruise_z)

        steps = []
        steps.append(self._step("CLIMB", "takeoff", hx, hy, cruise_z,
                                yaw=None, avoid=False, dwell_s=self.settle_time))

        # Survey center, drone-relative (takeoff == 0,0), then obstacle list.
        if self._survey_landmark and self._survey_landmark in LANDMARKS:
            lx, ly = LANDMARKS[self._survey_landmark]
            tx, ty = self.takeoff_world_xy
            cx_rel, cy_rel = lx - tx, ly - ty
        else:
            cx_rel, cy_rel = 0.0, 0.0
        obstacles = self._collect_relevant_obstacles()

        prev_rel = (0.0, 0.0)
        # Transit out to the survey center (segmented + avoided).
        if (cx_rel, cy_rel) != (0.0, 0.0):
            transit = self._segment_line((0.0, 0.0), (cx_rel, cy_rel), 3.0)
            if obstacles:
                transit, _ = avoid_obstacles([(x, y, cruise_z) for x, y in transit], obstacles)
                transit = [(x, y) for x, y, _ in transit]
            for i, (rx, ry) in enumerate(transit):
                label = "TRANSIT_OUT" if i == len(transit) - 1 else f"TRANSIT_OUT_{i}"
                steps.append(self._transit_step(label, prev_rel, (rx, ry), hx, hy, cruise_z))
                prev_rel = (rx, ry)
            cx_rel, cy_rel = prev_rel

        # Survey waypoints.
        waypoints = generate(self.survey_pattern, (cx_rel, cy_rel),
                             self.survey_radius, self.cruise_altitude)
        if obstacles:
            waypoints, _ = avoid_obstacles(waypoints, obstacles)
        for i, (rx, ry, _rz) in enumerate(waypoints):
            steps.append(self._transit_step("SURVEY", prev_rel, (rx, ry), hx, hy, cruise_z))
            prev_rel = (rx, ry)

        # Transit home.
        if prev_rel != (0.0, 0.0):
            home_seg = self._segment_line(prev_rel, (0.0, 0.0), 3.0)
            if obstacles:
                home_seg, _ = avoid_obstacles([(x, y, cruise_z) for x, y in home_seg], obstacles)
                home_seg = [(x, y) for x, y, _ in home_seg]
            for i, (rx, ry) in enumerate(home_seg):
                label = "TRANSIT_HOME" if i == len(home_seg) - 1 else f"TRANSIT_HOME_{i}"
                steps.append(self._transit_step(label, prev_rel, (rx, ry), hx, hy, cruise_z))
                prev_rel = (rx, ry)

        steps.append(self._step("PRE_DESCEND", "hover", hx, hy, cruise_z,
                                yaw=None, avoid=True, dwell_s=max(self.settle_time, 3.0)))
        steps.append(self._step("DESCEND", "land", hx, hy, hz,
                                yaw=None, avoid=False))
        return steps

    def _build_yaw_only(self, hx, hy, hz, cruise_z):
        steps = [self._step("CLIMB", "takeoff", hx, hy, cruise_z,
                            yaw=None, avoid=False, dwell_s=self.settle_time)]
        # Closed-loop 360: 8 hover goals at 45-degree yaw increments.
        for k in range(1, 9):
            yaw = self._wrap(k * (math.pi / 4.0))
            steps.append(self._step("YAW_360", "hover", hx, hy, cruise_z,
                                    yaw=yaw, avoid=False, dwell_s=0.6))
        steps.append(self._step("DESCEND", "land", hx, hy, hz, yaw=None, avoid=False))
        return steps

    def _build_land_now(self, hx, hy, hz, cruise_z):
        # Return-to-base: hold at current xy, then descend and land.
        return [
            self._step("PRE_DESCEND", "hover", hx, hy, self._odom_xyz[2],
                       yaw=None, avoid=True, dwell_s=self.settle_time),
            self._step("DESCEND", "land", hx, hy, hz, yaw=None, avoid=False),
        ]

    def _transit_step(self, label, prev_rel, rel, hx, hy, z):
        dist = math.hypot(rel[0] - prev_rel[0], rel[1] - prev_rel[1])
        timeout = dist / max(0.2, self.max_xy_speed) * 2.5 + 8.0
        return self._step(label, "goto", hx + rel[0], hy + rel[1], z,
                          yaw=None, avoid=True, timeout_s=timeout)

    def _step(self, phase, gtype, x, y, z, yaw=None, avoid=True, dwell_s=0.0, timeout_s=None):
        goal = {
            "type": gtype, "x": round(x, 3), "y": round(y, 3), "z": round(z, 3),
            "yaw": yaw, "avoid": bool(avoid), "pos_tol_m": self.pos_tol,
        }
        if timeout_s is not None:
            goal["timeout_s"] = round(timeout_s, 1)
        return {"phase": phase, "goal": goal, "dwell_s": float(dwell_s)}

    # ---------- sequencing ----------

    def _issue_current_goal(self):
        step = self._steps[self._idx]
        self._goal_seq += 1
        self._active_goal_id = f"g{self._goal_seq}"
        goal = dict(step["goal"])
        goal["goal_id"] = self._active_goal_id
        self.goal_pub.publish(String(data=json.dumps(goal)))
        # One phase status per *named* phase (skip noisy intermediate indices).
        phase = step["phase"]
        noisy = phase.rsplit("_", 1)[-1].isdigit()
        if not noisy:
            self._publish_status("phase", phase=phase)

    def _on_goal_status(self, msg: String):
        if self._state not in ("RUNNING",):
            return
        try:
            st = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if st.get("goal_id") != self._active_goal_id:
            return
        state = st.get("state")
        if state == "reached":
            self._on_goal_reached()
        elif state == "blocked":
            self._handle_problem("path_blocked", st)
        elif state == "timeout":
            self._handle_problem("goal_timeout", st)

    def _on_goal_reached(self):
        step = self._steps[self._idx]
        dwell = step.get("dwell_s", 0.0)
        if dwell > 0.0:
            self._state = "DWELL"
            self._dwell_action = "advance"
            self._dwell_until = time.time() + dwell
        else:
            self._advance()

    def _advance(self):
        self._idx += 1
        if self._idx >= len(self._steps):
            self._finish_ok()
            return
        self._state = "RUNNING"
        self._issue_current_goal()

    def _tick(self):
        if self._state == "DWELL" and time.time() >= self._dwell_until:
            if self._dwell_action == "resume":
                self._state = "RUNNING"
                self._issue_current_goal()
            else:
                self._advance()
        elif self._state == "WAIT_DIRECTIVE":
            if (time.time() - self._escalation_at) > self.directive_timeout_s:
                self.get_logger().warn("no directive in time; returning home and landing")
                self._begin_rth("directive_timeout")

    # ---------- escalation / tactical loop (deliberative layer) ----------

    def _handle_problem(self, event_type, status):
        if not self.escalate_to_agent:
            self._begin_abort(f"{event_type} (no agent escalation configured)")
            return
        # Hold position while we wait for the agent's decision.
        self._hold_here()
        self._pending_event_id = f"e{self._goal_seq}"
        self._state = "WAIT_DIRECTIVE"
        self._escalation_at = time.time()
        event = {
            "event_id": self._pending_event_id,
            "event_type": event_type,
            "phase": self._steps[self._idx]["phase"],
            "intent": self._mission_intent,
            "landmark": self._survey_landmark,
            "context": status,
            "wall_time": time.time(),
        }
        self.event_pub.publish(String(data=json.dumps(event)))
        self.get_logger().warn(f"escalating {event_type} -> /mission_event ({self._pending_event_id})")

    def _on_anomaly(self, msg: String):
        if not (self.escalate_anomalies and self._state == "RUNNING" and not self._anomaly_escalated):
            return
        try:
            alert = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if str(alert.get("severity", "")).lower() not in ("high", "critical"):
            return
        self._anomaly_escalated = True
        self._handle_problem("anomaly", alert)

    def _on_directive(self, msg: String):
        if self._state != "WAIT_DIRECTIVE":
            return
        try:
            d = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if d.get("event_id") not in (None, self._pending_event_id):
            return
        action = str(d.get("action", "continue")).lower()
        params = d.get("params") or {}
        reason = str(d.get("reason", ""))
        self.get_logger().info(f"directive: {action} ({reason})")
        self._pending_event_id = None

        if action == "continue":
            self._state = "RUNNING"
            self._issue_current_goal()
        elif action == "hold":
            hold_s = float(params.get("hold_s", 5.0))
            self._hold_here()
            self._state = "DWELL"
            self._dwell_action = "resume"   # hold, THEN retry the same goal
            self._dwell_until = time.time() + hold_s
        elif action in ("skip_goal", "skip"):
            self._advance()
        elif action in ("climb", "adjust_altitude"):
            self._shift_remaining_altitude(float(params.get("delta_alt_m", 2.0)))
            self._state = "RUNNING"
            self._issue_current_goal()
        elif action in ("divert", "rth", "return_home"):
            self._begin_rth(f"directive:{action}")
        elif action == "abort":
            self._begin_abort(reason or "agent abort")
        else:
            self.get_logger().warn(f"unknown directive action {action!r}; continuing")
            self._state = "RUNNING"
            self._issue_current_goal()

    def _shift_remaining_altitude(self, delta):
        for step in self._steps[self._idx:]:
            if step["goal"]["type"] != "land":
                step["goal"]["z"] = round(step["goal"]["z"] + delta, 3)

    # ---------- terminations ----------

    def _hold_here(self):
        if self._odom_xyz is None:
            return
        x, y, z = self._odom_xyz
        self._goal_seq += 1
        self._active_goal_id = f"g{self._goal_seq}"
        goal = {
            "goal_id": self._active_goal_id, "type": "hover",
            "x": round(x, 3), "y": round(y, 3), "z": round(z, 3),
            "yaw": None, "avoid": True, "pos_tol_m": self.pos_tol,
        }
        self.goal_pub.publish(String(data=json.dumps(goal)))

    def _begin_rth(self, reason):
        """Replace remaining plan with: hold -> descend at home -> DONE."""
        if self._takeoff_xyz is None:
            self._begin_abort(reason)
            return
        hx, hy, hz = self._takeoff_xyz
        cruise_z = self._odom_xyz[2] if self._odom_xyz else hz + self.cruise_altitude
        self._steps = [
            self._step("TRANSIT_HOME", "goto", hx, hy, cruise_z, yaw=None, avoid=True,
                       timeout_s=40.0),
            self._step("PRE_DESCEND", "hover", hx, hy, cruise_z, yaw=None, avoid=True,
                       dwell_s=max(self.settle_time, 3.0)),
            self._step("DESCEND", "land", hx, hy, hz, yaw=None, avoid=False),
        ]
        self._idx = 0
        self._state = "RUNNING"
        self.get_logger().warn(f"RTH: {reason}")
        self._issue_current_goal()

    def _begin_abort(self, reason):
        """Emergency: hover briefly, descend in place, terminal FAILED."""
        if self._odom_xyz is None:
            self._publish_status("aborted", phase="FAILED", reason=reason)
            self._reset()
            return
        x, y, z = self._odom_xyz
        self._abort_reason = reason
        self._steps = [
            self._step("EMERGENCY_LAND_HOVER", "hover", x, y, z, yaw=None, avoid=False,
                       dwell_s=1.0),
            self._step("EMERGENCY_LAND_DESCEND", "land", x, y,
                       (self._takeoff_xyz[2] if self._takeoff_xyz else z - z), yaw=None, avoid=False),
        ]
        self._idx = 0
        # Reuse the normal reached-handshake to run the emergency steps, but the
        # _terminating flag makes _finish_ok report 'aborted' instead of 'completed'.
        self._state = "RUNNING"
        self._terminating = True
        self.get_logger().warn(f"ABORT: {reason} -> EMERGENCY_LAND")
        self._issue_current_goal()

    def _on_abort(self, msg: String):
        if self._state == "IDLE":
            self.get_logger().warn("abort received with no active mission; ignoring")
            return
        if getattr(self, "_terminating", False):
            return
        self._begin_abort(msg.data.strip() or "operator abort")

    def _finish_ok(self):
        if getattr(self, "_terminating", False):
            self._publish_status("aborted", phase="FAILED",
                                 reason=getattr(self, "_abort_reason", "aborted"),
                                 extra=self._rth_audit())
        else:
            self._publish_status("completed", phase="DONE", extra=self._rth_audit())
        self._reset()

    def _reset(self):
        self._steps = None
        self._idx = 0
        self._state = "IDLE"
        self._active_goal_id = None
        self._pending_event_id = None
        self._terminating = False

    # ---------- planning helpers (geometric) ----------

    def _collect_relevant_obstacles(self):
        if not self.avoid_known:
            return []
        tx, ty = self.takeoff_world_xy
        relevant = []
        for name, (ox, oy, r, top) in KNOWN_OBSTACLES.items():
            if name == self._survey_landmark:
                continue
            if not is_obstacle_relevant_at_altitude(top, self.cruise_altitude):
                continue
            relevant.append((ox - tx, oy - ty, r))
        return relevant

    @staticmethod
    def _segment_line(start_xy, end_xy, segment_m):
        x0, y0 = start_xy
        x1, y1 = end_xy
        dx, dy = x1 - x0, y1 - y0
        dist = math.hypot(dx, dy)
        if dist <= segment_m + 1e-6:
            return [(x1, y1)]
        n = max(1, int(math.ceil(dist / segment_m)))
        return [(x0 + dx * (i / n), y0 + dy * (i / n)) for i in range(1, n + 1)]

    def _rth_audit(self):
        if self._takeoff_xyz is None or self._odom_xyz is None:
            return None
        x0, y0, z0 = self._takeoff_xyz
        x1, y1, z1 = self._odom_xyz
        return {
            "rth_error_xy_m": round(math.hypot(x1 - x0, y1 - y0), 3),
            "rth_error_xyz_m": round(math.sqrt((x1 - x0) ** 2 + (y1 - y0) ** 2 + (z1 - z0) ** 2), 3),
            "takeoff_pose_xyz": [round(v, 3) for v in self._takeoff_xyz],
            "final_pose_xyz": [round(v, 3) for v in self._odom_xyz],
        }

    def _publish_status(self, event, phase=None, reason="", extra=None):
        payload = {
            "event": event, "phase": phase,
            "intent": self._mission_intent, "landmark": self._survey_landmark,
            "reason": reason, "wall_time": time.time(),
        }
        if extra:
            payload.update(extra)
        self.status_pub.publish(String(data=json.dumps(payload)))

    @staticmethod
    def _wrap(angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def gpval(node, name):
    return node.get_parameter(name).value


def main(args=None):
    rclpy.init(args=args)
    node = MissionExecutorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
