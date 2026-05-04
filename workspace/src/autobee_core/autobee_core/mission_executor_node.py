"""Mission executor — open-loop timed survey, command-driven, with abort.

Subscribes:
    /mission_command   std_msgs/String (JSON) -- LLM-emitted plan
    /mission_abort     std_msgs/String         -- operator/watchdog kill
Publishes:
    /drone/cmd_vel     geometry_msgs/Twist
    /mission_status    std_msgs/String (JSON)  -- lifecycle state + outcome

Pose feedback through ros_gz_bridge is unreliable in this Fortress build, so
the executor runs open-loop on a pre-computed timed twist schedule. Trades
drift for reliability. Closed-loop control is gated on the SLAM phase.

Landmark handling: when /mission_command sets a "landmark" field, the executor
looks up its world XY from LANDMARKS, prepends a TRANSIT_OUT phase to fly
there from takeoff_world_xy, runs the survey centered on the landmark, then
appends a TRANSIT_HOME phase before descent. Landmark navigation is still
open-loop (timed yaw + forward) — it will be replaced with closed-loop nav
once /drone/pose from SLAM is reliable.

Schedule phases (published as mission_status state transitions):
    CLIMB           rise to cruise altitude
    SETTLE          brief pause
    TRANSIT_OUT     fly from takeoff to landmark XY (only if landmark set)
    SURVEY          per-waypoint yaw + forward
    TRANSIT_HOME    fly back from landmark to takeoff (only if landmark set)
    PRE_DESCEND     hover-at-home so drift settles
    DESCEND         drop to ground
    STOP            zero-twist hold
    DONE            terminal success
    EMERGENCY_LAND  aborted -- skip to descent immediately
    FAILED          terminal failure (with reason)

Outcome surface:
    {"event": "started"|"phase"|"completed"|"aborted"|"failed",
     "phase": "<name>", "elapsed_s": <float>,
     "intent": "<original>", "landmark": "<name>"|null,
     "reason": "<failure or abort cause>"}
"""

import json
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist
from std_msgs.msg import String

from autobee_core.survey_pattern_generator import generate


# World XY of named landmarks in campus.sdf (must stay in sync with the
# SDF model poses). Z is ignored — survey altitude comes from the mission.
LANDMARKS = {
    "main_gate":          (-20.0, 0.0),
    "library":            (10.0, 15.0),
    "convocation_ground": (15.0, -15.0),
    "admin":              (-10.0, -18.0),
}


class MissionExecutorNode(Node):
    def __init__(self):
        super().__init__("mission_executor_node")

        self.declare_parameter("default_cruise_altitude_m", 8.0)
        self.declare_parameter("default_survey_radius_m", 6.0)
        self.declare_parameter("default_survey_pattern", "orbit")
        self.declare_parameter("max_xy_speed", 1.0)
        self.declare_parameter("max_z_speed", 0.6)
        self.declare_parameter("max_yaw_rate", 0.6)
        self.declare_parameter("settle_time_s", 1.0)
        self.declare_parameter("autostart_mission", "")
        # Where in the world the drone takes off (its (0,0) reference).
        # Used to compute open-loop transit legs to/from named landmarks.
        self.declare_parameter("takeoff_world_x", 0.0)
        self.declare_parameter("takeoff_world_y", 0.0)
        self.takeoff_world_xy = (
            float(self.get_parameter("takeoff_world_x").value),
            float(self.get_parameter("takeoff_world_y").value),
        )

        self.max_xy_speed = float(self.get_parameter("max_xy_speed").value)
        self.max_z_speed = float(self.get_parameter("max_z_speed").value)
        self.max_yaw_rate = float(self.get_parameter("max_yaw_rate").value)
        self.settle_time = float(self.get_parameter("settle_time_s").value)

        self.cmd_pub = self.create_publisher(Twist, "/drone/cmd_vel", 10)
        self.status_pub = self.create_publisher(String, "/mission_status", 10)
        self.create_subscription(String, "/mission_command", self._on_command, 10)
        self.create_subscription(String, "/mission_abort", self._on_abort, 10)
        # /drone/pose is OPTIONAL feedback — when SLAM is up it gives us a
        # post-mission RTH error metric (and eventually closed-loop control).
        # When SLAM is not running, _last_pose stays None and the executor
        # behaves exactly as before (open-loop).
        self.create_subscription(Pose, "/drone/pose", self._on_pose, 10)
        self.create_timer(0.05, self._tick)
        self._last_pose = None
        self._takeoff_pose = None

        self.schedule = None
        self.start_time = None
        self._mission_intent = ""
        self._last_phase = None
        self._aborted = False
        self._abort_reason = ""

        self.get_logger().info(
            "mission_executor up (open-loop, command-driven); "
            "waiting for JSON on /mission_command (abort on /mission_abort)"
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
            reason = f"unsupported action: {command.get('action')!r}"
            self.get_logger().error(reason)
            self._publish_status("failed", phase="REJECTED", reason=reason)
            return

        cruise = float(command.get(
            "altitude_m", self.get_parameter("default_cruise_altitude_m").value))
        radius = float(command.get(
            "radius_m", self.get_parameter("default_survey_radius_m").value))
        pattern = str(command.get(
            "pattern", self.get_parameter("default_survey_pattern").value))
        intent = str(command.get("intent", ""))
        landmark = command.get("landmark")
        if isinstance(landmark, str):
            landmark = landmark.strip().lower() or None
        else:
            landmark = None
        if landmark is not None and landmark not in LANDMARKS:
            self.get_logger().warn(
                f"unknown landmark {landmark!r}; falling back to local-area survey"
            )
            landmark = None

        try:
            self.cruise_altitude = cruise
            self.survey_radius = radius
            self.survey_pattern = pattern
            self.survey_landmark = landmark
            self._mission_intent = intent
            self._aborted = False
            self._abort_reason = ""
            self.schedule, total = self._build_schedule()
        except Exception as e:
            reason = f"schedule build failed: {e}"
            self.get_logger().error(reason)
            self._publish_status("failed", phase="PLANNING", reason=reason)
            self.schedule = None
            return

        self.start_time = self.get_clock().now()
        # Snapshot the pose at mission start so we can compute RTH error at
        # the end. If SLAM hasn't started publishing, this stays None and the
        # final status simply omits the rth_error_m field.
        self._takeoff_pose = self._last_pose
        self._publish_status(
            "started",
            phase="CLIMB",
            extra={
                "pattern": pattern, "radius_m": radius,
                "altitude_m": cruise, "duration_s": round(total, 1),
                "landmark": landmark,
                "pose_feedback": self._last_pose is not None,
            },
        )
        self.get_logger().info(
            f"executing: pattern={pattern} radius={radius}m altitude={cruise}m "
            f"landmark={landmark!r} duration={total:.1f}s intent={intent!r}"
        )

    def _on_abort(self, msg: String):
        if self.schedule is None:
            self.get_logger().warn("abort received with no active mission; ignoring")
            return
        if self._aborted:
            return
        self._aborted = True
        self._abort_reason = msg.data.strip() or "operator abort"
        self.get_logger().warn(f"ABORT: {self._abort_reason}; switching to EMERGENCY_LAND")

        # Replace remaining schedule with: zero-stop, descend, terminal stop.
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        new_events = [(elapsed, elapsed + 1.0, Twist(), "EMERGENCY_LAND_HOVER")]
        descend_dur = self.cruise_altitude / self.max_z_speed if hasattr(self, "cruise_altitude") else 8.0
        descend_cmd = Twist()
        descend_cmd.linear.z = -self.max_z_speed
        new_events.append((elapsed + 1.0, elapsed + 1.0 + descend_dur, descend_cmd, "EMERGENCY_LAND_DESCEND"))
        new_events.append((elapsed + 1.0 + descend_dur, float("inf"), Twist(), "FAILED"))
        self.schedule = new_events

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

        # If a landmark is requested, fly there from takeoff before surveying.
        # Survey center is in body-relative coords (drone "thinks" it is at
        # (0,0)) — translate the landmark target into the same frame.
        if self.survey_landmark and self.survey_landmark in LANDMARKS:
            lx_world, ly_world = LANDMARKS[self.survey_landmark]
            tx, ty = self.takeoff_world_xy
            survey_cx = lx_world - tx
            survey_cy = ly_world - ty
        else:
            survey_cx, survey_cy = 0.0, 0.0

        prev_x, prev_y = 0.0, 0.0
        prev_heading = 0.0

        if (survey_cx, survey_cy) != (0.0, 0.0):
            t, prev_x, prev_y, prev_heading = self._append_transit(
                events, t, prev_x, prev_y, prev_heading,
                survey_cx, survey_cy, "TRANSIT_OUT",
            )

        waypoints = generate(
            self.survey_pattern,
            (survey_cx, survey_cy),
            self.survey_radius,
            self.cruise_altitude,
        )

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

        # Return-to-takeoff transit if the survey was off-center (landmark mode).
        if (prev_x, prev_y) != (0.0, 0.0):
            t, prev_x, prev_y, prev_heading = self._append_transit(
                events, t, prev_x, prev_y, prev_heading,
                0.0, 0.0, "TRANSIT_HOME",
            )

        # Hover-at-home: open-loop accumulates drift across the spiral; give
        # the drone a few seconds with zero command before descending so any
        # residual velocity damps out instead of carrying it sideways into
        # whatever's at ground level.
        hover_home_dur = max(self.settle_time, 4.0)
        events.append((t, t + hover_home_dur, Twist(), "PRE_DESCEND"))
        t += hover_home_dur

        descend_dur = self.cruise_altitude / self.max_z_speed
        descend_cmd = Twist()
        descend_cmd.linear.z = -self.max_z_speed
        events.append((t, t + descend_dur, descend_cmd, "DESCEND"))
        t += descend_dur

        # Final stop phase: explicitly publish zeros for a couple seconds after
        # nominal touchdown so the velocity-control plugin doesn't carry the
        # last command forward (this was the "drone keeps moving after Ctrl-C"
        # bug pattern -- same fix applied here as a normal mission end).
        events.append((t, t + 2.0, Twist(), "STOP"))
        t += 2.0

        events.append((t, float("inf"), Twist(), "DONE"))

        return events, t

    def _tick(self):
        if self.schedule is None:
            return
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        for (t_start, t_end, cmd, label) in self.schedule:
            if t_start <= elapsed < t_end:
                self.cmd_pub.publish(cmd)
                # Skip logging the per-waypoint micro-phases (YAW->n, FWD->n)
                # and the transit yaw sub-phases — those are implementation
                # detail. The transit forward legs (TRANSIT_OUT, TRANSIT_HOME)
                # *do* log so operators see when the drone changed area.
                noisy = (
                    label.startswith(("YAW", "FWD"))
                    or label.endswith("_YAW")
                )
                if label != self._last_phase and not noisy:
                    self.get_logger().info(f"phase: {label} (t={elapsed:.1f}s)")
                    self._last_phase = label
                    # Phase transition -- emit lifecycle event
                    if label == "DONE":
                        self._publish_status(
                            "completed", phase="DONE", elapsed=elapsed,
                            extra=self._rth_audit(),
                        )
                    elif label == "FAILED":
                        self._publish_status(
                            "aborted", phase="FAILED", elapsed=elapsed,
                            reason=self._abort_reason or "unknown",
                            extra=self._rth_audit(),
                        )
                    else:
                        self._publish_status("phase", phase=label, elapsed=elapsed)
                return
        # Past the end of the schedule entirely: hold zero
        self.cmd_pub.publish(Twist())

    def _publish_status(self, event, phase=None, elapsed=None, reason="", extra=None):
        payload = {
            "event": event,
            "phase": phase,
            "elapsed_s": round(elapsed, 2) if elapsed is not None else None,
            "intent": self._mission_intent,
            "landmark": getattr(self, "survey_landmark", None),
            "reason": reason,
            "wall_time": time.time(),
        }
        if extra:
            payload.update(extra)
        msg = String()
        msg.data = json.dumps(payload)
        self.status_pub.publish(msg)

    def _on_pose(self, msg: Pose):
        self._last_pose = (msg.position.x, msg.position.y, msg.position.z)

    def _rth_audit(self):
        """Compute RTH error if pose feedback was available at start + end."""
        if self._takeoff_pose is None or self._last_pose is None:
            return None
        x0, y0, z0 = self._takeoff_pose
        x1, y1, z1 = self._last_pose
        err_xy = math.hypot(x1 - x0, y1 - y0)
        err_xyz = math.sqrt((x1 - x0) ** 2 + (y1 - y0) ** 2 + (z1 - z0) ** 2)
        return {
            "rth_error_xy_m": round(err_xy, 3),
            "rth_error_xyz_m": round(err_xyz, 3),
            "takeoff_pose_xyz": [round(v, 3) for v in self._takeoff_pose],
            "final_pose_xyz": [round(v, 3) for v in self._last_pose],
        }

    def _append_transit(self, events, t, x0, y0, heading0, x1, y1, label):
        """Append a yaw + forward leg from (x0,y0) to (x1,y1). Returns updated t/state."""
        dx = x1 - x0
        dy = y1 - y0
        distance = math.hypot(dx, dy)
        if distance < 1e-3:
            return t, x0, y0, heading0
        target_heading = math.atan2(dy, dx)
        heading_diff = self._wrap(target_heading - heading0)
        yaw_dur = abs(heading_diff) / self.max_yaw_rate
        if yaw_dur > 0.05:
            yaw_cmd = Twist()
            yaw_cmd.angular.z = self.max_yaw_rate * (1.0 if heading_diff > 0 else -1.0)
            events.append((t, t + yaw_dur, yaw_cmd, f"{label}_YAW"))
            t += yaw_dur
        fwd_dur = distance / self.max_xy_speed
        fwd_cmd = Twist()
        fwd_cmd.linear.x = self.max_xy_speed
        events.append((t, t + fwd_dur, fwd_cmd, label))
        t += fwd_dur
        return t, x1, y1, target_heading

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
