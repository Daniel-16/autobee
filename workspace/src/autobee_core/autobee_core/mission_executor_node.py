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

from autobee_core.survey_pattern_generator import (
    avoid_obstacles,
    generate,
    is_obstacle_relevant_at_altitude,
)


# World XY of named landmarks in campus.sdf (must stay in sync with the
# SDF model poses). Z is ignored — survey altitude comes from the mission.
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


# Hand-rolled known-map for geometric obstacle avoidance — mirrors campus.sdf.
# Format: name -> (world_x, world_y, safety_radius_m, top_height_m).
# safety_radius is approximated as (max footprint extent / 2) + drone buffer
# so a circular keep-out comfortably contains an axis-aligned bounding box.
# Convocation_ground is intentionally absent — it's a ground-level platform
# (z<0.2) and never a collision risk above 4m altitude.
KNOWN_OBSTACLES = {
    # Trees: ~1.6 m canopy + buffer
    "tree_1":       (28.0, -22.0, 3.0, 4.6),
    "tree_2":       (28.0, -18.0, 3.0, 4.6),
    "tree_3":       (28.0, -14.0, 3.0, 5.1),
    "tree_4":       (28.0, -10.0, 3.0, 4.6),
    "tree_5":       (0.0,  10.0,  3.0, 5.1),
    "tree_6":       (-5.0, -5.0,  3.0, 4.6),
    # Buildings: max(sx, sy)/2 + 1.5 m drone buffer
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
        # Hard ceiling on cruise_altitude regardless of LLM/mission_command input.
        # Outdoor: leave at default 100 (no effect). Indoor/Tello: set to e.g. 1.5.
        self.declare_parameter("cruise_altitude_max_m", 100.0)
        self.declare_parameter("default_survey_radius_m", 6.0)
        # Hard ceiling on survey_radius_m. Outdoor: leave at default 100 (no effect).
        # Indoor/Tello: set to e.g. 1.5 so the LLM's 3..15m envelope gets shrunk.
        self.declare_parameter("survey_radius_max_m", 100.0)
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
        # Geometric obstacle avoidance against KNOWN_OBSTACLES (campus.sdf).
        # Set to false to fly the raw pattern (debug only).
        self.declare_parameter("avoid_known_obstacles", True)
        # Whether named landmarks (main_gate, library, etc.) are real places
        # the drone can transit to. False on indoor Tello flights — the
        # campus.sdf coordinates are meaningless indoors and the executor
        # would otherwise try to fly to a phantom XY several metres away.
        self.declare_parameter("landmarks_enabled", True)
        # Optical-flow looming + YOLO obstacle detection backstop. Lets a
        # perception trigger publish /mission_abort even mid-pattern.
        self.declare_parameter("perception_backstop", True)
        self.takeoff_world_xy = (
            float(self.get_parameter("takeoff_world_x").value),
            float(self.get_parameter("takeoff_world_y").value),
        )
        self.avoid_known = bool(self.get_parameter("avoid_known_obstacles").value)
        self.perception_backstop = bool(self.get_parameter("perception_backstop").value)
        self.landmarks_enabled = bool(self.get_parameter("landmarks_enabled").value)

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
        # Perception backstops — publish /mission_abort if collision imminent.
        self.create_subscription(String, "/perception/looming", self._on_looming, 10)
        self.create_subscription(String, "/perception/obstacles", self._on_obstacle_detection, 10)
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
        cruise_max = float(self.get_parameter("cruise_altitude_max_m").value)
        if cruise > cruise_max:
            self.get_logger().warn(
                f"clamping cruise_altitude {cruise:.1f}m -> {cruise_max:.1f}m (cruise_altitude_max_m)"
            )
            cruise = cruise_max
        radius = float(command.get(
            "radius_m", self.get_parameter("default_survey_radius_m").value))
        radius_max = float(self.get_parameter("survey_radius_max_m").value)
        if radius > radius_max:
            self.get_logger().warn(
                f"clamping survey_radius {radius:.1f}m -> {radius_max:.1f}m (survey_radius_max_m)"
            )
            radius = radius_max
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
        if landmark is not None and not self.landmarks_enabled:
            # Indoor / Tello: the campus.sdf XY for this landmark is meaningless.
            # Degrade to a local survey at takeoff origin so the drone doesn't
            # try to fly several metres to a phantom coordinate.
            self.get_logger().warn(
                f"landmarks_enabled=false; ignoring landmark {landmark!r} "
                f"and surveying at takeoff origin"
            )
            landmark = None

        try:
            self.cruise_altitude = cruise
            self.survey_radius = radius
            self.survey_pattern = pattern
            self.survey_landmark = landmark
            # If the destination is a building obstacle, force the orbit radius
            # to clear its safety circle. Otherwise the survey would happen
            # inside the building.
            if landmark and landmark in KNOWN_OBSTACLES and self.avoid_known:
                _, _, dst_r, dst_top = KNOWN_OBSTACLES[landmark]
                if is_obstacle_relevant_at_altitude(dst_top, cruise):
                    min_orbit = dst_r + 1.5
                    if self.survey_radius < min_orbit:
                        self.get_logger().warn(
                            f"clamping survey_radius {self.survey_radius:.1f}m -> "
                            f"{min_orbit:.1f}m to orbit around {landmark!r} cleanly"
                        )
                        self.survey_radius = min_orbit
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

    def _build_yaw_only_schedule(self):
        """scan_in_place: climb -> 360 deg yaw -> descend -> land. No translation."""
        events = []
        t = 0.0

        climb_dur = self.cruise_altitude / self.max_z_speed
        climb = Twist(); climb.linear.z = self.max_z_speed
        events.append((t, t + climb_dur, climb, "CLIMB"))
        t += climb_dur

        events.append((t, t + self.settle_time, Twist(), "SETTLE"))
        t += self.settle_time

        yaw_dur = (2.0 * math.pi) / self.max_yaw_rate
        yaw = Twist(); yaw.angular.z = self.max_yaw_rate
        events.append((t, t + yaw_dur, yaw, "YAW_360"))
        t += yaw_dur

        events.append((t, t + self.settle_time, Twist(), "SETTLE_POST_YAW"))
        t += self.settle_time

        descend_dur = self.cruise_altitude / self.max_z_speed
        descend = Twist(); descend.linear.z = -self.max_z_speed
        events.append((t, t + descend_dur, descend, "DESCEND"))
        t += descend_dur

        events.append((t, t + 2.0, Twist(), "STOP"))
        t += 2.0
        events.append((t, float("inf"), Twist(), "DONE"))
        return events, t

    def _build_land_now_schedule(self):
        """return_home: when the drone is grounded this is a 1s no-op that
        completes the mission cleanly. When airborne, /mission_abort is the
        right entry point — return_home as a fresh /mission_command implies
        the drone has nothing to do."""
        events = []
        t = 0.0
        events.append((t, t + 1.0, Twist(), "AT_HOME"))
        t += 1.0
        events.append((t, float("inf"), Twist(), "DONE"))
        return events, t

    def _build_schedule(self):
        # Skill-specific schedule shapes that aren't orbit/spiral.
        if self.survey_pattern == "yaw_only":
            return self._build_yaw_only_schedule()
        if self.survey_pattern == "land_now":
            return self._build_land_now_schedule()

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

        # Build the obstacle list (in the drone-relative frame) BEFORE planning
        # the transit so both the transit leg and the survey waypoints get
        # repaired against the same set.
        obstacles_relative = self._collect_relevant_obstacles()

        if (survey_cx, survey_cy) != (0.0, 0.0):
            # Split the transit into ~3m segments, repair each waypoint
            # against the obstacle list, then chain them as transit legs.
            # A single straight line from takeoff to landmark would otherwise
            # fly through any tree on the path.
            transit_waypoints = self._segment_line(
                (prev_x, prev_y), (survey_cx, survey_cy),
                segment_m=3.0,
                z=self.cruise_altitude,
            )
            if obstacles_relative and transit_waypoints:
                transit_waypoints, repairs = avoid_obstacles(transit_waypoints, obstacles_relative)
                if repairs:
                    self.get_logger().info(
                        f"transit avoidance: pushed {repairs}/{len(transit_waypoints)} "
                        f"transit waypoints out of obstacles"
                    )
            for i, (tx_, ty_, _) in enumerate(transit_waypoints):
                label = "TRANSIT_OUT" if i == len(transit_waypoints) - 1 else f"TRANSIT_OUT_{i}"
                t, prev_x, prev_y, prev_heading = self._append_transit(
                    events, t, prev_x, prev_y, prev_heading,
                    tx_, ty_, label,
                )
            # Final survey center lines up with the last transit waypoint
            # (which may have been pushed out by avoidance).
            survey_cx, survey_cy = prev_x, prev_y

        waypoints = generate(
            self.survey_pattern,
            (survey_cx, survey_cy),
            self.survey_radius,
            self.cruise_altitude,
        )

        if obstacles_relative:
            waypoints, repairs = avoid_obstacles(waypoints, obstacles_relative)
            if repairs:
                self.get_logger().info(
                    f"obstacle avoidance: pushed {repairs}/{len(waypoints)} waypoints "
                    f"out of {len(obstacles_relative)} known obstacles"
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
            home_segments = self._segment_line(
                (prev_x, prev_y), (0.0, 0.0),
                segment_m=3.0,
                z=self.cruise_altitude,
            )
            if obstacles_relative and home_segments:
                home_segments, repairs = avoid_obstacles(home_segments, obstacles_relative)
                if repairs:
                    self.get_logger().info(
                        f"return avoidance: pushed {repairs}/{len(home_segments)} "
                        f"home-transit waypoints out of obstacles"
                    )
            for i, (tx_, ty_, _) in enumerate(home_segments):
                label = "TRANSIT_HOME" if i == len(home_segments) - 1 else f"TRANSIT_HOME_{i}"
                t, prev_x, prev_y, prev_heading = self._append_transit(
                    events, t, prev_x, prev_y, prev_heading,
                    tx_, ty_, label,
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
                # Skip logging the per-waypoint micro-phases (YAW->n, FWD->n),
                # the transit yaw sub-phases, and intermediate transit waypoints
                # (TRANSIT_OUT_0, TRANSIT_HOME_2 etc). Only the final TRANSIT_OUT
                # / TRANSIT_HOME labels (no trailing index) log so operators see
                # when the drone changed area.
                # Per-waypoint micro-phases ("YAW->3", "FWD->5", "TRANSIT_OUT_3_YAW",
                # "TRANSIT_OUT_2") are noise. Skill-level phases that share a prefix
                # ("YAW_360", "SETTLE_POST_YAW", "AT_HOME") are not — keep them visible.
                _SKILL_LEVEL_PHASES = {"YAW_360", "SETTLE_POST_YAW", "AT_HOME"}
                noisy = label not in _SKILL_LEVEL_PHASES and (
                    label.startswith(("YAW->", "FWD->"))
                    or label.endswith("_YAW")
                    or label.rsplit("_", 1)[-1].isdigit()
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
                        # Mission is finished — drop the schedule so the next
                        # /mission_command starts from a clean state instead of
                        # racing against a still-active DONE-loop.
                        self.schedule = None
                        self._last_phase = None
                    elif label == "FAILED":
                        self._publish_status(
                            "aborted", phase="FAILED", elapsed=elapsed,
                            reason=self._abort_reason or "unknown",
                            extra=self._rth_audit(),
                        )
                        self.schedule = None
                        self._last_phase = None
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

    def _on_looming(self, msg: String):
        """Optical-flow looming alert -> emergency-land.

        The detector itself is gated against YAW/CLIMB/DESCEND phases (it
        suppresses publish during those). By the time a message reaches us,
        we trust it and route through the existing abort path.
        """
        if not self.perception_backstop or self.schedule is None or self._aborted:
            return
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        ttc = payload.get("ttc_s")
        reason = f"looming detected (ttc={ttc}s) — emergency land"
        abort_msg = String()
        abort_msg.data = reason
        # Re-use the standard abort path so the failure mode is identical to
        # an operator abort (EMERGENCY_LAND_HOVER -> DESCEND -> FAILED).
        self._on_abort(abort_msg)

    def _on_obstacle_detection(self, msg: String):
        """Large-and-centered YOLO non-person detection -> emergency-land."""
        if not self.perception_backstop or self.schedule is None or self._aborted:
            return
        # Suppress during yaw/climb/descend — these phases generate big
        # detections sliding across frame that aren't actually approaching.
        suppressing_phases = {
            "CLIMB", "SETTLE", "PRE_DESCEND", "DESCEND", "STOP",
            "YAW_360", "SETTLE_POST_YAW", "AT_HOME",
        }
        if self._last_phase in suppressing_phases or (
            self._last_phase and self._last_phase.endswith("_YAW")
        ):
            return
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        cls = payload.get("class")
        score = payload.get("score")
        reason = f"obstacle detected (class={cls!r} score={score}) — emergency land"
        abort_msg = String()
        abort_msg.data = reason
        self._on_abort(abort_msg)

    def _collect_relevant_obstacles(self):
        """Return KNOWN_OBSTACLES filtered for this mission, in drone-relative XY.

        Excludes the destination landmark (we want to orbit around it, not
        avoid it). Excludes obstacles whose top is well below cruise altitude
        (no collision risk). Translates each obstacle into the drone-relative
        frame so distances match the planner's waypoint coords.
        """
        if not self.avoid_known:
            return []
        tx, ty = self.takeoff_world_xy
        relevant = []
        for name, (ox, oy, r, top) in KNOWN_OBSTACLES.items():
            if name == self.survey_landmark:
                continue
            if not is_obstacle_relevant_at_altitude(top, self.cruise_altitude):
                continue
            relevant.append((ox - tx, oy - ty, r))
        return relevant

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

    @staticmethod
    def _segment_line(start_xy, end_xy, segment_m, z):
        """Split (start->end) into a list of waypoints ~segment_m apart, ending at end_xy.

        Returns a list of (x, y, z). At least one waypoint (the endpoint) is
        always returned so the caller can iterate without an empty-check.
        """
        x0, y0 = start_xy
        x1, y1 = end_xy
        dx, dy = x1 - x0, y1 - y0
        distance = math.hypot(dx, dy)
        if distance <= segment_m + 1e-6:
            return [(x1, y1, z)]
        n = max(1, int(math.ceil(distance / segment_m)))
        return [
            (x0 + dx * (i / n), y0 + dy * (i / n), z)
            for i in range(1, n + 1)
        ]

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
