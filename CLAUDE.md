# AutoBee

Autonomous aerial surveillance drone for Bingham University campus. CS dissertation project by Daniel Olamide Oloruntoba.

## Stack

- **ROS2 Humble** (ros-base, Dockerised on Ubuntu 22.04)
- **Gazebo Ignition Fortress** (`ros-humble-ros-gz`) — simulation
- **Nav2** — path planning / obstacle avoidance
- **ORB-SLAM3** — visual-inertial SLAM (built from source, C++14-patched)
- **YOLOv8 via `ultralytics`** — object detection (CPU inference, YOLOv8n)
- **LangChain + Google Gemini (gemini-2.5-flash)** — LLM mission planner + report writer
- **python-dotenv** — loads `GOOGLE_API_KEY` from `/workspaces/AUTOBEE/.env`
- **PyTorch CPU-only** — no NVIDIA GPU available on host
- **Target actuator**: DJI Tello (low-altitude, indoor/outdoor, no GPS). **Closed-loop control runs on the Tello's onboard velocity/height telemetry, not SLAM** — SLAM is off the critical path (a later drift-correction/mapping upgrade). See the autonomy_redesign memory.

## Layered autonomy (closed-loop)

Three layers; the LLM is **never** in the inner control loop:

1. **Reactive (~20 Hz, deterministic)** — `flight_controller_node` (`autobee_control`). Velocity/heading control to one goal from `/drone/odom`, emits `/drone/cmd_vel`. Reactive avoidance *steers* (repulsion bias) rather than only aborting. Handshakes arrival on `/drone/goal_status` (`reached`/`timeout`/`blocked`).
2. **Executive (10 Hz)** — `mission_executor_node` (`autobee_core`). Pre-plans a goal list (geometric known-map avoidance), issues goals one at a time, advances on `reached`. On `blocked`/`timeout`/high-severity anomaly it escalates to the agent via `/mission_event` and holds until a `/mission_directive` arrives.
3. **Deliberative + tactical** — `llm_agent_node` (`autobee_agent`). `/user_intent` → `/mission_command` (plan), and `/mission_event` → `/mission_directive` (mid-mission tactical decision: continue/hold/skip/climb/divert/rth/abort). Has a rule-based fallback so a missing LLM never stalls the executive.

## Tello-portable interface

All autonomy nodes consume/produce a fixed topic graph so the simulator can be swapped for a `tello_socket` driver with no node changes:

- `/drone/cmd_vel` (`geometry_msgs/Twist`) — control input (published by `flight_controller_node`)
- `/drone/odom` (`nav_msgs/Odometry`) — **unified feedback contract** (pose + body twist). Real: `tello_odometry_node` integrates `/drone/state`. Sim: `sim_odometry_node` converts Gazebo ground-truth `/drone/pose` (+ injected Tello-like noise). The controller and executive both consume this.
- `/drone/goal` (`std_msgs/String`, JSON) — single goal, executive → controller (`{goal_id,type,x,y,z,yaw,avoid,...}`)
- `/drone/goal_status` (`std_msgs/String`, JSON) — controller → executive handshake (`reached`/`timeout`/`blocked`)
- `/mission_event` (`std_msgs/String`, JSON) — executive → agent escalation (`path_blocked`/`goal_timeout`/`anomaly`)
- `/mission_directive` (`std_msgs/String`, JSON) — agent → executive tactical decision (echoes `event_id`)
- `/drone/pose` (`geometry_msgs/Pose`) — Gazebo ground-truth (sim) / future SLAM (`orb_slam3_node`). Now feeds `sim_odometry_node`; no longer consumed directly by the executor.
- `/tts_say` (`std_msgs/String`) — ad-hoc "speak this now" channel into `tts_node`
- `/tts_speaking` (`std_msgs/String`, JSON) — TTS state events (`{"state":"started","text":"..."}`)
- `/drone/camera/image_raw` (`sensor_msgs/Image`) — camera feed
- `/drone/imu` (`sensor_msgs/Imu`) — IMU
- `/user_intent` (`std_msgs/String`) — natural-language command in
- `/mission_command` (`std_msgs/String`, JSON) — LLM-emitted structured plan
- `/perception/event` (`std_msgs/String`, JSON) — per-frame *person* detection summary (YOLO is now run unfiltered, but this channel keeps person-only semantics for backward compat)
- `/perception/obstacles` (`std_msgs/String`, JSON) — most-threatening non-person YOLO detection per frame (large + centred); now includes a **signed** `cx_offset_frac` (+right/−left) so the controller can steer laterally. Feeds the controller's reactive avoidance. Silent when no detection clears the size/centredness/score thresholds.
- `/perception/looming` (`std_msgs/String`, JSON) — Lucas-Kanade optical-flow looming alert from `looming_detector_node`. Fires when median radial expansion ratio implies time-to-contact below `ttc_threshold_s` for `debounce_frames` consecutive frames. Suppressed during YAW/CLIMB/DESCEND/SETTLE phases via `/mission_status` gating. Feeds the controller's reactive avoidance.
- `/perception/anomaly` (`std_msgs/String`) — single-frame threshold breach (legacy, raw)
- `/crowd_analytics` (`std_msgs/String`, JSON) — temporal aggregation: peak/mean/median count, trend, rate, density label
- `/anomaly_alert` (`std_msgs/String`, JSON) — typed anomalies (SURGE, DISPERSAL, SUSTAINED_HIGH, GATHERING) with severity
- `/mission_report` (`std_msgs/String`) — final natural-language summary synthesised from analytics + alerts

Operational envelope is clamped to Tello-class limits in `llm_agent_node`: radius 3..15 m, altitude 4..12 m. Default cruise altitude is 11 m so the survey clears all campus building tops (max 9 m at the hostel) plus a 1.5 m vertical safety margin.

## Obstacle avoidance (two layers)

1. **Geometric, primary.** `mission_executor_node` keeps a `KNOWN_OBSTACLES` table mirroring `campus.sdf` (8 buildings + 6 trees, each with safety radius and height). Before flying, it: (a) excludes the destination landmark from the avoidance set, (b) drops obstacles whose top is below `cruise_altitude - 1.5 m`, (c) splits the transit-out and transit-home legs into ~3 m segments and runs `survey_pattern_generator.avoid_obstacles()` on every waypoint, (d) clamps the survey radius up to `dst_safety_r + 1.5` so building-target orbits actually go *around* the building. Disabled with `avoid_known_obstacles:=false` (debug only).
2. **Perception backstop, runtime — now steers, not just aborts.** `flight_controller_node` subscribes to `/perception/looming` and `/perception/obstacles` and biases the velocity command: looming (no bearing) → brake forward + climb + back off; obstacle (signed `cx_offset_frac`) → brake + push to the opposite side + slight climb. Gated against climb/descend goals and hard turns. If a threat persists past `block_timeout_s` the controller reports `blocked`, the executive escalates to the agent, and the agent decides (typically climb-over or divert). Hard `/mission_abort` (operator) remains the kill switch. Disabled with `avoidance_enabled:=false` (wired to the `perception_backstop` launch arg on Tello).

The combination matters: geometric covers known/textureless targets the planner can foresee; looming covers unknown/dynamic obstacles the planner doesn't know about. Either alone leaves gaps.

## Architecture: host vs container

**Host** (Ubuntu 25.10, 14 GB RAM, 4-core CPU, Intel HD 520, no GPU):
- Git, VS Code, Docker daemon
- Code edits happen here (`/home/daniel-toba/Documents/AUTOBEE`)
- Cannot run ROS2 natively (Humble is 22.04-only; Jazzy needs 24.04)

**Container** `autobee:humble` (5.3 GB):
- Runs all ROS2, Gazebo, SLAM, YOLO workloads
- Workspace mounted from host: `/workspaces/AUTOBEE` (host path bind)
- Persistent container `autobee_dev` — `docker start` after reboot, don't recreate

## Key paths

- `Dockerfile` — image definition. Layered for network resilience (see phase1_gotchas memory).
- `.devcontainer/devcontainer.json` — VS Code Dev Containers config
- `docker-compose.yml` — CLI alternative with X11/network config
- `scripts/build_orbslam3.sh` — ORB-SLAM3 build (split from image build; recoverable).
  Applies a C++14 sed-patch to ORB-SLAM3's CMakeLists. Don't "clean up" this patch — it's load-bearing.
- `scripts/xhost_setup.sh` — host-side X11 grant (run once per host boot for GUI apps)
- `PRD_AutoBee_Surveillance_System.md` — authoritative project spec. Phase plan lives in Section 6.
- `/opt/autobee/third_party/` (inside container) — Pangolin + ORB-SLAM3 source builds

## Common commands

```bash
# Host side (once per reboot for GUI):
xhost +local:docker

# Enter running container:
docker exec -it autobee_dev bash

# Start stopped container:
docker start autobee_dev

# Fresh container for throwaway work:
docker run --rm -it --network=host --ipc=host \
  -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /home/daniel-toba/Documents/AUTOBEE:/workspaces/AUTOBEE \
  autobee:humble bash
```

Inside container:
```bash
source /opt/ros/humble/setup.bash       # (auto-sourced via .bashrc)
cd /root/autobee_ws && colcon build
ros2 launch <pkg> <launch_file>
```

## Phase plan (from PRD Section 6)

- **Phase 1 — Environment setup** ✅ COMPLETE (2026-04-24)
- **Phase 2 — Simulation assets** ✅ Campus world + drone, camera ~7 FPS (sw-rendered ogre2). Campus has 4 named landmark models: `main_gate`, `library`, `convocation_ground`, `admin`.
- **Phase 3 — Perception** ✅ YOLOv8n node + crowd_analytics_node (temporal stats) + anomaly_detection_node (SURGE/DISPERSAL/SUSTAINED_HIGH/GATHERING) + humanoid stand-ins. NB: YOLO recall on geometric stand-ins is ~0; analytics pipeline is verified end-to-end against the abstract event stream and re-targets to real Tello footage without changes.
- **Phase 4 — SLAM** 🟡 ORB-SLAM3 library built; `autobee_slam` provides a thin C++ shim (`orb_slam3_node`) publishing `/drone/pose`. **Off the closed-loop critical path** (decision 2026-06-08): control runs on Tello telemetry, not SLAM. SLAM is now a later drift-correction/mapping upgrade (mono-inertial for metric scale) that drops into `/drone/odom`. Build/runtime debug still pending.
- **Phase 5 — Navigation** 🟡 Nav2 deferred, but **closed-loop velocity control is implemented** (not Nav2): `flight_controller_node` velocity/heading loop + `mission_executor_node` goal-sequencing state machine over `/drone/odom`. RTH error reported in `/mission_status`. See the autonomy_redesign memory and the layered-autonomy section above.
- **Phase 6 — LLM agent** ✅ Mission planner (`llm_agent_node`) + report writer (`report_node`). Now landmark-aware (T-01..T-04 vocabulary).
- **Phase 7 — Voice I/O** 🟡 `autobee_voice` package: `whisper_stt_node` + `tts_node` (gTTS / pyttsx3 fallback). Wired into `demo.launch.py voice:=true`. **Build/runtime debug pending on user side** (audio device passthrough — see autobee_voice/README.md).

## Acceptance harness

`scripts/acceptance/`:
- `smoke_test.sh` — topic-graph + node-liveness check, run after `demo.launch.py` is up
- `run_acceptance.py` — drives PRD §11 T-01..T-09 by publishing `/user_intent` and asserting on `/mission_command` + `/mission_status` + `/mission_report`. Writes `acceptance_results.jsonl` and prints a TSR summary.

## What I'll write vs use off-the-shelf

Writing: Python ROS2 nodes (perception, glue, voice), YAML configs, xacro/SDF world + drone, launch files. **One small C++ shim** (`autobee_slam/src/orb_slam3_node.cpp`, ~140 lines) — chosen over vendoring a community wrapper because community wrappers' build configs are notoriously brittle across ORB-SLAM3 versions, and a debuggable 140-line shim is preferable to a 5000-line one we'd have to fork anyway.
Using as-is: Nav2 stack (deferred), ORB-SLAM3 library, Gazebo plugins, openai-whisper, gTTS/pyttsx3.

## Resource awareness

- CPU-only everything. YOLOv8 FPS target is **≥5** (not 10 — lifted in PRD NFR-02 because of the no-GPU constraint).
- Build with `MAKEFLAGS=-j2` (or `-j1`) when memory is tight — `-j4` hits OOM on this host. See phase1_gotchas memory.
- `/tmp`, pip cache, and apt cache inside the container are ephemeral — don't store work there.
