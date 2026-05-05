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
- **Target actuator**: DJI Tello (low-altitude, indoor/outdoor, no GPS — SLAM is mandatory for closed loop)

## Tello-portable interface

All autonomy nodes consume/produce a fixed topic graph so the simulator can be swapped for a `tello_socket` driver with no node changes:

- `/drone/cmd_vel` (`geometry_msgs/Twist`) — control input
- `/drone/pose` (`geometry_msgs/Pose`) — feedback from `autobee_slam`'s `orb_slam3_node` (Tello + sim). **Mission executor still flies open-loop on a timed schedule by default; `/drone/pose` is consumed for a post-mission RTH audit (`rth_error_xy_m` in `/mission_status`). Closed-loop control is the next step.**
- `/tts_say` (`std_msgs/String`) — ad-hoc "speak this now" channel into `tts_node`
- `/tts_speaking` (`std_msgs/String`, JSON) — TTS state events (`{"state":"started","text":"..."}`)
- `/drone/camera/image_raw` (`sensor_msgs/Image`) — camera feed
- `/drone/imu` (`sensor_msgs/Imu`) — IMU
- `/user_intent` (`std_msgs/String`) — natural-language command in
- `/mission_command` (`std_msgs/String`, JSON) — LLM-emitted structured plan
- `/perception/event` (`std_msgs/String`, JSON) — per-frame *person* detection summary (YOLO is now run unfiltered, but this channel keeps person-only semantics for backward compat)
- `/perception/obstacles` (`std_msgs/String`, JSON) — most-threatening non-person YOLO detection per frame (large + centred); feeds the executor's collision backstop. Silent when no detection clears the size/centredness/score thresholds.
- `/perception/looming` (`std_msgs/String`, JSON) — Lucas-Kanade optical-flow looming alert from `looming_detector_node`. Fires when median radial expansion ratio implies time-to-contact below `ttc_threshold_s` for `debounce_frames` consecutive frames. Suppressed during YAW/CLIMB/DESCEND/SETTLE phases via `/mission_status` gating.
- `/perception/anomaly` (`std_msgs/String`) — single-frame threshold breach (legacy, raw)
- `/crowd_analytics` (`std_msgs/String`, JSON) — temporal aggregation: peak/mean/median count, trend, rate, density label
- `/anomaly_alert` (`std_msgs/String`, JSON) — typed anomalies (SURGE, DISPERSAL, SUSTAINED_HIGH, GATHERING) with severity
- `/mission_report` (`std_msgs/String`) — final natural-language summary synthesised from analytics + alerts

Operational envelope is clamped to Tello-class limits in `llm_agent_node`: radius 3..15 m, altitude 4..12 m. Default cruise altitude is 11 m so the survey clears all campus building tops (max 9 m at the hostel) plus a 1.5 m vertical safety margin.

## Obstacle avoidance (two layers)

1. **Geometric, primary.** `mission_executor_node` keeps a `KNOWN_OBSTACLES` table mirroring `campus.sdf` (8 buildings + 6 trees, each with safety radius and height). Before flying, it: (a) excludes the destination landmark from the avoidance set, (b) drops obstacles whose top is below `cruise_altitude - 1.5 m`, (c) splits the transit-out and transit-home legs into ~3 m segments and runs `survey_pattern_generator.avoid_obstacles()` on every waypoint, (d) clamps the survey radius up to `dst_safety_r + 1.5` so building-target orbits actually go *around* the building. Disabled with `avoid_known_obstacles:=false` (debug only).
2. **Perception backstop, runtime.** Two channels can publish `/mission_abort`-equivalent emergency-land triggers: `looming_detector_node` (Lucas-Kanade optical flow on the central image region; expansion ratio → time-to-contact) and the un-filtered YOLO obstacle channel. Both are gated against YAW/CLIMB/DESCEND/SETTLE phases to suppress yaw-induced false positives. Disabled with `perception_backstop:=false`.

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
- **Phase 4 — SLAM** 🟡 ORB-SLAM3 library built; `autobee_slam` package now provides a thin C++ shim (`orb_slam3_node`) that publishes `/drone/pose`. **Build/runtime debug pending on user side** (see autobee_slam/README.md). Mono-only for now; mono-inertial is the upgrade.
- **Phase 5 — Navigation** ⏸ Nav2 integration deferred. Executor remains open-loop timed but now subscribes to `/drone/pose` and emits a `rth_error_xy_m` audit field on completion when SLAM is up.
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
