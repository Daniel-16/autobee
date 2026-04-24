# AutoBee

Autonomous aerial surveillance drone for Bingham University campus. CS dissertation project by Daniel Olamide Oloruntoba.

## Stack

- **ROS2 Humble** (ros-base, Dockerised on Ubuntu 22.04)
- **Gazebo Ignition Fortress** (`ros-humble-ros-gz`) — simulation
- **Nav2** — path planning / obstacle avoidance
- **ORB-SLAM3** — visual-inertial SLAM (built from source, C++14-patched)
- **YOLOv8 via `ultralytics`** — object detection (CPU inference, YOLOv8n)
- **LangChain + Google Gemini** — LLM agent for mission reasoning
- **PyTorch CPU-only** — no NVIDIA GPU available on host

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
- **Phase 2 — Simulation assets** (Gazebo campus world + drone URDF + verify camera topic @ 30 FPS)
- **Phase 3 — Perception** (YOLOv8 ROS2 node, ≥5 FPS on CPU)
- **Phase 4 — SLAM** (wrap ORB-SLAM3 as ROS2 node)
- **Phase 5 — Navigation** (Nav2 integration)
- **Phase 6 — LLM agent** (LangChain + Gemini for mission reasoning)

## What I'll write vs use off-the-shelf

Writing: Python ROS2 nodes (perception, glue), YAML configs, xacro/SDF world + drone, launch files.
Using as-is: Nav2 stack, ORB-SLAM3 library (with an existing ros2-ORB_SLAM3 wrapper), Gazebo plugins.
Effectively zero C++ authoring expected.

## Resource awareness

- CPU-only everything. YOLOv8 FPS target is **≥5** (not 10 — lifted in PRD NFR-02 because of the no-GPU constraint).
- Build with `MAKEFLAGS=-j2` (or `-j1`) when memory is tight — `-j4` hits OOM on this host. See phase1_gotchas memory.
- `/tmp`, pip cache, and apt cache inside the container are ephemeral — don't store work there.
