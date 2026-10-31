# AutoBee

[![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.22085150.svg)](https://doi.org/10.5281/zenodo.22085150)
[![License: Source-Available Proprietary](https://img.shields.io/badge/license-source--available%20proprietary-blue)](LICENSE)

AutoBee lets a non-pilot issue a natural-language command — *"check if the
field is overcrowded"* — and have a drone autonomously plan a survey, fly it
with closed-loop velocity control, avoid obstacles using its camera, detect and
count crowds, and speak/write back a report. The simulator (Gazebo) and the real
DJI Tello share an identical ROS 2 topic graph, so the same autonomy code runs
on both.

> **Which version does the report describe?** The technical report
> ([doi:10.5281/zenodo.22085150](https://doi.org/10.5281/zenodo.22085150))
> evaluates an earlier version of this code, tagged
> [`report-v0.0.1`](https://github.com/Daniel-16/autobee/tree/report-v0.0.1). In that version the mission
> executor ran open loop on a timed velocity schedule, the planner required
> Gemini, and the LLM made no mid-mission decisions. On 8 June 2026 the executor
> was rewritten as the closed-loop goal sequencer described in this README, and
> the agent gained a tactical layer and an offline rule-based planner. The
> current version has **not** yet been through the acceptance evaluation, so the
> report's numbers (25 of 34 runs passed, 73.5%) apply to the tagged version
> only.

> **Heads-up for the person running this on a Mac:** jump straight to
> [Running on macOS](#running-on-macos). The rest of this README explains the
> system; that section is the step-by-step.

---

## Table of contents

- [What it does](#what-it-does)
- [The validation gate](#the-validation-gate)
- [Architecture at a glance](#architecture-at-a-glance)
- [Tech stack](#tech-stack)
- [Repository layout](#repository-layout)
- [Running on macOS](#running-on-macos) ← **start here to run it**
- [Running on Linux](#running-on-linux)
- [Web dashboard](#web-dashboard)
- [Driving a mission](#driving-a-mission)
- [Optional features (voice, SLAM)](#optional-features)
- [Real DJI Tello (live drone + web dashboard)](#real-dji-tello-live-drone--web-dashboard)
- [Troubleshooting](#troubleshooting)
- [Project status](#project-status)
- [Citation](#citation)
- [License](#license)

---

## What it does

1. You give it an intent in plain English (typed, or spoken if voice is on).
2. An **LLM agent** (Google Gemini) picks one skill from a closed catalogue of
   eight and proposes its parameters (landmark, altitude, survey radius). With
   no internet, a deterministic keyword parser does the same job. Either way,
   the [validation gate](#the-validation-gate) checks the result before
   anything is published to the executor.
3. A **mission executor** breaks the plan into a sequence of geometric goals,
   pre-routing around known buildings/trees.
4. A **flight controller** flies to each goal with closed-loop velocity control,
   steering around unexpected obstacles seen by the camera.
5. **YOLOv8 perception** counts people per frame; analytics aggregate crowd
   trends and raise typed anomalies (surge, dispersal, sustained-high,
   gathering).
6. When something blocks the drone mid-mission, the executor escalates to the
   agent, which picks one of seven fixed actions (continue, hold, skip goal,
   climb, divert, return home, abort). Any other answer is discarded and a
   rule-based directive is used instead.
7. At the end, a **report writer** synthesises a natural-language summary.

The LLM is **never** in the inner control loop — it plans and makes high-level
tactical calls; deterministic code flies the drone.

## The validation gate

The gate is ordinary code in `llm_agent_node` that sits between the planner and
the executor. The model never emits coordinates or velocities. The gate applies
three rules before a command is published on `/mission_command`:

1. **Catalogue check.** A skill outside the eight in `SKILL_DEFAULTS` is
   discarded. In the report version this rejected the command outright; in the
   current version the command is re-planned by the rule-based parser.
2. **Landmark fallback.** A skill that needs a landmark but arrives without a
   known one becomes a local headcount survey (`count_people`) at the launch
   position.
3. **Envelope clamp.** Radius is clamped to 3 to 15 m and altitude to 4 to
   12 m. On the real Tello, `demo_tello.launch.py` then caps the executor
   further for indoor flight (see [Pre-flight](#pre-flight)).

`abort` bypasses the gate and the mission queue and goes straight to
`/mission_abort`. Because the gate is plain code, it can be tested with
hand-written inputs and no model call.

Known gap: in the current version, mid-mission directives are checked against
the list of seven actions, but their numeric parameters (`delta_alt_m`,
`hold_s`) are not yet bounded.

## Architecture at a glance

Three layers of autonomy (see `docs/figures/mission_phases.png`):

| Layer | Node | Rate | Responsibility |
|-------|------|------|----------------|
| **Reactive** | `flight_controller_node` | ~20 Hz | Velocity/heading control to one goal; reactive obstacle steering. |
| **Executive** | `mission_executor_node` | 10 Hz | Sequences goals, pre-routes around the known map, escalates problems. |
| **Deliberative + tactical** | `llm_agent_node` | event-driven | Natural language → plan; mid-mission tactical decisions. |

All layers talk over a fixed ROS 2 topic graph so the Gazebo simulator can be
swapped for the real Tello with **no code changes**. The key contract topic is
`/drone/odom` (`nav_msgs/Odometry`) — produced by `sim_odometry_node` in sim and
`tello_odometry_node` on the real drone. Full topic list is in
[`CLAUDE.md`](CLAUDE.md).

## Tech stack

- **ROS 2 Humble** (Dockerised on Ubuntu 22.04) — everything runs in a container
- **Gazebo Ignition Fortress** (`ros-gz`) — simulation
- **YOLOv8** (`ultralytics`, CPU inference) — person/obstacle detection
- **LangChain + Google Gemini** (`gemini-2.5-flash`) — mission planner + reporter,
  with a deterministic keyword parser as the offline fallback
- **ORB-SLAM3** — monocular SLAM wrapper (optional, off the critical path)
- **openai-whisper + gTTS/pyttsx3** — optional voice I/O
- **PyTorch CPU-only** — no GPU required

> **Why Docker?** ROS 2 Humble only supports Ubuntu 22.04. Running it in a
> container is the supported way to use it on macOS or any other host. You do
> **not** install ROS on your Mac — it lives inside the container.

## Repository layout

```
AUTOBEE/
├── Dockerfile                 # The autobee:humble image (ROS2 + all deps)
├── docker-compose.yml         # One-command container with X11/audio wired
├── .devcontainer/             # VS Code "Dev Containers" config (alternative)
├── .env.example               # Copy to .env, add your Gemini API key
├── CLAUDE.md                  # Authoritative architecture + topic graph notes
├── PRD_AutoBee_Surveillance_System.md   # Full project spec
├── scripts/
│   ├── build_orbslam3.sh      # Builds ORB-SLAM3 (optional, slow)
│   ├── xhost_setup.sh         # Host-side X11 grant (Linux)
│   └── acceptance/            # smoke_test.sh + run_acceptance.py
├── docs/figures/              # Dissertation diagrams
├── dashboard/                 # Next.js + Tailwind web control panel (host-run)
└── workspace/src/             # The ROS 2 packages (the actual code)
    ├── autobee_agent/         # LLM planner + report writer
    ├── autobee_control/       # flight_controller + odometry adapters
    ├── autobee_core/          # mission_executor + launch files + logger
    ├── autobee_perception/    # YOLO + crowd analytics + anomaly + looming
    ├── autobee_sim/           # Gazebo campus world + drone model + bridge
    ├── autobee_slam/          # ORB-SLAM3 C++ shim
    ├── autobee_tello/         # Real DJI Tello driver
    ├── autobee_ui/            # Web dashboard bridge (ROS <-> HTTP/SSE API)
    └── autobee_voice/         # Whisper STT + TTS
```

---

## Running on macOS

This is the full walkthrough for a Mac (Apple Silicon **or** Intel). Everything
heavy runs inside a Docker container, so your Mac only needs Docker, a text
editor, and — if you want the Gazebo 3-D window — an X server.

### 0. What you need

| Requirement | Notes |
|-------------|-------|
| **Docker Desktop** | [docker.com/products/docker-desktop](https://www.docker.com/products/docker-desktop). Works on Apple Silicon and Intel. |
| **~10 GB free disk** | The image is ~5 GB; the build downloads more. |
| **≥ 8 GB RAM given to Docker** | Docker Desktop → Settings → **Resources** → set Memory to **8 GB** (the default 2 GB will OOM during `colcon build`). |
| **A Google Gemini API key** | Free at [aistudio.google.com/apikey](https://aistudio.google.com/apikey). The agent has a rule-based fallback, but the LLM features need this. |
| **XQuartz** *(optional)* | Only if you want the Gazebo/RViz GUI window. [xquartz.org](https://www.xquartz.org). Missions run fine **without** it (headless). |

> **Apple Silicon (M1/M2/M3/M4):** the image builds natively for `arm64` from a
> multi-arch ROS base — no slow x86 emulation. Intel Macs build `amd64`
> natively. Either way, just run the build below; Docker picks the right arch.

### 1. Clone and configure

```bash
git clone <your-github-url>/AUTOBEE.git
cd AUTOBEE

# Create your .env from the template and paste in your Gemini key.
cp .env.example .env
# then edit .env and set:  GOOGLE_API_KEY=AIza...your-key...
```

`.env` is gitignored — your key never gets committed.

### 2. Build the Docker image

```bash
docker compose build
```

This takes 15–40 min the first time (it installs ROS 2, PyTorch CPU, YOLO,
LangChain, etc.). It's cached afterward. ☕

> **Note:** `docker-compose.yml` is written for a Linux host (it uses
> `network_mode: host`, `/dev/snd`, and `NVIDIA_*` vars that don't apply on a
> Mac). The build works fine regardless; for **running** on a Mac, use the
> `docker run` command in step 3 below rather than `docker compose run`, because
> host networking and `/dev/snd` aren't available on Docker Desktop for Mac.

### 3. Start the container

**Headless (recommended on Mac — no GUI needed):**

```bash
docker run --rm -it \
  -p 8080:8080 \
  -v "$PWD/workspace":/root/autobee_ws \
  -v "$PWD":/workspaces/AUTOBEE \
  --name autobee_dev \
  autobee:humble bash
```

**With the Gazebo GUI (needs XQuartz):**

```bash
# One-time XQuartz setup: open XQuartz → Preferences → Security →
# tick "Allow connections from network clients", then restart XQuartz.
# Each session, allow your local machine to connect:
xhost + 127.0.0.1

docker run --rm -it \
  -e DISPLAY=host.docker.internal:0 \
  -p 8080:8080 \
  -v "$PWD/workspace":/root/autobee_ws \
  -v "$PWD":/workspaces/AUTOBEE \
  --name autobee_dev \
  autobee:humble bash
```

> `-p 8080:8080` exposes the dashboard bridge so the web UI (running on your Mac)
> can reach it. See [Web dashboard](#web-dashboard).

You're now at a shell **inside** the container. ROS 2 is auto-sourced.

### 4. Build the ROS workspace (first time, inside the container)

```bash
cd /root/autobee_ws
MAKEFLAGS=-j2 colcon build      # -j2 keeps memory in check
source install/setup.bash
```

> Re-run `colcon build` only after you change source. New shells inside the
> container need `source /root/autobee_ws/install/setup.bash` (the image's
> `.bashrc` does this automatically once `install/` exists).

### 5. Launch the simulation

```bash
ros2 launch autobee_core demo.launch.py
```

This brings up the Gazebo campus world, the drone, YOLO perception, the LLM
agent, the flight controller, and the mission executor.

- **With XQuartz** you'll see the 3-D Gazebo window.
- **Headless** (no `DISPLAY`), Gazebo runs without a window — the camera,
  physics, and the whole autonomy pipeline still work. This is the smoother path
  on Mac; GUI rendering over X11 on macOS is slow because there's no GPU
  passthrough.

### 6. Send it a command

In a **second terminal**, attach to the same container and publish an intent:

```bash
docker exec -it autobee_dev bash
source /root/autobee_ws/install/setup.bash

ros2 topic pub --times 3 --rate 1 /user_intent std_msgs/msg/String \
  '{data: "check if the convocation ground is overcrowded"}'
```

Watch the first terminal: the agent plans, the executor sequences goals, the
controller flies, and a `/mission_report` is produced at the end. See
[Driving a mission](#driving-a-mission) for more example commands and how to
watch progress.

That's the whole loop. 🎉

> Prefer a point-and-click UI over the terminal? See [Web dashboard](#web-dashboard)
> — type commands and watch the mission live in your browser.

---

## Running on Linux

On a Linux host with Docker you can use the provided compose file, which already
wires X11, audio, and host networking:

```bash
bash scripts/xhost_setup.sh        # grant X11 to local containers (once per boot)
docker compose build               # first time only
docker compose run --rm autobee    # drops you into the container shell
```

Then inside the container, the same steps 4–6 as above
(`colcon build` → `ros2 launch ...` → publish `/user_intent`).

To start/stop the persistent container later:

```bash
docker start autobee_dev
docker exec -it autobee_dev bash
```

---

## Web dashboard

A browser-based control panel (Next.js + Tailwind) lets a non-pilot type a
command and watch the mission live — status/phase, telemetry, crowd analytics,
anomaly alerts, the LLM's plan, the annotated camera feed, and the final report.

**Architecture:** the dashboard is a front-end only. A ROS node
(`autobee_ui/web_ui_node`) bridges the topic graph to a small HTTP/SSE API; the
Next.js app consumes it. Because Node.js isn't in the ROS container, you run the
dashboard on your **host** while the bridge runs in the container.

```
 Browser ──HTTP/SSE──► Next.js app (host :3000) ──► ROS bridge (container :8080) ──► AutoBee
```

**1. Bring up the stack with the bridge port exposed.** The bridge node starts
automatically as part of `demo.launch.py`; just make sure the container was
started with `-p 8080:8080` (the macOS `docker run` commands above already do).

**2. Run the dashboard on your host** (needs [Node.js](https://nodejs.org) 18.18+):

```bash
cd dashboard
npm install          # first time only
npm run dev
```

Open **http://localhost:3000**. The connection dot turns green when it reaches
the bridge. Full details (config, production build, API) are in
[`dashboard/README.md`](dashboard/README.md).

> The terminal workflow (`ros2 topic pub /user_intent ...`) still works too —
> the dashboard just publishes to the same `/user_intent` topic.

This section uses the **simulator**. To drive a **real Tello** from the same
dashboard, see [Real DJI Tello (live drone + web dashboard)](#real-dji-tello-live-drone--web-dashboard).

---

## Driving a mission

Send any of these to `/user_intent` (campus landmarks are
`main_gate`, `library`, `convocation_ground`, `admin`):

```bash
ros2 topic pub --times 3 --rate 1 /user_intent std_msgs/msg/String \
  '{data: "do a quick check of the area"}'

ros2 topic pub --times 3 --rate 1 /user_intent std_msgs/msg/String \
  '{data: "survey the library and tell me how busy it is"}'
```

Useful topics to watch while it flies (each in its own `docker exec` shell):

```bash
ros2 topic echo /mission_status      # phase transitions, RTH error
ros2 topic echo /mission_command     # the structured plan the LLM emitted
ros2 topic echo /crowd_analytics     # live crowd counts / trend
ros2 topic echo /mission_report      # final natural-language summary
ros2 topic echo /drone/goal_status   # per-goal reached/timeout/blocked
```

### Acceptance / smoke tests

With `demo.launch.py` running, in another shell:

```bash
bash scripts/acceptance/smoke_test.sh          # topic-graph + node liveness
python3 scripts/acceptance/run_acceptance.py   # PRD §11 scenarios T-01..T-09
```

---

## Optional features

### Voice I/O
```bash
ros2 launch autobee_core demo.launch.py voice:=true
```
Adds Whisper speech-to-text and gTTS/pyttsx3 text-to-speech. **Requires audio
device passthrough**, which is not available on Docker Desktop for Mac — voice
is a Linux-host feature. On a Mac, use the text `/user_intent` channel instead.

### SLAM
```bash
ros2 launch autobee_core demo.launch.py slam:=true
```
Brings up the ORB-SLAM3 wrapper. The library must be built first (inside the
container, ~30–60 min):
```bash
bash /workspaces/AUTOBEE/scripts/build_orbslam3.sh
```
SLAM is **off the closed-loop critical path** — control runs on telemetry, not
SLAM. It's an optional drift-correction/mapping upgrade.

---

## Real DJI Tello (live drone + web dashboard)

The autonomy code is Tello-portable: `demo_tello.launch.py` swaps Gazebo for the
real driver and brings up the **same** web bridge, so you fly the real drone
from the dashboard exactly like the sim. Full procedure:

### Pre-flight
- Props on, **battery > 25%** (the driver refuses to arm below that), a clear
  indoor space, flat floor. The Tello launch uses a conservative envelope
  (≤ 1.8 m altitude, ≤ 1.5 m survey radius, 0.3 m/s speed cap), disables
  landmark transits and the campus obstacle table, and does local survey/hover
  patterns.

### 1. Enable host networking (Mac only — the drone link needs it)
The Tello is controlled over its own Wi-Fi via UDP at `192.168.10.1`, so the
container must share the host's network. On **Docker Desktop for Mac 4.29+**:
**Settings → Resources → Network → tick "Enable host networking"**, then Apply &
Restart. (On Linux this is native — nothing to enable.)

> If you can't enable host networking, the real drone can't be reached from the
> container on a Mac — fall back to a **Linux host** for live flight. The sim +
> dashboard above don't need this.

### 2. Connect to the Tello's Wi-Fi (host)
Power on the Tello, then on your Mac join its Wi-Fi `TELLO-XXXXXX` (no password).

> ⚠️ On the Tello's Wi-Fi your machine has **no internet**, so Gemini is
> unreachable and the agent uses its built-in rule-based planner — commands
> still work (the dashboard's default commands are all handled offline).

### 3. Start the container with host networking
```bash
docker run --rm -it \
  --network=host \
  -v "$PWD/workspace":/root/autobee_ws \
  -v "$PWD":/workspaces/AUTOBEE \
  --name autobee_dev \
  autobee:humble bash
```
(With `--network=host` you don't add `-p` — the container shares the host's
ports, so the bridge is already at `localhost:8080`.) Verify the drone is
reachable from inside the container:
```bash
ping -c 3 192.168.10.1
```

### 4. Build & launch the real-drone stack (in the container)
```bash
cd /root/autobee_ws
colcon build --packages-skip autobee_slam
source install/setup.bash
ros2 launch autobee_core demo_tello.launch.py
```
Watch for these lines:
```
Tello connected. battery=NN%
AutoBee dashboard live at http://localhost:8080
```

### 5. Run the dashboard and fly (on your Mac)
```bash
cd dashboard
npm install        # first time only
npm run dev
```
Open **http://localhost:3000**. The **Camera (live)** card shows the real Tello
feed. Type a command (or click a chip) → the drone **auto-takes-off on the first
movement command**, flies the mission, and lands on completion. The
**EMERGENCY LAND** button (top-right) stops and lands it any time.

> **First flight:** send `scan in place` first (a yaw-only sweep) to confirm it
> rotates and holds position before trying commands that translate.

---

## Troubleshooting

| Symptom | Fix |
|---------|-----|
| `colcon build` killed / OOM | Give Docker ≥ 8 GB RAM (Docker Desktop → Resources). Build with `MAKEFLAGS=-j2` or `-j1`. |
| Gazebo window never appears (Mac) | XQuartz must allow network clients (Preferences → Security) and you must run `xhost + 127.0.0.1` on the host; container needs `-e DISPLAY=host.docker.internal:0`. Or just run **headless** — missions don't need the GUI. |
| `ros2: command not found` in a new shell | `source /opt/ros/humble/setup.bash && source /root/autobee_ws/install/setup.bash`. |
| LLM does nothing / plan looks rule-based | `GOOGLE_API_KEY` missing or wrong in `.env`. Confirm `.env` is at the repo root (mounted to `/workspaces/AUTOBEE/.env`). |
| Image nodes crash / segfault | The container pins `numpy<2` on purpose — don't upgrade numpy, it breaks `cv_bridge`. |
| Slow camera FPS in sim | Expected — CPU software rendering. Target is ≥ 5 FPS; perception tolerates it. |
| Voice does nothing on Mac | Audio passthrough isn't supported on Docker Desktop for Mac; use the text `/user_intent` channel. |

---

## Project status

| Phase | Status |
|-------|--------|
| 1 — Environment (Docker/ROS 2) | ✅ Complete |
| 2 — Simulation assets (campus world + drone) | ✅ Complete |
| 3 — Perception (YOLO + analytics + anomalies) | ✅ Complete |
| 4 — SLAM (ORB-SLAM3) | 🟡 Built; off critical path, runtime debug pending |
| 5 — Navigation (closed-loop velocity control) | ✅ Implemented 8 June 2026 (Nav2 deferred); not yet evaluated |
| 6 — LLM agent (planner + reporter + tactical) | ✅ Complete |
| 7 — Voice I/O | 🟡 Wired; audio passthrough debug pending |

**Evaluation.** The version tagged `report-v0.0.1` (open-loop executor) was
run through the nine acceptance scenarios 34 times: 25 runs in Gazebo and 9 on
a real DJI Tello. 25 passed, a task success rate of 73.5% (Wilson 95% interval
56.9% to 85.4%), below the 80% target. Full results and limitations are in the
[report](https://doi.org/10.5281/zenodo.22085150).

The current closed-loop version runs in Gazebo and on the real Tello through
`demo_tello.launch.py`, but has not yet been re-run through the acceptance
harness. See [`CLAUDE.md`](CLAUDE.md) and the PRD for detail.

---

## Citation

AutoBee is described in:

> Oloruntoba, D. O. (2026). *AutoBee: An Explicit Validation Gate Between a
> Language-Model Planner and a Low-Cost Aerial Robot.* Bingham University.
> Zenodo. https://doi.org/10.5281/zenodo.22085150

If you use this software, or report results obtained from it, please cite that
report. GitHub renders a ready-made citation from [`CITATION.cff`](CITATION.cff)
via the **Cite this repository** button, and BibTeX is below.

```bibtex
@techreport{oloruntoba2026autobee,
  title       = {AutoBee: An Explicit Validation Gate Between a Language-Model
                 Planner and a Low-Cost Aerial Robot},
  author      = {Oloruntoba, Daniel Olamide},
  institution = {Bingham University},
  year        = {2026},
  month       = {10},
  doi         = {10.5281/zenodo.22085150},
  url         = {https://doi.org/10.5281/zenodo.22085150}
}
```

The report itself is published under CC BY 4.0. The source code in this
repository is not — see below.

## License

Source-available, but **not open source**. See [`LICENSE`](LICENSE) for the
full terms. In short:

- **You may** read, clone, build, run, and privately modify AutoBee for private
  study, non-commercial academic research, teaching, peer review, and
  reproducing or verifying the published results, with attribution.
- **You may not** use it commercially or operationally, redistribute it,
  publish modified versions, relicense it, or use it to train AI models.

Copyright (c) 2026 Daniel Olamide Oloruntoba. All rights reserved. For
commercial or operational use, contact danieloloruntoba681@gmail.com.

Third-party dependencies (ROS 2, Gazebo, ORB-SLAM3, Ultralytics YOLO, PyTorch,
LangChain, and others) remain under their own licenses, some of them copyleft.

---

*CS dissertation, Bingham University. Built CPU-only, no GPU required.*
