# Product Requirements Document (PRD)
## AutoBee: Autonomous Natural-Language Driven Aerial Surveillance System
### Crowd Analytics & Anomaly Detection using ROS2 and Computer Vision

---

**Author:** Daniel Olamide Oloruntoba  
**Matric No:** BHU/22/04/05/0054  
**Institution:** Bingham University Karu, Nasarawa State, Nigeria  
**Department:** Computer Science, Faculty of Computing  
**Document Version:** 1.0  
**Date:** January 2026  
**Status:** Active — Simulation Phase

---

## Table of Contents

1. [Project Overview](#1-project-overview)
2. [Goals & Non-Goals](#2-goals--non-goals)
3. [Critical Constraint: Parrot Bebop 2 on ROS2](#3-critical-constraint-parrot-bebop-2-on-ros2)
4. [System Architecture Summary](#4-system-architecture-summary)
5. [Development Phases](#5-development-phases)
6. [Phase 1: Environment Setup (Local Machine)](#6-phase-1-environment-setup-local-machine)
7. [Phase 2: Gazebo Simulation Environment](#7-phase-2-gazebo-simulation-environment)
8. [Phase 3: Perception Module — YOLOv8](#8-phase-3-perception-module--yolov8)
9. [Phase 4: Navigation Stack — ORB-SLAM3 + Nav2](#9-phase-4-navigation-stack--orb-slam3--nav2)
10. [Phase 5: Cognitive Layer — LLM Integration](#10-phase-5-cognitive-layer--llm-integration)
11. [Phase 6: Full System Integration in Simulation](#11-phase-6-full-system-integration-in-simulation)
12. [Phase 7: Bebop 2 Bridge & Real-World Transition](#12-phase-7-bebop-2-bridge--real-world-transition)
13. [Functional Requirements](#13-functional-requirements)
14. [Non-Functional Requirements](#14-non-functional-requirements)
15. [Evaluation & Success Metrics](#15-evaluation--success-metrics)
16. [Tech Stack & Dependencies](#16-tech-stack--dependencies)
17. [Repository Structure](#17-repository-structure)
18. [Risk Register](#18-risk-register)
19. [Glossary](#19-glossary)

---

## 1. Project Overview

AutoBee is an autonomous aerial surveillance agent designed to replace the paradigm of the "flying camera" operated by a fatigued human with an "agentic AI" that can receive high-level natural language commands, plan its own flight path, navigate GPS-denied environments, and deliver semantically rich crowd analytics and anomaly alerts — with no joystick input required.

The system is built to serve campus security in Nigerian tertiary institutions, where threats such as cultism, civil unrest, and infrastructure vandalism require rapid, intelligent aerial response that current commercial drones and manpower cannot provide.

### The Problem in One Sentence
Current drone surveillance fails not because of poor hardware, but because the cognitive load of simultaneously flying a drone and interpreting its video feed overwhelms human operators within twenty minutes of continuous monitoring.

### The Solution in One Sentence
Replace the operator's dual cognitive burden with a three-layer AI stack: an LLM for intent parsing, ORB-SLAM3 for GPS-denied navigation, and YOLOv8 for real-time crowd analytics — all orchestrated through ROS2.

### Current Phase
**Simulation is the entire priority.** The system must be fully functional and validated inside the Gazebo simulator before any real-world flight testing begins. The Parrot Bebop 2 drone is the target physical platform, but it will not be touched until the simulation phase is complete.

---

## 2. Goals & Non-Goals

### Goals (In Scope)

- Build a fully functional autonomous surveillance system that runs on a local Linux machine via a reproducible Docker-based development environment (ROS2 Humble on Ubuntu 22.04 inside the container).
- Simulate the complete system — navigation, perception, and cognitive reasoning — inside Gazebo Fortress.
- Implement a three-layer Cognitive-Agentic Architecture with ROS2 Humble as middleware.
- Integrate YOLOv8 (fine-tuned on VisDrone) for real-time aerial crowd detection and anomaly flagging.
- Integrate ORB-SLAM3 with IMU fusion for GPS-denied localization.
- Integrate an LLM (Gemini via LangChain) to convert natural language into ROS2 Action Goals.
- Solve the Parrot Bebop 2 ↔ ROS2 compatibility problem using the `ros1_bridge` strategy.
- Validate performance against defined metrics (ATE, mAP, TSR, FPS, latency).
- Produce a documented evaluation framework comparing NLP metrics (BLEU/ROUGE) against robotics metrics (TSR, NE, SPL).

### Non-Goals (Explicitly Out of Scope)

- Custom drone hardware or flight controller design.
- Facial recognition or individual identity tracking.
- Physical real-world flight testing (deferred to post-simulation validation).
- Multi-drone swarm coordination (architectural provisions made, but implementation deferred).
- Cloud deployment of any component.
- Docker or containerization of any kind.
- RTK-GPS reliance — the system must function in GPS-denied environments from the start.
- Paid LLM API usage above what is minimally needed for testing.

> **Note on Docker:** Earlier revisions of this PRD excluded Docker. This constraint has been lifted because the host development machine runs Ubuntu 25.10 with Python 3.13, neither of which are supported by ROS2 Humble apt binaries. Docker provides the cleanest path to the PRD's intended Ubuntu 22.04 / ROS2 Humble stack while keeping the project portable to other hosts (including potential future macOS development).

---

## 3. Critical Constraint: Parrot Bebop 2 on ROS2

### The Problem

The Parrot Bebop 2 is an excellent drone for research — open SDK, 14MP fisheye camera, and accessible pricing. However, its only mature, well-tested ROS driver is [`bebop_autonomy`](https://github.com/AutonomyLab/bebop_autonomy), which is a **ROS1** package. It does not run natively in ROS2 Humble. This is the most critical integration challenge in the entire project.

### Evaluated Options

| Option | Description | Verdict |
|--------|-------------|---------|
| **A: `ros1_bridge`** | Run ROS1 Noetic + ROS2 Humble simultaneously. Bridge topics/services between them. bebop_autonomy runs on ROS1 side; all project nodes run on ROS2 side. | ✅ **Recommended — Most Stable** |
| **B: Community ROS2 Port** | Use an unofficial community-maintained ROS2 port of bebop_autonomy (e.g., `ros2_bebop`). | ⚠️ Fragile — unmaintained forks, incomplete features |
| **C: MAVLink Wrapper** | Re-flash Bebop with custom firmware and use a MAVLink-ROS2 bridge. | ❌ High risk — voids warranty, complex |

### Chosen Solution: `ros1_bridge` (Option A)

The `ros1_bridge` package is the official Anthropic-recommended solution for exactly this class of problem. It allows ROS1 and ROS2 nodes to communicate over a shared network by dynamically bridging topics, services, and actions.

#### What This Means in Practice

```
[ Parrot Bebop 2 (WiFi) ]
         |
         ↓ (WiFi UDP)
[ ROS1 Noetic ]
  └── bebop_autonomy driver
      publishes: /bebop/image_raw
      publishes: /bebop/odom
      subscribes: /bebop/cmd_vel
         |
         ↓ (ros1_bridge)
[ ROS2 Humble ] ← ALL your project nodes live here
  ├── llm_agent_node
  ├── yolov8_perception_node
  ├── orb_slam3_node
  └── nav2 stack
```

#### Setup Requirements for the Bridge

1. **Both ROS distros must be installed on the same machine** — Ubuntu 20.04 supports both Noetic and Foxy/Galactic. However, Ubuntu 22.04 natively supports ROS2 Humble. For Noetic on 22.04, a source build is needed.

> **Recommendation:** Use Ubuntu 20.04 LTS if possible, which natively supports both ROS1 Noetic and ROS2 Galactic/Humble side-by-side with less friction. If the machine is already on 22.04, proceed with a Noetic source build alongside Humble.

2. **`ros1_bridge` must be built from source** when combining Noetic with Humble, as no official binary exists for this pairing.

3. **Topic mapping** must be explicitly configured. A `bridge.yaml` config file maps Bebop's ROS1 topic names to the ROS2 topic names your nodes expect.

#### Bridge Configuration (bridge.yaml — to be created in Sprint 1)

```yaml
# ros1_bridge topic mappings for Parrot Bebop 2
topics:
  - ros1_topic_name: "/bebop/image_raw"
    ros2_topic_name: "/drone/camera/image_raw"
    ros1_type_name: "sensor_msgs/Image"
    ros2_type_name: "sensor_msgs/msg/Image"
    
  - ros1_topic_name: "/bebop/odom"
    ros2_topic_name: "/drone/odom"
    ros1_type_name: "nav_msgs/Odometry"
    ros2_type_name: "nav_msgs/msg/Odometry"
    
  - ros1_topic_name: "/bebop/cmd_vel"
    ros2_topic_name: "/drone/cmd_vel"
    ros1_type_name: "geometry_msgs/Twist"
    ros2_type_name: "geometry_msgs/msg/Twist"
    
  - ros1_topic_name: "/bebop/takeoff"
    ros2_topic_name: "/drone/takeoff"
    ros1_type_name: "std_msgs/Empty"
    ros2_type_name: "std_msgs/msg/Empty"
    
  - ros1_topic_name: "/bebop/land"
    ros2_topic_name: "/drone/land"
    ros1_type_name: "std_msgs/Empty"
    ros2_type_name: "std_msgs/msg/Empty"
```

#### Important Note on Simulation vs Real Drone

During simulation (which is the current priority), the `ros1_bridge` is **not needed at all**. The Gazebo simulation publishes and subscribes to ROS2 topics natively. The bridge is only set up in Phase 7 when transitioning to the physical Bebop 2.

---

## 4. System Architecture Summary

The system uses a **Hierarchical Cognitive-Agentic Architecture** with three layers, all orchestrated through ROS2 Humble.

```
┌─────────────────────────────────────────────────────────────────┐
│                     COGNITIVE LAYER                             │
│   LLM Agent (Gemini via LangChain)  +  Campus Ontology         │
│   Node: llm_agent_node                                          │
│   Input:  /user_command (String)                                │
│   Output: /mission_plan (JSON ActionGoal)                       │
└────────────────────────┬────────────────────────────────────────┘
                         │ ROS2 Action Goal
┌────────────────────────▼────────────────────────────────────────┐
│                     EXECUTIVE LAYER                             │
│   Nav2 Stack (BehaviorTree + GlobalPlanner + LocalPlanner)      │
│   Input:  /mission_plan, /slam/pose (from Reactive Layer)       │
│   Output: /drone/cmd_vel (Twist)                                │
└────────────────────────┬────────────────────────────────────────┘
                         │ cmd_vel
┌────────────────────────▼────────────────────────────────────────┐
│                     REACTIVE LAYER                              │
│   ┌──────────────────────┐   ┌──────────────────────────┐       │
│   │ ORB-SLAM3            │   │ YOLOv8 Perception        │       │
│   │ (Visual-Inertial)    │   │ (VisDrone fine-tuned)    │       │
│   │ Output: /slam/pose   │   │ Output: /detections      │       │
│   │ Output: /slam/map    │   │ Output: /anomaly_alerts  │       │
│   └──────────────────────┘   └──────────────────────────┘       │
│               │                           │                      │
│   ┌───────────▼───────────────────────────▼───────────────┐     │
│   │        Flight Controller (Simulated / Real)           │     │
│   │        Safety Watchdog Node                           │     │
│   └───────────────────────────────────────────────────────┘     │
└─────────────────────────────────────────────────────────────────┘
```

---

## 5. Development Phases

Development follows an **Agile/Sprint methodology**, with simulation completion as the hard gate before any real-world work begins.

| Phase | Name | Priority | Status |
|-------|------|----------|--------|
| 1 | Environment Setup (Local Machine) | 🔴 Critical | Pending |
| 2 | Gazebo Simulation World | 🔴 Critical | Pending |
| 3 | YOLOv8 Perception Module | 🔴 Critical | Pending |
| 4 | ORB-SLAM3 + Nav2 Navigation | 🔴 Critical | Pending |
| 5 | LLM Cognitive Layer | 🔴 Critical | Pending |
| 6 | Full Simulation Integration | 🔴 Critical | Pending |
| 7 | Bebop 2 Bridge + Real-World | 🟡 Deferred | Blocked by Phase 6 |

---

## 6. Phase 1: Environment Setup (Docker-Based)

### Overview

All development happens inside a Docker container based on `osrf/ros:humble-desktop-full`. This gives us Ubuntu 22.04 + ROS2 Humble + Python 3.10 as the PRD originally intended, regardless of the host OS. The host machine in use is Ubuntu 25.10 with Python 3.13 and no NVIDIA GPU, which is why the containerized path is required.

### Host Machine (Current)

| Component | Current Host | Notes |
|-----------|--------------|-------|
| OS | Ubuntu 25.10 (Questing) | Host only — container runs Ubuntu 22.04 |
| CPU | 4-core Intel | Meets minimum |
| RAM | 14 GB | Meets minimum |
| GPU | Intel HD 520 (integrated) | No CUDA — YOLOv8n CPU inference; FPS target revised to ≥ 5 |
| Storage | ~96 GB free | Sufficient for image + datasets |

> **YOLOv8 on CPU:** With no NVIDIA GPU, YOLOv8s will not hit 10 FPS. The project will use **YOLOv8n** (nano variant) and the **Inference FPS target is revised from ≥ 10 to ≥ 5**. If a CUDA-capable machine is added later, the model can be upgraded back to YOLOv8s without code changes — only the model weights file changes.

### Project Files

The Docker setup lives in the project root:

```
AUTOBEE/
├── Dockerfile              # osrf/ros:humble-desktop-full + Nav2 + Python deps + Pangolin
├── docker-compose.yml      # X11 forwarding, host network, workspace volume mount
├── .dockerignore
├── scripts/
│   ├── xhost_setup.sh      # Host-side: grant X11 to container
│   └── build_orbslam3.sh   # Run inside container to build ORB-SLAM3
└── workspace/              # Mounted into container at /root/autobee_ws (persists across container runs)
    └── src/
```

### What's in the Container

| Layer | Contents |
|-------|----------|
| Base | `osrf/ros:humble-desktop-full` (Ubuntu 22.04 + ROS2 Humble + RViz + demo pkgs) |
| ROS2 apt packages | `ros-humble-ros-gz` (Gazebo Fortress bridge), `ros-humble-navigation2`, `ros-humble-nav2-bringup`, `ros-humble-vision-msgs`, `ros-humble-tf2-ros`, `ros-humble-robot-localization`, `ros-humble-teleop-twist-keyboard` |
| Build tools | `build-essential`, `cmake`, `python3-colcon-common-extensions`, `python3-rosdep`, `python3-vcstool` |
| Python packages | `ultralytics`, `langchain`, `langchain-google-genai`, `google-generativeai`, `opencv-python-headless`, `numpy`, `scipy`, `transforms3d` |
| ORB-SLAM3 deps | `libopencv-dev`, `libeigen3-dev`, `libglew-dev`, `libssl-dev`, `libboost-all-dev`, Pangolin (built from source — no apt package exists for Humble-era Ubuntu 22.04) |
| ORB-SLAM3 itself | Built at runtime via `scripts/build_orbslam3.sh` to isolate failures from the image build |

### Setup Procedure (Host → Container)

```bash
# 1. On the host, grant X11 access to local Docker containers (once per login session)
./scripts/xhost_setup.sh

# 2. Build the image (first time only, ~20–40 min depending on bandwidth)
docker compose build

# 3. Start an interactive container
docker compose run --rm autobee

# Inside the container — build ORB-SLAM3 (~10–20 min, one-time)
/root/scripts/build_orbslam3.sh

# Still inside the container — initialize the ROS2 workspace
cd /root/autobee_ws && colcon build
```

### Key Design Decisions

- **`network_mode: host`** — so ROS2 DDS discovery works between container and any future ROS2 node running on the host.
- **Volume-mounted `workspace/`** — source code and build artifacts persist on the host between container sessions, so `docker compose run --rm` is safe.
- **Root user inside container** — keeps the setup simple. File ownership on the host-mounted volume will match the root UID; if this causes friction later, a non-root user with matching host UID can be added.
- **X11 via `/tmp/.X11-unix` bind mount** — Gazebo, RViz, and `rqt_image_view` render to the host display with minimal overhead.
- **No NVIDIA runtime** — the host has Intel integrated graphics only. The `NVIDIA_*` env vars in compose are harmless no-ops on this machine and activate automatically on any GPU host.

### Deliverable for Phase 1

- [ ] `docker compose build` completes without errors
- [ ] `docker compose run --rm autobee` drops into a shell with ROS2 sourced
- [ ] Inside the container: `ros2 topic list` runs without errors
- [ ] Inside the container: `ign gazebo` launches an empty world and displays on the host X server
- [ ] Inside the container: `python3 -c "from ultralytics import YOLO; print('YOLOv8 OK')"` passes
- [ ] Inside the container: `ros2 launch nav2_bringup navigation_launch.py` does not crash
- [ ] Inside the container: `/root/scripts/build_orbslam3.sh` completes successfully

---

## 7. Phase 2: Gazebo Simulation Environment

### Overview

A digital twin of Bingham University campus (or a representative environment with similar spatial properties) is built in Gazebo Fortress. This is the primary testing sandbox for the entire project. No real drone flies until every module performs correctly here.

### Simulation World Requirements

The Gazebo world must contain:

| Feature | Specification |
|---------|---------------|
| Outdoor area | Open convocation ground / courtyard with textured surfaces |
| Indoor corridors | At least one indoor corridor for GPS-denied SLAM testing |
| Simulated crowd | Spawnable actor agents (Gazebo actors) representing pedestrians |
| Landmarks | Named semantic zones (e.g., "Main Gate", "Convocation Ground", "Library") mapped in the Campus Ontology |
| Lighting | Variable lighting to stress-test the vision system |
| Obstacles | Static obstacles (benches, walls) and at least two dynamic obstacles |

### Drone Model in Gazebo

The simulated drone is modeled as a quadrotor with the following sensor suite, matching the Parrot Bebop 2 spec:

```xml
<!-- gazebo_drone.urdf.xacro (abridged) -->
<robot name="autobee_drone">
  <!-- Quadrotor body -->
  <link name="base_link"> ... </link>

  <!-- Monocular camera (14MP equivalent, 90° FOV fisheye) -->
  <sensor name="camera" type="camera">
    <camera>
      <horizontal_fov>1.5708</horizontal_fov>  <!-- 90 degrees -->
      <image>
        <width>1920</width>
        <height>1080</height>
      </image>
    </camera>
    <plugin name="camera_plugin"
      filename="libgazebo_ros_camera.so">
      <ros>
        <remapping>image_raw:=/drone/camera/image_raw</remapping>
      </ros>
    </plugin>
  </sensor>

  <!-- IMU -->
  <sensor name="imu" type="imu">
    <plugin filename="libgazebo_ros_imu_sensor.so">
      <ros>
        <remapping>~/out:=/drone/imu</remapping>
      </ros>
    </plugin>
  </sensor>
</robot>
```

### Simulated Actor Population (Crowd Simulation)

Gazebo's `<actor>` tag is used to populate the crowd. Actors follow scripted paths but can be configured with varying densities to trigger anomaly detection thresholds.

```xml
<!-- Spawn 10 pedestrian actors in the convocation ground -->
<actor name="pedestrian_1">
  <skin><filename>walk.dae</filename></skin>
  <animation name="walking">
    <filename>walk.dae</filename>
    <interpolate_x>true</interpolate_x>
  </animation>
  <script>
    <trajectory id="0" type="walking">
      <waypoint><time>0</time><pose>5 2 0 0 0 0</pose></waypoint>
      <waypoint><time>10</time><pose>10 2 0 0 0 0</pose></waypoint>
    </trajectory>
  </script>
</actor>
```

### ROS2 ↔ Gazebo Bridge Topics

| Gazebo Topic | ROS2 Topic | Message Type |
|--------------|------------|--------------|
| `/drone/camera/image_raw` | `/drone/camera/image_raw` | `sensor_msgs/msg/Image` |
| `/drone/imu` | `/drone/imu` | `sensor_msgs/msg/Imu` |
| `/drone/odom` | `/drone/odom` | `nav_msgs/msg/Odometry` |
| `/drone/cmd_vel` | `/drone/cmd_vel` | `geometry_msgs/msg/Twist` |

### Deliverable for Phase 2

- [ ] Gazebo world launches with the drone model and at least 10 pedestrian actors
- [ ] `/drone/camera/image_raw` publishes at 30 FPS when monitored with `ros2 topic hz`
- [ ] `/drone/imu` publishes data
- [ ] Manual teleoperation with `ros2 run teleop_twist_keyboard teleop_twist_keyboard` moves the simulated drone

---

## 8. Phase 3: Perception Module — YOLOv8

### Overview

The perception module is a dedicated ROS2 node (`yolov8_perception_node`) that subscribes to the camera topic, runs inference with a fine-tuned YOLOv8 model, and publishes detected objects, crowd density estimates, and anomaly alerts.

### Model Strategy

| Model Variant | Size | Speed (GPU) | Use Case |
|---------------|------|-------------|----------|
| YOLOv8n | 3.2MB | ~100 FPS | CPU fallback, resource-constrained |
| YOLOv8s | 11.2MB | ~80 FPS | Balanced — **primary choice for simulation** |
| YOLOv8m | 25.9MB | ~50 FPS | Better accuracy, use if GPU available |
| YOLOv8x | 68.7MB | ~25 FPS | Not suitable for edge/drone |

**Selected model for simulation: YOLOv8s**, fine-tuned on VisDrone-CC2020.

### Training Dataset

| Dataset | Purpose | Source |
|---------|---------|--------|
| VisDrone-CC2020 | Primary training — aerial pedestrian images | [VisDrone GitHub](https://github.com/VisDrone/VisDrone-Dataset) |
| COCO (person class only) | Pre-training backbone | Standard |
| Custom Bingham Campus | Validation — recorded in Phase 6 | Primary data collection |

### Fine-Tuning Script

```python
# train_yolov8_visdrone.py
from ultralytics import YOLO

model = YOLO("yolov8s.pt")  # Start from pretrained weights

results = model.train(
    data="visdrone.yaml",    # Dataset config
    epochs=100,
    imgsz=640,
    batch=16,
    device=0,                # GPU 0; use 'cpu' if no GPU
    project="autobee_runs",
    name="yolov8s_visdrone",
    patience=20,             # Early stopping
    augment=True,            # Important for aerial data variety
)
```

### `visdrone.yaml` (Dataset Config)

```yaml
path: ~/datasets/VisDrone
train: images/train
val: images/val
test: images/test

nc: 10
names: ['pedestrian', 'people', 'bicycle', 'car', 'van',
        'truck', 'tricycle', 'awning-tricycle', 'bus', 'motor']
```

> **Crowd analytics uses the `pedestrian` and `people` classes exclusively** for density estimation. Other classes are tracked but do not contribute to the crowd density threshold.

### Perception ROS2 Node Specification

**Node name:** `yolov8_perception_node`

**Subscriptions:**
- `/drone/camera/image_raw` — `sensor_msgs/msg/Image`

**Publications:**
- `/perception/detections` — `vision_msgs/msg/Detection2DArray`
- `/perception/crowd_density` — `std_msgs/msg/Float32` (persons per m²)
- `/perception/anomaly_alert` — `std_msgs/msg/String` (JSON payload)
- `/perception/annotated_image` — `sensor_msgs/msg/Image` (for visualization)

**Parameters:**
- `model_path` (string): Path to fine-tuned `.pt` weights
- `confidence_threshold` (float, default: 0.5): Minimum detection confidence
- `crowd_density_threshold` (float, default: 2.0): persons/m² above which an anomaly is triggered
- `inference_device` (string, default: `"cuda:0"`): `"cpu"` or `"cuda:0"`

**Anomaly Alert JSON Schema:**
```json
{
  "timestamp": "2026-01-15T10:45:22Z",
  "alert_type": "crowd_density_exceeded",
  "severity": "high",
  "crowd_count": 34,
  "density_estimate": 3.2,
  "confidence": 0.87,
  "sector": "Sector_4",
  "message": "Crowd density exceeds safe limits in Sector 4; potential stampede risk detected"
}
```

### Crowd Density Estimation Logic

Density is estimated by combining person count with altitude-adjusted area estimation:

```python
def estimate_density(detections, altitude_m, frame_width, frame_height, fov_deg=90.0):
    """
    Estimate crowd density in persons per square meter.
    
    Uses altitude and camera FOV to compute the ground area visible in the frame.
    altitude_m: drone altitude from SLAM or barometer
    """
    import math
    
    fov_rad = math.radians(fov_deg)
    ground_width = 2 * altitude_m * math.tan(fov_rad / 2)
    ground_height = ground_width * (frame_height / frame_width)
    visible_area_m2 = ground_width * ground_height
    
    person_count = sum(
        1 for d in detections
        if d.class_name in ["pedestrian", "people"]
        and d.confidence >= 0.5
    )
    
    density = person_count / visible_area_m2 if visible_area_m2 > 0 else 0.0
    return density, person_count, visible_area_m2
```

### Deliverable for Phase 3

- [ ] YOLOv8s fine-tuned on VisDrone with mAP@50 ≥ 50% on validation set
- [ ] `yolov8_perception_node` detects simulated Gazebo pedestrian actors
- [ ] `/perception/crowd_density` publishes at ≥ 10 FPS
- [ ] An anomaly alert is published when ≥ 5 simulated actors are in view
- [ ] Annotated video stream viewable in `rqt_image_view`

---

## 9. Phase 4: Navigation Stack — ORB-SLAM3 + Nav2

### Overview

Autonomous navigation in GPS-denied environments is achieved by pairing **ORB-SLAM3** (localization and mapping) with the **Nav2** stack (path planning and execution).

### ORB-SLAM3 Node Configuration

ORB-SLAM3 runs as a ROS2 node using the `ros2-ORB_SLAM3` wrapper. It ingests the monocular camera feed and IMU data to produce a 6-DOF pose estimate.

**Subscriptions:**
- `/drone/camera/image_raw` — camera frames
- `/drone/imu` — IMU data for visual-inertial fusion

**Publications:**
- `/slam/pose` — `geometry_msgs/msg/PoseStamped`
- `/slam/map_points` — `sensor_msgs/msg/PointCloud2`
- `/tf` — `map → odom → base_link` transform tree

**ORB-SLAM3 Config File (Bebop2_MonoInertial.yaml — to be created):**

```yaml
# Camera parameters (Parrot Bebop 2 fisheye — calibrate before real-world use)
Camera.type: "PinHole"
Camera.fx: 964.828979
Camera.fy: 964.828979
Camera.cx: 960.0
Camera.cy: 540.0
Camera.k1: 0.0
Camera.k2: 0.0
Camera.p1: 0.0
Camera.p2: 0.0
Camera.width: 1920
Camera.height: 1080
Camera.fps: 30.0
Camera.RGB: 1

# ORB Feature parameters
ORBextractor.nFeatures: 1000
ORBextractor.scaleFactor: 1.2
ORBextractor.nLevels: 8
ORBextractor.iniThFAST: 20
ORBextractor.minThFAST: 7

# IMU parameters (Bebop 2 internal IMU)
IMU.NoiseGyro: 0.004
IMU.NoiseAcc: 0.04
IMU.GyroWalk: 0.000022
IMU.AccWalk: 0.00086
IMU.Frequency: 200.0
```

> **Important:** Camera calibration parameters above are approximate defaults. Before real-world deployment on the Bebop 2, these values must be recalibrated using a checkerboard pattern and the `camera_calibration` ROS2 package.

### Dynamic Object Filtering (Semantic SLAM)

To prevent moving pedestrians from corrupting the SLAM map (the "static world assumption" problem), a dynamic masking preprocessor node is inserted between the camera and ORB-SLAM3:

```
/drone/camera/image_raw
        |
        ▼
[ dynamic_mask_node ]   ← Uses YOLOv8 detections to mask moving persons
        |
        ▼
/slam/camera/masked_image
        |
        ▼
[ ORB-SLAM3 node ]
```

The `dynamic_mask_node` blacks out bounding box regions containing detected `pedestrian`/`people` classes before those frames reach ORB-SLAM3, preventing dynamic features from being added to the map. This directly addresses the "Crowd-SLAM" problem identified in the literature.

### Nav2 Stack Configuration

Nav2 handles path planning and obstacle avoidance using a Behavior Tree.

**Critical Nav2 parameters for aerial navigation (nav2_params.yaml — abridged):**

```yaml
controller_server:
  ros__parameters:
    controller_frequency: 20.0
    FollowPath:
      plugin: "nav2_mppi_controller::MPPIController"  # Better than DWA for aerial
      time_steps: 56
      model_dt: 0.05
      batch_size: 2000
      vx_std: 0.2
      vy_std: 0.2
      wz_std: 0.4
      vx_max: 2.0
      vx_min: -0.5

planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5
      use_astar: true

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      width: 10
      height: 10
      resolution: 0.1

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      resolution: 0.1
```

### Safety Watchdog Node

A dedicated `safety_watchdog_node` monitors all critical nodes. If any expected heartbeat topic goes silent for more than 2 seconds, it publishes a `SAFE_LAND` command.

```python
# safety_watchdog_node.py (abridged)
class SafetyWatchdog(Node):
    MONITORED_TOPICS = [
        ("/slam/pose", PoseStamped, 2.0),
        ("/perception/detections", Detection2DArray, 3.0),
        ("/llm_agent/status", String, 10.0),
    ]
    
    def on_timeout(self, topic_name):
        self.get_logger().error(f"WATCHDOG: Timeout on {topic_name}. Initiating SAFE LAND.")
        self.publish_safe_land()
    
    def publish_safe_land(self):
        msg = String()
        msg.data = "SAFE_LAND"
        self.emergency_pub.publish(msg)
```

### Deliverable for Phase 4

- [ ] ORB-SLAM3 produces a pose estimate from simulated camera + IMU data
- [ ] `/tf` tree (map → odom → base_link) publishes without gaps
- [ ] Drone navigates autonomously to a hardcoded goal pose in Gazebo without collision
- [ ] Nav2 `navigate_to_pose` action server responds to goal requests
- [ ] Safety watchdog correctly triggers safe land on simulated node failure
- [ ] ATE < 0.5 m over a 50-meter simulated trajectory

---

## 10. Phase 5: Cognitive Layer — LLM Integration

### Overview

The cognitive layer translates natural language intent from the security operator into structured ROS2 Action Goals using **Gemini Pro via LangChain**.

### LLM Agent Node Specification

**Node name:** `llm_agent_node`

**Subscriptions:**
- `/user_command` — `std_msgs/msg/String` (raw text command)
- `/perception/anomaly_alert` — `std_msgs/msg/String` (for generating operator reports)

**Publications:**
- `/mission_plan` — `std_msgs/msg/String` (JSON ActionGoal payload)
- `/llm_agent/status` — `std_msgs/msg/String` (heartbeat)
- `/operator_report` — `std_msgs/msg/String` (natural language status updates back to operator)

### Campus Ontology

The ontology is a JSON file that maps natural language place names to physical simulation coordinates. This is one of two grounding mechanisms the LLM uses; the other is the special `takeoff_pose` reference (see below) for commands that don't name a specific landmark.

```json
{
  "campus_ontology": {
    "locations": {
      "main_gate": {"x": 0.0, "y": 0.0, "z": 2.0, "yaw": 0.0},
      "convocation_ground": {"x": 45.0, "y": 30.0, "z": 5.0, "yaw": 0.0},
      "library": {"x": 80.0, "y": 10.0, "z": 3.0, "yaw": 1.57},
      "engineering_block": {"x": 60.0, "y": -20.0, "z": 3.0, "yaw": 0.0},
      "admin_building": {"x": 20.0, "y": 50.0, "z": 3.0, "yaw": 0.0},
      "sports_complex": {"x": -30.0, "y": 40.0, "z": 5.0, "yaw": 0.0}
    },
    "zones": {
      "high_risk_zones": ["convocation_ground", "sports_complex"],
      "indoor_zones": ["library", "engineering_block"]
    }
  }
}
```

#### Reserved Reference: `takeoff_pose`

Not every command names a specific landmark. Phrases like *"is this place overcrowded?"*, *"scan around me"*, or *"check this area"* refer to the drone's **current/launch position**, not a pre-mapped location. To support these, the ontology reserves a special location name `takeoff_pose` that resolves at runtime to the SLAM pose recorded at arming time. This is what makes the system deployable at venues that have not been pre-mapped — including demo day. The `takeoff_pose` reference is the default `center` when a `survey_local_area` action does not specify one.

### System Prompt Engineering (LangChain)

The LLM is constrained by a carefully engineered system prompt that prevents hallucinated commands and enforces a strict JSON output schema:

```python
SYSTEM_PROMPT = """
You are the Cognitive Agent for AutoBee, an autonomous surveillance drone operating 
at Bingham University, Nigeria. You receive natural language commands from a security 
operator and must convert them into structured JSON mission plans.

RULES:
1. You NEVER output anything other than valid JSON. No explanations, no markdown.
2. You ONLY use locations defined in the provided campus ontology.
3. You NEVER issue commands that override the safety watchdog.
4. If a command is ambiguous, default to the safest interpretation.
5. Maximum 3 waypoints per mission to prevent context overload.
6. Always include a return_home action as the final step.

OUTPUT SCHEMA:
{
  "mission_id": "<uuid>",
  "original_command": "<user's text>",
  "actions": [
    // Action type 1 — navigate to a named landmark from the ontology.
    // Use when the user references a specific place (e.g. "the convocation ground").
    {
      "type": "navigate_to_pose",
      "target_location": "<location_name>",
      "coordinates": {"x": float, "y": float, "z": float},
      "altitude_m": float,
      "task": "survey | monitor | hover"
    },
    // Action type 2 — survey a circular area around a reference pose.
    // Use when the user does NOT name a specific landmark
    // (e.g. "is this place overcrowded?", "scan around me", "check this area").
    // If "center" is omitted or set to "takeoff_pose", the survey is centered
    // on the drone's launch position.
    {
      "type": "survey_local_area",
      "center": "takeoff_pose | <location_name>",
      "radius_m": float,            // default 20.0
      "altitude_m": float,          // default 8.0
      "pattern": "spiral | orbit | lawnmower",   // default "spiral"
      "duration_s": int,            // default 90
      "alert_threshold": {"crowd_density": float}  // default 2.0 p/m²
    }
  ],
  "perception_tasks": ["crowd_count", "anomaly_detect"],
  "priority": "normal | high | emergency"
}

CAMPUS ONTOLOGY:
{ontology_json}
"""
```

### Command Processing Pipeline

The agent supports two command shapes, matching the two action types in the schema above. Both run through the same LangChain + Gemini pipeline; only the resulting plan differs.

#### Example A — Place-Based Command (uses Campus Ontology)

```
User Command: "Scan the convocation ground for overcrowding"
      |
      ▼
[ LangChain + Gemini Pro ]
  - Loads Campus Ontology
  - Calls Gemini API (local proxy for low latency)
  - Validates JSON output against schema
      |
      ▼
Mission Plan JSON:
{
  "mission_id": "m-001",
  "original_command": "Scan the convocation ground for overcrowding",
  "actions": [
    {
      "type": "navigate_to_pose",
      "target_location": "convocation_ground",
      "coordinates": {"x": 45.0, "y": 30.0, "z": 5.0},
      "altitude_m": 5.0,
      "task": "survey"
    },
    {
      "type": "navigate_to_pose",
      "target_location": "main_gate",
      "coordinates": {"x": 0.0, "y": 0.0, "z": 2.0},
      "altitude_m": 2.0,
      "task": "hover"
    }
  ],
  "perception_tasks": ["crowd_count", "anomaly_detect"],
  "priority": "normal"
}
      |
      ▼
[ Mission Executor Node ] → Publishes to Nav2 NavigateToPose Action Server
```

#### Example B — Local-Area Command (uses `takeoff_pose`)

This is the **demo-day path.** The user references "this place" rather than a named landmark, so the LLM emits a `survey_local_area` action centered on the drone's launch position. No pre-mapped landmark is required, which makes the system deployable at any venue.

```
User Command: "Let me know if this place is overcrowded"
      |
      ▼
[ LangChain + Gemini Pro ]
  - Recognizes "this place" → no ontology lookup needed
  - Recognizes "overcrowded" → set crowd_density alert threshold
  - Fills missing parameters with safe defaults
      |
      ▼
Mission Plan JSON:
{
  "mission_id": "m-002",
  "original_command": "Let me know if this place is overcrowded",
  "actions": [
    {
      "type": "survey_local_area",
      "center": "takeoff_pose",
      "radius_m": 20.0,
      "altitude_m": 8.0,
      "pattern": "spiral",
      "duration_s": 90,
      "alert_threshold": {"crowd_density": 2.0}
    }
  ],
  "perception_tasks": ["crowd_count", "anomaly_detect"],
  "priority": "normal"
}
      |
      ▼
[ Mission Executor Node ]
  1. Snapshots /slam/pose at arm time → resolves "takeoff_pose"
  2. Generates a spiral of waypoints within 20m of that pose at 8m altitude
  3. Configures yolov8_perception_node: crowd_density_threshold=2.0
  4. Issues each waypoint as a Nav2 NavigateToPose goal in sequence
  5. On /perception/anomaly_alert, captures snapshot + bearing relative to takeoff_pose,
     forwards to llm_agent_node for natural-language report synthesis
  6. On duration_s expiry or operator cancel, returns to takeoff_pose and lands
```

This second path is what enables the demo-day scenario T-09 (Section 11) and supports FR-13.

### Latency Mitigation

Per the empirical findings of Pollini et al. (2025), cloud LLM API calls can exceed 1 second of latency. The following strategies are applied:

1. **Local API proxy:** Run Gemini API calls through a lightweight local caching proxy that stores responses for repeated/similar commands.
2. **Command latency target:** < 5 seconds from user input to drone first movement (as per the non-functional requirements).
3. **Async execution:** The LLM call is made asynchronously. Nav2 begins moving toward the last known goal while the new plan is being generated, then updates on receipt.
4. **Fallback commands:** A set of 10 pre-compiled mission plans for common commands (patrol perimeter, scan convocation ground, return home) that bypass LLM inference entirely.

### Deliverable for Phase 5

- [ ] `llm_agent_node` publishes a valid JSON mission plan within 5 seconds for any of 20 test commands
- [ ] Campus ontology correctly resolves all 6 defined locations
- [ ] `takeoff_pose` reference resolves correctly at runtime to the current SLAM pose at arm time
- [ ] LLM correctly emits `survey_local_area` (not `navigate_to_pose`) for at least 5 phrasing variants of local-area commands ("is this place overcrowded", "scan around me", "check this area for crowds", "watch here for big groups", "tell me if it gets busy here")
- [ ] Invalid/ambiguous commands produce a graceful fallback, not a crash
- [ ] `Task Success Rate (TSR)` ≥ 80% on the 20-command test suite (mix of place-based and local-area commands)
- [ ] No LLM-generated command bypasses the safety watchdog

---

## 11. Phase 6: Full System Integration in Simulation

### Overview

All three layers (Cognitive, Executive, Reactive) are launched together in a unified simulation. This phase produces the final validated prototype and generates the evaluation data required for the dissertation.

### Launch Architecture

A single top-level launch file brings up the entire stack:

```python
# autobee_sim_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription

def generate_launch_description():
    return LaunchDescription([
        
        # 1. Gazebo world
        IncludeLaunchDescription("gazebo_campus_world.launch.py"),
        
        # 2. ORB-SLAM3 (Monocular-Inertial mode)
        Node(
            package='orbslam3',
            executable='mono_inertial',
            parameters=[{'vocabulary_file': 'ORBvoc.txt',
                         'settings_file': 'Bebop2_MonoInertial.yaml'}]
        ),
        
        # 3. Dynamic mask preprocessor
        Node(package='autobee_perception', executable='dynamic_mask_node'),
        
        # 4. YOLOv8 perception
        Node(
            package='autobee_perception',
            executable='yolov8_perception_node',
            parameters=[{'model_path': 'models/yolov8s_visdrone.pt',
                         'confidence_threshold': 0.5,
                         'crowd_density_threshold': 2.0}]
        ),
        
        # 5. Nav2 stack
        IncludeLaunchDescription("nav2_bringup.launch.py"),
        
        # 6. LLM Agent
        Node(
            package='autobee_cognition',
            executable='llm_agent_node',
            parameters=[{'ontology_path': 'config/campus_ontology.json'}]
        ),
        
        # 7. Safety watchdog
        Node(package='autobee_core', executable='safety_watchdog_node'),
        
        # 8. Mission executor
        Node(package='autobee_core', executable='mission_executor_node'),
    ])
```

### End-to-End Test Scenarios

The following scenarios must all pass in simulation before Phase 7 is unlocked:

| ID | Command | Expected Behavior | Pass Criteria |
|----|---------|-------------------|---------------|
| T-01 | "Scan the convocation ground for overcrowding" | Drone navigates to convocation\_ground, hovers, publishes crowd density | TSR=Pass, ATE<0.5m, alert triggered at >2p/m² |
| T-02 | "Check the library entrance for unauthorized gatherings" | Drone navigates to library, activates perception | TSR=Pass, detections published |
| T-03 | "Return to base immediately" | Drone navigates to main\_gate | TSR=Pass, RTH completed in <60s |
| T-04 | "Patrol the engineering block and sports complex" | Drone visits both waypoints sequentially | Both waypoints reached, no collision |
| T-05 | "Gibberish input: xkcd surveillance gamma" | Graceful error message published | No crash, fallback response published |
| T-06 | Simulate node failure (kill perception node mid-flight) | Watchdog triggers SAFE\_LAND | Drone lands safely within 5s of failure |
| T-07 | Spawn 20 actors in convocation ground | Crowd density alert published | Alert fired with density > 2.0 p/m² |
| T-08 | Navigate 50m corridor (indoor, no GPS) | ORB-SLAM3 maintains localization | ATE < 0.5m at goal |
| T-09 | "Let me know if this place is overcrowded" (local-area survey, no named landmark) | Drone arms, snapshots takeoff_pose, flies a 20m-radius spiral at 8m altitude, runs perception, fires anomaly alert when an actor cluster crosses 2.0 p/m², returns to takeoff_pose and lands. LLM synthesizes a natural-language report including bearing/distance from takeoff_pose. | TSR=Pass; alert fired with density >2.0 within mission duration; drone returns to within 0.5m of takeoff_pose; report mentions an approximate location and count. **This is the demo-day acceptance test.** |

### Evaluation Data Collection

During simulation runs, the system logs:

1. **Navigation logs:** Pose ground truth (Gazebo) vs SLAM estimate → compute ATE, RPE
2. **Perception logs:** Ground truth actor count vs detected count → compute mAP, recall
3. **Command logs:** Command issued, JSON plan generated, actual drone behavior → compute TSR
4. **Latency logs:** Timestamp from command publish to first cmd\_vel → compute command latency

### Deliverable for Phase 6

- [ ] All 8 test scenarios (T-01 through T-08) pass
- [ ] ATE < 0.5 m across all navigation tests
- [ ] mAP@50 ≥ 50% on simulated crowd detection
- [ ] TSR ≥ 80% on 20-command test suite
- [ ] Command latency < 5 seconds (95th percentile)
- [ ] Inference FPS ≥ 10 on the local machine

---

## 12. Phase 7: Bebop 2 Bridge & Real-World Transition

> **This phase is BLOCKED until Phase 6 is fully complete and all deliverables are signed off.**

### Overview

With simulation validated, the system is transferred to the physical Parrot Bebop 2 drone using the `ros1_bridge` strategy described in Section 3.

### Pre-Flight Checklist

Before the first autonomous flight, the following must be completed:

- [ ] ROS1 Noetic installed (source build on Ubuntu 22.04 or native on Ubuntu 20.04)
- [ ] `bebop_autonomy` driver compiled and tested (`roslaunch bebop_driver bebop_node.launch`)
- [ ] `ros1_bridge` built from source and bridge.yaml configured (see Section 3)
- [ ] Camera calibration completed with checkerboard — new `Bebop2_MonoInertial.yaml` generated
- [ ] ORB-SLAM3 re-tested with real camera feed in a known environment
- [ ] All Phase 6 simulation tests re-run with bridge active (Bebop topics on ROS1 side)
- [ ] Hardware kill switch verified — physical remote override confirmed working
- [ ] All Phase 6 tests passing with ≥ 90% success rate before outdoor flight

### Real-World Testing Progression

Real-world testing must be incremental — never jump to full autonomy on first outdoor flight:

1. **Step 1:** Manual flight via `bebop_autonomy` only — verify driver, camera, IMU
2. **Step 2:** ORB-SLAM3 running during manual flight — verify localization quality
3. **Step 3:** YOLOv8 running during manual flight — verify detections on real crowd
4. **Step 4:** Single autonomous waypoint (Nav2 navigates 5m forward and hovers) — operator standby with kill switch
5. **Step 5:** Full end-to-end autonomous mission on campus — human observer present at all times

---

## 13. Functional Requirements

| ID | Priority | Requirement | Acceptance Criteria |
|----|----------|-------------|---------------------|
| FR-01 | Must Have | Autonomous Navigation | Navigate start → goal without human joystick input in GPS-denied sim environment |
| FR-02 | Must Have | Natural Language Parsing | Parse English text commands into ROS2 goals within 5 seconds |
| FR-03 | Must Have | Real-Time Object Detection | Detect persons at ≥ 0.5 confidence at ≥ 10 FPS |
| FR-04 | Must Have | Crowd Density Estimation | Calculate and publish persons/m² using altitude-adjusted area |
| FR-05 | Must Have | Safety Watchdog | Trigger SAFE\_LAND within 2 seconds of critical node failure |
| FR-06 | Should Have | Anomaly Alerting | Auto-trigger alert when crowd density > 2.0 p/m² |
| FR-07 | Should Have | Return-to-Home | Auto-RTH on battery < 15% or communication loss |
| FR-08 | Should Have | Operator Reports | LLM synthesizes a natural-language situation report from perception data |
| FR-09 | Could Have | Voice Input | Accept voice commands via ROS2-compatible speech-to-text node |
| FR-10 | Could Have | Voice Feedback | Drone speaks mission status updates back to operator |
| FR-11 | Won't Have | Facial Recognition | Explicitly excluded for privacy compliance |
| FR-12 | Won't Have | Multi-drone Swarm | Deferred to future work |
| FR-13 | Must Have | Local-Area Survey Mode | The system must support commands that reference the drone's current position rather than a pre-mapped landmark (e.g. "is this place overcrowded?", "scan around me"). On such commands, the LLM emits a `survey_local_area` action centered on `takeoff_pose`, and the mission executor flies a configurable-radius pattern (default 20 m, spiral, 8 m altitude) while running perception. This is the deployment path for venues that have not been pre-mapped, including demo-day. |

---

## 14. Non-Functional Requirements

| ID | Category | Requirement | Target |
|----|----------|-------------|--------|
| NFR-01 | Performance | Command Latency | < 5 seconds (text command → first drone movement) |
| NFR-02 | Performance | Inference Speed | ≥ 5 FPS for YOLOv8n (CPU-only, revised from ≥ 10 FPS on GPU due to lack of NVIDIA hardware on current dev machine) |
| NFR-03 | Performance | LLM Response Time | < 3 seconds for mission plan generation |
| NFR-04 | Reliability | SLAM Drift | < 0.5 m ATE over 50 m trajectory |
| NFR-05 | Reliability | System Uptime | No unhandled crashes during a 20-minute simulated mission |
| NFR-06 | Safety | Failsafe | Hardware/software kill switch fully functional |
| NFR-07 | Safety | Human Override | Manual control recoverable within 1 second of activation |
| NFR-08 | Privacy | Data Handling | No facial features stored; only density metadata and encrypted video |
| NFR-09 | Usability | Operator Interface | Non-technical user can issue a command and receive a report with no training beyond 30 minutes |
| NFR-10 | Maintainability | Code Quality | All ROS2 nodes must have unit tests; CI-runnable test suite |

---

## 15. Evaluation & Success Metrics

### Navigation Metrics

| Metric | Definition | Target | Measurement Tool |
|--------|------------|--------|-----------------|
| ATE (Absolute Trajectory Error) | RMS error between estimated and ground-truth trajectory | < 0.5 m | evo\_ape (TUM tools) |
| RPE (Relative Pose Error) | Local drift per unit distance | < 0.05 m/m | evo\_rpe |
| Navigation Success Rate (NSR) | % of missions reaching goal without collision | ≥ 85% | Mission logs |
| Path Length Efficiency | Actual path / optimal path | < 1.3 | Mission logs |

### Perception Metrics

| Metric | Definition | Target | Measurement Tool |
|--------|------------|--------|-----------------|
| mAP@50 | Mean Average Precision at IoU=0.5 | ≥ 50% | Ultralytics val |
| mAP@50-95 | Mean Average Precision across IoU thresholds | ≥ 30% | Ultralytics val |
| Inference FPS | Frames processed per second | ≥ 5 FPS (YOLOv8n, CPU) | ROS2 hz monitor |
| Crowd Count Error | % error in people count vs ground truth | ≤ 20% | Gazebo actor count |
| False Positive Rate | Spurious anomaly alerts / total alerts | < 15% | Alert logs |

### Cognitive Metrics

| Metric | Definition | Target | Measurement Tool |
|--------|------------|--------|-----------------|
| Task Success Rate (TSR) | % of NL commands correctly executed end-to-end | ≥ 80% | Test suite logs |
| Command Latency | Time from /user\_command publish to first cmd\_vel | < 5 seconds | ROS2 timestamps |
| Navigation Error (NE) | Euclidean distance from intended to actual goal | < 1.0 m | Pose logs |
| SPL (Success weighted by Path Length) | TSR penalized by path efficiency | ≥ 0.6 | Computed metric |

### Comparative Evaluation (NLP vs Robotics Metrics)

This project produces the evaluation framework as a research contribution. All 20 test commands are evaluated using both:

- **NLP metrics:** BLEU-4 and ROUGE-L (comparing generated JSON to reference JSON)
- **Robotics metrics:** TSR, NE, SPL (measuring actual physical task completion)

The hypothesis to be proven: BLEU/ROUGE poorly predict whether a robot physically completes a task. High BLEU can coexist with failed navigation, validating the argument for robotics-specific evaluation metrics.

---

## 16. Tech Stack & Dependencies

### Core Stack

| Component | Technology | Version | License |
|-----------|------------|---------|---------|
| Middleware | ROS2 Humble Hawksbill | Humble | Apache 2.0 |
| Simulation | Gazebo Fortress (Ignition) | 6.x | Apache 2.0 |
| Navigation | Nav2 | Humble-compatible | Apache 2.0 |
| SLAM | ORB-SLAM3 | 1.0 | GPLv3 |
| Object Detection | Ultralytics YOLOv8 | 8.x | AGPL-3.0 |
| LLM Orchestration | LangChain | 0.3.x | MIT |
| LLM Model | Google Gemini Pro | via API | Proprietary |
| Language | Python 3.10 / C++17 | — | — |
| OS | Ubuntu 22.04 LTS | Jammy | — |

### Python Packages

```txt
# requirements.txt
ultralytics>=8.0.0
langchain>=0.3.0
google-generativeai>=0.8.0
opencv-python-headless>=4.8.0
numpy>=1.24.0
scipy>=1.11.0
transforms3d>=0.4.1
rclpy  # installed with ROS2
```

### ROS2 Packages

```bash
# Install via apt
ros-humble-navigation2
ros-humble-nav2-bringup
ros-humble-ros-gz
ros-humble-vision-msgs
ros-humble-tf2-ros
ros-humble-robot-localization
```

---

## 17. Repository Structure

```
autobee_ws/
└── src/
    ├── autobee_core/              # Core system nodes
    │   ├── autobee_core/
    │   │   ├── mission_executor_node.py     # consumes mission plan JSON; dispatches navigate_to_pose and survey_local_area
    │   │   ├── survey_pattern_generator.py  # spiral / orbit / lawnmower waypoint generators around a center pose
    │   │   ├── safety_watchdog_node.py
    │   │   └── __init__.py
    │   ├── config/
    │   │   ├── campus_ontology.json
    │   │   └── nav2_params.yaml
    │   ├── launch/
    │   │   └── autobee_sim_launch.py
    │   └── package.xml
    │
    ├── autobee_perception/        # YOLOv8 + crowd analytics
    │   ├── autobee_perception/
    │   │   ├── yolov8_perception_node.py
    │   │   ├── dynamic_mask_node.py
    │   │   ├── crowd_density_estimator.py
    │   │   └── __init__.py
    │   ├── models/
    │   │   └── yolov8s_visdrone.pt  # fine-tuned weights (gitignored)
    │   ├── scripts/
    │   │   └── train_yolov8_visdrone.py
    │   └── package.xml
    │
    ├── autobee_cognition/         # LLM agent + ontology
    │   ├── autobee_cognition/
    │   │   ├── llm_agent_node.py
    │   │   ├── campus_ontology_loader.py
    │   │   ├── command_validator.py
    │   │   └── __init__.py
    │   ├── prompts/
    │   │   └── system_prompt.txt
    │   └── package.xml
    │
    ├── autobee_simulation/        # Gazebo world + drone model
    │   ├── worlds/
    │   │   └── bingham_campus.world
    │   ├── models/
    │   │   ├── autobee_drone/
    │   │   │   ├── model.sdf
    │   │   │   └── model.config
    │   │   └── campus_buildings/
    │   ├── launch/
    │   │   └── gazebo_campus_world.launch.py
    │   └── package.xml
    │
    ├── autobee_slam/              # ORB-SLAM3 ROS2 wrapper + config
    │   ├── config/
    │   │   ├── Bebop2_MonoInertial.yaml
    │   │   └── ORBvoc.txt
    │   └── package.xml
    │
    └── autobee_interfaces/        # Custom ROS2 message/service types
        ├── msg/
        │   ├── AnomalyAlert.msg
        │   ├── CrowdDensity.msg
        │   └── MissionPlan.msg
        ├── srv/
        │   └── ExecuteMission.srv
        └── package.xml
```

---

## 18. Risk Register

| ID | Risk | Likelihood | Impact | Mitigation |
|----|------|-----------|--------|------------|
| R-01 | ORB-SLAM3 tracking loss in textureless Gazebo environments | High | High | Add artificial texture to sim world walls; tune ORBextractor.nFeatures |
| R-02 | YOLOv8 fails to detect Gazebo actors (render style mismatch) | Medium | High | Pre-train on rendered/synthetic data; add Gazebo-style images to training set |
| R-03 | Gemini API rate limits or latency spikes during evaluation | Medium | Medium | Implement command caching; fallback to pre-compiled mission plans |
| R-04 | ORB-SLAM3 and Nav2 TF tree conflict | High | High | Carefully map TF frames; use `robot_localization` for EKF fusion |
| R-05 | GPU unavailable on local machine — YOLOv8 FPS < 10 | Low | High | Use YOLOv8n; reduce input resolution to 320px; adjust FPS target to 8 |
| R-06 | `ros1_bridge` build fails on Ubuntu 22.04 with Humble | Medium | Medium | Use pre-built bridge binaries if available; or test on Ubuntu 20.04 VM just for bridge testing |
| R-07 | Bebop 2 battery limits impose <20 min flight time | — | Medium | Efficient path planning; no waypoints >15 min apart; simulation tests always run first |
| R-08 | LLM generates hallucinated coordinates outside ontology | Medium | High | Strict JSON schema validation layer; reject plans with unknown locations before execution |
| R-09 | Camera calibration for Bebop 2 deviates significantly from sim params | — | Medium | Re-calibrate in Phase 7; never transfer sim YAML directly to real drone |

---

## 19. Glossary

| Term | Definition |
|------|------------|
| **ATE** | Absolute Trajectory Error — RMS distance between estimated and ground-truth trajectory |
| **Agentic AI** | AI systems that can perceive, reason, plan, and act autonomously toward a goal |
| **Bebop 2** | Parrot Bebop 2 quadrotor drone — the physical hardware platform for this project |
| **BLEU** | Bilingual Evaluation Understudy — NLP metric for text similarity; argued to be insufficient alone for robotics evaluation |
| **cmd_vel** | Velocity command topic in ROS2 (geometry\_msgs/Twist) |
| **DDS** | Data Distribution Service — the communication middleware underlying ROS2 |
| **FPS** | Frames Per Second — inference speed metric for YOLOv8 |
| **Gazebo Fortress** | LTS version of Ignition Gazebo simulator, compatible with ROS2 Humble |
| **GPS-Denied** | Environments where GPS signals are unavailable (indoors, urban canyons) |
| **HRI** | Human-Robot Interaction |
| **IMU** | Inertial Measurement Unit — measures acceleration and rotation |
| **LLM** | Large Language Model — e.g., Gemini Pro, GPT-4 |
| **mAP** | Mean Average Precision — standard object detection accuracy metric |
| **Nav2** | ROS2 Navigation Stack — provides autonomous path planning and obstacle avoidance |
| **NE** | Navigation Error — Euclidean distance between intended and reached goal |
| **ORB-SLAM3** | State-of-the-art visual-inertial SLAM library used for GPS-denied navigation |
| **RAG** | Retrieval-Augmented Generation — LLM pattern for grounding responses in external data |
| **ROS2 Humble** | The ROS2 LTS distribution used in this project (Ubuntu 22.04 compatible) |
| **ros1\_bridge** | Official ROS package for bridging topics between ROS1 and ROS2 |
| **RPE** | Relative Pose Error — measures local SLAM drift per unit distance |
| **SLAM** | Simultaneous Localization and Mapping |
| **SPL** | Success weighted by Path Length — navigation metric combining success rate and path efficiency |
| **TSR** | Task Success Rate — % of natural language commands correctly executed end-to-end |
| **UAV** | Unmanned Aerial Vehicle |
| **VisDrone** | Benchmark dataset of drone-captured aerial images for object detection research |
| **Watchdog** | ROS2 node that monitors system health and triggers failsafe on node failure |
| **YOLOv8** | Ultralytics' 8th iteration of the You Only Look Once real-time object detection framework |

---

*This PRD is a living document. It must be updated after each sprint with actual performance data, blockers encountered, and any scope changes agreed upon.*

*Simulation completion is the hard gate before any real-world flight activity. Do not proceed to Phase 7 without all Phase 6 deliverables checked off.*
