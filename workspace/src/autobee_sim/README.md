# autobee_sim

Phase 2 simulation package for AutoBee: campus world + drone model + ros_gz bridge.

## Contents

- `worlds/campus.sdf` — campus world with ground, 8 building landmarks (main gate, library, convocation ground, admin, lecture hall, chapel, hostel, dining hall), 6 trees, 4 benches, 3 lamp posts, walkway strips, and 10 pedestrian stand-ins.
- `models/autobee_drone/` — quadrotor-shaped single-body drone with a 640x480 / 30 Hz / 90° FOV forward camera, IMU, and a `VelocityControl` plugin (takes `Twist` directly, no flight physics).
- `config/bridge.yaml` — ros_gz_bridge topic mappings.
- `launch/sim.launch.py` — brings up Gazebo + spawns drone + starts bridge.

## Bridged topics

| ROS 2 topic | Gazebo topic | Type |
|---|---|---|
| `/drone/camera/image_raw` | `/camera` | `sensor_msgs/Image` |
| `/drone/camera/camera_info` | `/camera_info` | `sensor_msgs/CameraInfo` |
| `/drone/imu` | `/imu` | `sensor_msgs/Imu` |
| `/drone/pose` | `/model/autobee_drone/pose` | `geometry_msgs/PoseStamped` |
| `/drone/cmd_vel` | `/model/autobee_drone/cmd_vel` | `geometry_msgs/Twist` |
| `/clock` | `/clock` | `rosgraph_msgs/Clock` |

## Running

Must be run inside the `autobee:humble` Docker container with X11 forwarded for the GUI.

### 1. Build

```bash
cd /root/autobee_ws
colcon build --packages-select autobee_sim
source install/setup.bash
```

### 2. Launch with GUI

```bash
ros2 launch autobee_sim sim.launch.py
```

Gazebo Fortress window opens with the campus world. The drone spawns at `(0, 0, 2)` and the bridge starts.

Headless (no GUI, for CI / remote debugging):

```bash
ros2 launch autobee_sim sim.launch.py headless:=true
```

### 3. Verify camera at 30 FPS (Phase 2 gate)

In a second terminal inside the container:

```bash
source /opt/ros/humble/setup.bash
source /root/autobee_ws/install/setup.bash
ros2 topic hz /drone/camera/image_raw
```

Expected: `average rate: 30.000` (±1 FPS). On CPU-only hosts without a GPU you may see 20–30 FPS — acceptable per PRD NFR-02 concession.

### 4. Teleoperate the drone

Second terminal:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args --remap cmd_vel:=/drone/cmd_vel
```

Keys: `i/,` forward/back, `j/l` yaw, `t/b` up/down. The drone should drift in response (it's a kinematic platform, not aerodynamic).

### 5. View the live camera

```bash
ros2 run rqt_image_view rqt_image_view /drone/camera/image_raw
```

## Known limitations (Phase 2 scope)

- **Pedestrians are static coloured boxes**, not animated `<actor>` meshes. Upgrading requires pre-downloading Gazebo Fuel meshes into `~/.ignition/fuel/` (offline-safe) and re-enabling the commented `<actor>` blocks.
- **No flight physics.** Drone is velocity-controlled directly. A proper `MulticopterVelocityControl` is a Phase 4 concern.
- **No indoor corridor yet** — will be added in Phase 2.5 for ORB-SLAM3 GPS-denied testing.
