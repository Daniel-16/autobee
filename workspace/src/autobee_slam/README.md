# autobee_slam

Monocular ORB-SLAM3 wrapper that publishes `/drone/pose` for the rest of the
AutoBee stack. A thin (~140 line) C++ shim against the locally-built
ORB-SLAM3 library — chosen over vendoring a community wrapper so the
integration is debuggable and version-locked to *our* ORB-SLAM3 build.

## Why monocular and not monocular-inertial?

The dissertation milestone needs *some* pose feedback so the executor can
close the loop on RTH and so PRD §11 acceptance scenarios that depend on
position can pass. Monocular gets us there with one sensor stream and far
less tuning surface than mono-inertial. Mono-inertial is the upgrade path
once the camera+IMU pipeline is validated end-to-end.

## Build prerequisites

These all happen inside `autobee_dev`:

1. **ORB-SLAM3 library built** at `/opt/autobee/third_party/ORB_SLAM3`:
   ```bash
   bash scripts/build_orbslam3.sh
   ```
   This is the same script that's been there since Phase 1 — it produces
   `lib/libORB_SLAM3.so` plus the DBoW2 / g2o thirdparty libs.

2. **Vocabulary file** (~145 MB) extracted next to it:
   ```bash
   bash workspace/src/autobee_slam/scripts/fetch_vocab.sh
   ```
   Idempotent — does nothing if `ORBvoc.txt` already exists.

3. **Pangolin** + **OpenCV** + **Eigen3** must be findable by CMake. The
   ORB-SLAM3 build script installs Pangolin to `/usr/local`; OpenCV/Eigen3
   are already in the base image.

## Build

```bash
cd /root/autobee_ws
colcon build --packages-select autobee_slam --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

If CMake can't find `libORB_SLAM3.so`, override the path:
```bash
colcon build --packages-select autobee_slam \\
  --cmake-args -DORB_SLAM3_ROOT=/path/to/ORB_SLAM3
```

## Run

Standalone:
```bash
ros2 launch autobee_slam orb_slam3.launch.py
```

Wired into the full demo via the `slam:=true` flag:
```bash
ros2 launch autobee_core demo.launch.py slam:=true
```

## What you'll likely have to debug on first run

(Honest list — this is a C++ link-against-an-external-library setup, those
never work first try.)

| Symptom | Likely cause | Fix |
|---|---|---|
| `cannot open shared object file: libORB_SLAM3.so` | rpath wasn't honoured | `export LD_LIBRARY_PATH=/opt/autobee/third_party/ORB_SLAM3/lib:/opt/autobee/third_party/ORB_SLAM3/Thirdparty/DBoW2/lib:/opt/autobee/third_party/ORB_SLAM3/Thirdparty/g2o/lib:$LD_LIBRARY_PATH` |
| Segfault on first frame | Vocab path wrong / not extracted | Re-run `fetch_vocab.sh`; verify file size ~145 MB |
| `Failed to load image. Camera Calibration parameters do not look ok.` | Settings YAML format mismatch (ORB-SLAM3 v3 vs v2 syntax) | The shipped `autobee_camera_sim.yaml` uses v3 syntax (`Camera1.fx` etc); confirm your ORB-SLAM3 build is from the master branch (UZ-SLAMLab/ORB_SLAM3) |
| Tracking never initialises | Camera pitched too far down, not enough features in view | Increase altitude in `autostart_mission`, or temporarily un-pitch the sim camera in `autobee_drone/model.sdf` to get past initialisation |
| `cv_bridge: encoding ... not supported` | Image topic encoding != `bgr8` | Set `image_encoding:=mono8` (or whatever `ros2 topic echo /drone/camera/image_raw --field encoding` shows) |
| OpenCV header version mismatch at link time | OpenCV inside ORB-SLAM3 != colcon's OpenCV | Build ORB-SLAM3 against the same OpenCV the ros-humble base image ships (4.5.x). The current `build_orbslam3.sh` already does this. |

## Topic contract

- **In**: `/drone/camera/image_raw` (`sensor_msgs/Image`)
- **Out**: `/drone/pose` (`geometry_msgs/Pose`, world-from-camera)

Once `/drone/pose` is publishing reliably, the next step is to teach
`mission_executor_node` to consume it for closed-loop RTH instead of timed
open-loop dead-reckoning. That edit is small and lives in `autobee_core`,
not here.

## Real-Tello upgrade path

1. Calibrate the physical Tello camera (checkerboard) → produce a new
   YAML at `config/autobee_camera_tello.yaml` with the real intrinsics.
2. Launch with `settings_path:=.../autobee_camera_tello.yaml`.
3. **Do not** reuse the sim YAML on the real drone (PRD risk R-09).
