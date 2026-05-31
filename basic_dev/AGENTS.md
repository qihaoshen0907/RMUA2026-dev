# AGENTS.md — AI Coding Agent Guide

> This file documents the architecture, build system, and conventions of the
> `RMUA2026-dev/basic_dev` workspace. It is intended for AI coding agents that
> need to read, modify, or extend the codebase.

---

## 1. Project Overview

This is a **ROS Noetic** catkin workspace for the **RoboMaster AI Challenge
(RMUA2026)**. The goal is autonomous drone navigation inside the **Microsoft
AirSim** simulator. The workspace contains:

* **Custom competition packages** — sensor fusion, state estimation, control,
  and high-level mission logic.
* **FAST-LIVO2** — an external, tightly-coupled LiDAR-Inertial-Visual
  Odometry (LIVO) SLAM system (T-RO '24).
* **rpg_vikit** — a vision toolkit (camera models, math utilities) required
  by FAST-LIVO2.

The simulator publishes sensor data (pose, GPS, IMU, stereo images, LiDAR) on
`/airsim_node/drone_1/*` topics. The competition code consumes these topics,
runs estimation / planning / control algorithms, and publishes velocity
commands back to the simulator.

> **Note:** Code comments are frequently written in **Chinese**; English is
> used for markdown documentation and some inline comments.

---

## 2. Technology Stack

| Layer | Technology |
|-------|------------|
| OS / ROS | Ubuntu 20.04, ROS Noetic |
| Build tool | `catkin_make` |
| C++ standard | C++17 (`controller_test`, `odometry`, `imu_gps_odometry`, `fast_livo`) |
| Python | Python 3 (utility scripts) |
| Linear algebra | Eigen3 |
| Computer vision | OpenCV 4.10.0 (exact version required by `controller_test` and `odometry`) |
| Point clouds | PCL (via `pcl_ros`, `pcl_conversions`) |
| SLAM backend | g2o (`odometry` package) |
| Lie algebra | Sophus (required by FAST-LIVO2 / vikit) |
| Container | Docker (`Dockerfile` present, base image `osrf/ros:noetic-desktop-full-focal`) |

---

## 3. Workspace Layout

```
RMUA2026-dev/basic_dev/
├── src/
│   ├── airsim_ros/          # Custom ROS msg/srv definitions for AirSim bridge
│   ├── basic_dev/           # Main mission-orchestration node
│   ├── controller/          # Cascaded PD controller + test node
│   ├── imu_gps_odometry/    # ESKF-based IMU+GPS state estimator
│   ├── odometry/            # Stereo visual odometry + keyframe map manager
│   ├── FAST-LIVO2/          # fast_livo package (LIVO SLAM)
│   ├── rpg_vikit/           # vikit_common, vikit_ros, vikit_py
│   └── CMakeLists.txt       # Catkin toplevel (symlink to ROS install)
├── build/                   # catkin_make build artifacts
├── devel/                   # catkin_make devel space
├── Dockerfile               # Container definition for competition deployment
├── run_basic_dev.sh         # Docker run helper (host networking)
├── setup.bash               # Entrypoint: sources devel/setup.bash, runs basic_dev node
├── .vscode/settings.json    # VS Code: ros.distro = noetic
└── livo-fast-test.md        # Open-loop milestone plan for FAST-LIVO2 integration
```

---

## 4. Package Details

### 4.1 `airsim_ros`
* **Role:** Defines the ROS interface to the AirSim simulator.
* **Messages:** `VelCmd` (body-frame velocity command), `RotorPWM`, `CarState`,
  `CarControls`, `GimbalAngleEulerCmd`, `GimbalAngleQuatCmd`, `GPSYaw`,
  `Altimeter`, `Environment`, `PoseCmd`, `VelCmdGroup`.
* **Services:** `Takeoff`, `TakeoffGroup`, `Land`, `LandGroup`, `Reset`,
  `SetGPSPosition`, `SetLocalPosition`, `DebugSphere`, `TriggerPort`.
* **Build note:** This package must be built **first** (it is a dependency of
  almost every other package). The Dockerfile does exactly this with
  `catkin_make --only-pkg-with-deps airsim_ros`.

### 4.2 `basic_dev`
* **Role:** High-level mission logic and state machine for the competition.
* **Executables:**
  * `basic_dev` — main C++ node (`src/basic_dev.cpp`). Subscribes to
    `/airsim_node/drone_1/debug/pose_gt`, `/airsim_node/drone_1/gps`, etc.
    Publishes `airsim_ros/VelCmd` to `/airsim_node/drone_1/vel_body_cmd`.
    Uses services `Takeoff`, `Land`, `Reset`.
  * `keyboard_teleop.py` — manual tele-operation script (WASD + Q/E + J/L).
* **Dependencies:** `airsim_ros`, `roscpp`, `cv_bridge`, `pcl_ros`, `tf2*`,
  `image_transport`, `OpenCV`.

### 4.3 `controller_test`
* **Role:** Low-level flight controller.
* **Executables:**
  * `controller_test` — reads spline waypoints from `src/Splines.txt`, runs
    the cascaded PD controller, and publishes motor commands.
* **Key class:** `UAVLinearController` (`include/PDcontroller.hpp` /
  `src/PDcontroller.cpp`). Implements cascaded PD loops for position →
  velocity → attitude → PWM.
* **Launch file:** `launch/controller_test.launch` (also starts
  `imu_gps_odometry`).

### 4.4 `imu_gps_odometry`
* **Role:** IMU-GPS sensor fusion using an Error-State Kalman Filter (ESKF).
* **Executables:**
  * `imu_gps_odometry` — ROS wrapper (`src/imu_gps_odometry.cpp`) + ESKF
    core (`src/eskf.cpp`).
* **Topics:**
  * Subscribes: `/airsim_node/drone_1/gps`, `/airsim_node/drone_1/imu/imu`,
    `/airsim_node/initial_pose`
  * Publishes: `/eskf_odom` (`nav_msgs/Odometry`)
* **Caveat:** `CMakeLists.txt` sets `UTILS_PATH = ${PROJECT_SOURCE_DIR}/../utils`
  and adds it to `include_directories`. That directory **does not exist** in
  the current tree; removing or populating it may be necessary if the build
  fails.

### 4.5 `odometry`
* **Role:** Stereo visual odometry with keyframe management and g2o bundle
  adjustment.
* **Library:** `map_manager` (`src/mapmanager.cpp`, `src/keyframe.cpp`).
* **Executable:** `odometry` — subscribes to stereo images
  (`front_left/Scene`, `front_right/Scene`), pose, and IMU; runs CUDA-based
  stereo SGM (`cv::cuda::StereoSGM`) and maintains a sliding-window g2o
  optimizer.
* **Caveat:** Same `../utils` include-path issue as `imu_gps_odometry`.
  Also requires `g2o` and OpenCV compiled with CUDA modules (uses
  `cv::cuda::StereoSGM`).
* **Launch file:** `launch/odometry.launch`

### 4.6 `fast_livo` (FAST-LIVO2)
* **Role:** State-of-the-art LIVO SLAM for drift-characterized mapping and
  localization.
* **Executable:** `fastlivo_mapping` — main node (`src/main.cpp`).
* **Key source files:** `LIVMapper.cpp`, `vio.cpp`, `voxel_map.cpp`,
  `preprocess.cpp`, `IMU_Processing.cpp`.
* **Config / Launch:**
  * `config/*.yaml` — sensor extrinsics, IMU noise, LiDAR type, VIO params.
  * `launch/mapping_avia.launch`, `mapping_hesaixt32_hilti22.launch`,
    `mapping_ouster_ntu.launch`, `mapping_avia_marslvig.launch`.
* **RViz configs:** `rviz_cfg/fast_livo2.rviz`, `M300.rviz`, `hilti.rviz`,
  `ntu_viral.rviz`.
* **Dependencies:** `vikit_common`, `vikit_ros`, `Sophus`, `PCL`, `Eigen3`,
  `OpenCV`, `Boost::thread`.

### 4.7 `rpg_vikit`
* **Role:** Camera models and vision math utilities for FAST-LIVO2.
* **Sub-packages:** `vikit_common`, `vikit_ros`, `vikit_py`.
* **Build note:** `vikit_common` uses CMake 2.8.3 and `-std=c++0x` / `-std=c++11`.
  It compiles with `-O3` and SSE/AVX flags on x86_64.

---

## 5. Build Instructions

### 5.1 Native build (requires ROS Noetic + all system deps)

```bash
cd /path/to/RMUA2026-dev/basic_dev
source /opt/ros/noetic/setup.bash

# Step 1: Build message package first
catkin_make --only-pkg-with-deps airsim_ros
source devel/setup.bash

# Step 2: Build everything else
catkin_make
source devel/setup.bash
```

### 5.2 Docker build

```bash
docker build -t basic_dev .
docker run -it --net host --name basic_dev --rm basic_dev
```

The Dockerfile:
1. Installs ROS Noetic desktop-full.
2. Installs additional APT packages (`ros-noetic-geographic-msgs`,
   `ros-noetic-tf2-sensor-msgs`, `ros-noetic-cv-bridge`, `libopencv-dev`, …).
3. Builds `airsim_ros` first, then `basic_dev`.

### 5.3 Known build issues

* **Hard-coded CMake paths:** `controller_test` and `odometry` set
  `cv_bridge_DIR`, `pcl_conversions_DIR`, `pcl_ros_DIR` to
  `/usr/local/share/...`. If your system installed these via APT, the paths
  are `/usr/share/...` — you may need to adjust or remove those lines.
* **OpenCV exact version:** `controller_test` and `odometry` require
  `find_package(OpenCV 4.10.0 EXACT)`. If a different version is installed,
  CMake will fail unless the constraint is relaxed.
* **Missing `../utils`:** `imu_gps_odometry` and `odometry` add
  `${PROJECT_SOURCE_DIR}/../utils` to `include_directories`. This directory
  does not exist. The build may succeed (it is only an include path), but if
  headers are expected from there the build will break.
* **CUDA dependency:** `odometry` uses `cv::cuda::StereoSGM`. Building this
  package requires OpenCV compiled with CUDA support.

---

## 6. Running the System

### 6.1 Manual tele-operation (quick sanity check)

```bash
# Terminal 1 — start the AirSim ROS wrapper (outside this workspace)
# Terminal 2 — run the keyboard teleop node
rosrun basic_dev keyboard_teleop.py
```

Controls (as documented in the script):
* `W/A/S/D` — forward / left / backward / right
* `Q/E` — up / down
* `J/L` — yaw left / yaw right
* `K/SPACE` — stop
* `T` — takeoff, `G` — land

### 6.2 Controller + state estimator

```bash
roslaunch controller_test controller_test.launch
```

This starts `imu_gps_odometry` and `controller_test` together.

### 6.3 FAST-LIVO2 (open-loop test)

See `livo-fast-test.md` for the detailed milestone plan. Quick start:

```bash
# 1. Ensure a sensor adapter node is running (NED→ENU conversion).
# 2. Launch FAST-LIVO2 with a simulator-tuned YAML.
roslaunch fast_livo mapping_avia.launch
# 3. Optionally run the open-loop pilot or fly manually.
# 4. Visualize:
rviz -d $(rospack find fast_livo)/rviz_cfg/fast_livo2.rviz
```

---

## 7. Code Style & Conventions

* **Language:** C++17 for algorithm code; Python 3 for utilities.
* **Naming:**
  * Classes: `PascalCase` (e.g., `BasicDev`, `UAVLinearController`)
  * Member variables: `m_` prefix (e.g., `m_mass`, `m_Kpx`)
  * Global variables: `g_` prefix (e.g., `g_eskf_ptr`, `g_map_manager`)
  * ROS topics: snake_case with full path (e.g., `/airsim_node/drone_1/gps`)
* **Headers:** `#pragma once` is used in some packages; include guards
  (`#ifndef _FILE_HPP_`) in others.
* **Math:** Eigen3 is the default linear-algebra library. PCL is used for
  point-cloud types and conversions.
* **Comments:** A large fraction of inline comments are in **Chinese**.
  Agents should be comfortable reading Chinese technical comments or using
  translation when necessary.

---

## 8. Testing Strategy

* **No formal unit-test suite** exists for the competition packages
  (`basic_dev`, `controller_test`, `imu_gps_odometry`, `odometry`).
* `rpg_vikit/vikit_common/test/` contains a handful of standalone C++ tests
  (`test_camera.cpp`, `test_patch_score.cpp`, `test_triangulation.cpp`).
  These are **not** integrated into `catkin_make run_tests`.
* **Primary validation method:** launch the simulator + node, observe RViz,
  and compare outputs against ground-truth topics
  (`/airsim_node/drone_1/debug/pose_gt`).
* The `livo-fast-test.md` document defines quantitative success criteria
  (APE < 0.5 m over 30 s open-loop flight) for FAST-LIVO2 integration.

---

## 9. Deployment

* **Docker** is the intended deployment path for the competition.
* `run_basic_dev.sh` shows the exact Docker run flags used:
  `--net host` (required for ROS topic communication across containers).
* `setup.bash` is the container `ENTRYPOINT`; it sources the workspace and
  runs `rosrun basic_dev basic_dev`.

---

## 10. Security Considerations

* No secrets, API keys, or credentials are stored in the repository.
* `TriggerPort.srv` contains a comment: *"该接口仅供测试，正式比赛请勿使用"*
  ("This interface is for testing only; do not use in the official
  competition"). Avoid relying on it in production code.
* Docker image runs as `root` inside the container. In a hardened environment,
  consider adding an unprivileged user.

---

## 11. Useful References

| Document | Purpose |
|----------|---------|
| `src/*/FOLDER_DESCRIPTION.md` | One-page summary of each package |
| `livo-fast-test.md` | FAST-LIVO2 integration milestone plan |
| `src/FAST-LIVO2/README.md` | FAST-LIVO2 build & run instructions |
| `src/rpg_vikit/README.md` | Vikit camera-model documentation |
