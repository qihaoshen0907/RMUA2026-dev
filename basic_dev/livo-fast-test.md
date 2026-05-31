# RMUA2026 Open-Loop Milestone Plan

> Validate FAST-LIVO2 integration before closing the loop with planning and control.
> Scope: Simulator → Adapter → FAST-LIVO2 → RViz (visual validation only).

---

## 1. Goal

Produce a **visually verified, drift-characterized FAST-LIVO2 mapping and localization stream** from the RMUA simulator before any planning or control code is written.

"Open loop" means the simulator drives the drone via a simple pre-programmed pilot script; FAST-LIVO2 consumes sensor data and builds a map; a human operator watches RViz and compares the SLAM trajectory to ground truth. There is no EGO-Planner, no MPC, and no feedback path.

> **Yes — FAST-LIVO2 is fully RViz-compatible.** It publishes `/cloud_registered` (dense world-frame point cloud), `/path` (trajectory history), and `/aft_mapped_to_init` (odometry with covariance) out of the box. The repository ships pre-made RViz configs (`M300.rviz`, `fast_livo2.rviz`).

---

## 2. Architecture for This Milestone

```
┌─────────────┐      ┌──────────────────┐      ┌──────────────┐      ┌────────┐
│  RMUA Sim   │─────▶│ sim_sensor_adapter│─────▶│  FAST-LIVO2  │─────▶│  RViz  │
│  (NED)      │      │ (NED→ENU)        │      │   (ENU)      │      │(human) │
│             │      │                  │      │  LIVO mode   │      └────────┘
└─────────────┘      └──────────────────┘      └──────────────┘
       ▲
       │
┌──────┴──────┐
│ keyboard_   │
│  teleop.py  │  ← manual velocity commands to make the drone move
└─────────────┘
```

**What is intentionally left out:**
- EGO-Planner
- MPC / controller
- Trajectory message definitions
- `fastlivo2_to_ego_bridge`
- Competition Docker auto-launch logic

---

## 3. Deliverables

1. **`sim_sensor_adapter`** node — NED→ENU conversion for LiDAR, IMU, and front-left camera.
2. **`fast_livo2_sim.yaml`** — FAST-LIVO2 config tuned for simulator extrinsics and camera intrinsics.
3. **`keyboard_teleop.py`** — manual tele-operation script (WASD + Q/E + J/L) used to fly the drone and generate motion parallax for FAST-LIVO2.
4. **RViz configuration** — either reuse `FAST-LIVO2/rviz_cfg/M300.rviz` or create `sim_open_loop.rviz` with Fixed Frame = `camera_init`.
5. **Ground-truth comparison script** — overlays `/airsim_node/drone_1/debug/pose_gt` and `/aft_mapped_to_init` in RViz or exports both to a TUM-format file for `evo_traj` analysis.

---

## 4. Simulator Sensor Specifications

The following parameters were provided for the RMUA2026 AirSim simulator and are used to configure the adapter and FAST-LIVO2 extrinsics.

| Sensor | Parameter | Value |
|--------|-----------|-------|
| **Front-left camera** | Resolution | ~~640 × 480~~ **960 × 720** *(see changelog)* |
| | Horizontal FOV | 60° |
| | Update rate | 20 Hz |
| | Position (body) | 175 mm forward, 150 mm left of centre |
| | Intrinsics (pinhole) | fx = fy ≈ **831.38**, cx = **480**, cy = **360**, zero distortion |
| **IMU** | Position | Centre of drone |
| | Update rate | 100 Hz |
| **LiDAR** | Model | Livox MID360 (emulated) |
| | Position | 50 mm above centre |
| | Update rate | 10 Hz |
| | Points per frame | ~20,000 |
| | Range | 30 m |
| | Horizontal FOV | 360° |
| | Vertical FOV | +40° / –19° |

**Coordinate convention (AirSim body frame → adapter → ENU)**  
The `sim_sensor_adapter` applies a static NED→ENU rotation to all vectors and point coordinates. Because both LiDAR points and IMU measurements are transformed into the same ENU frame, the sensor extrinsics fed to FAST-LIVO2 are expressed in that ENU-aligned body frame:

* **LiDAR → IMU:** `T = [0, 0, 0.050]`, `R = I`
* **Camera → LiDAR:** `Rcl = [1,0,0, 0,0,-1, 0,1,0]`, `Pcl = [0.150, -0.050, -0.175]`

---

## 4. Step-by-Step Execution

### 4.1 Build Workspace

Add FAST-LIVO2 + Vikit to `RMUA2026-dev/basic_dev/src/`, install Sophus, and build:

```bash
cd RMUA2026-dev/basic_dev/src/
git clone https://github.com/hku-mars/FAST-LIVO2
git clone https://github.com/xuankuzcr/rpg_vikit.git
cd ../
catkin_make --only-pkg-with-deps fast_livo
source devel/setup.bash
```

### 4.2 Write `sim_sensor_adapter`

Responsibilities:
1. Subscribe to `/airsim_node/drone_1/lidar` (PointCloud2, NED).
2. Subscribe to `/airsim_node/drone_1/imu/imu` (Imu, NED).
3. Subscribe to `/airsim_node/drone_1/front_left/Scene` (Image, NED).
4. Apply static NED→ENU rotation to point coordinates, IMU accel/gyro, and image header `frame_id`.
5. Publish to `/sim/lidar`, `/sim/imu`, `/sim/camera`.
6. Use IMU timestamp as master clock reference.

**No ring/time injection is required** because FAST-LIVO2 will use `lidar_type: 3` (standard PointCloud2 handler).

### 4.3 Write `fast_livo2_sim.yaml`

Created files:
* `src/FAST-LIVO2/config/fast_livo2_sim.yaml` — main FAST-LIVO2 parameters
* `src/FAST-LIVO2/config/camera_fast_livo2_sim.yaml` — pinhole camera model for vikit
* `src/FAST-LIVO2/launch/mapping_sim.launch` — loads both YAMLs and starts `fastlivo_mapping`

Key blocks:
- `common`: `lid_topic: /sim/lidar`, `imu_topic: /sim/imu`, `img_topic: /sim/camera`, `img_en: 1`
- `preprocess`: `lidar_type: 3` (Ouster handler, compatible with plain XYZ PointCloud2), `blind: 0.5`, `point_filter_num: 1`
- `extrin_calib`: `extrinsic_T: [0,0,0.050]`, `extrinsic_R: I`, `Rcl: [1,0,0, 0,0,-1, 0,1,0]`, `Pcl: [0.150,-0.050,-0.175]`
- `publish`: `dense_map_en: true`

> **Note:** The Ouster handler (`lidar_type: 3`) is used because FAST-LIVO2 does not ship a generic XYZ-only handler. PCL zero-initialises missing struct fields (`intensity`, `t`, `ring`, etc.) during `fromROSMsg`, so the adapter’s plain PointCloud2 works safely in the non-feature branch (default).

### 4.4 Manual Flight with `keyboard_teleop.py`

Instead of an automated pilot, use the existing `keyboard_teleop.py` to fly the drone manually. This gives FAST-LIVO2 the parallax and motion it needs to initialize and build a map.

Recommended flight pattern:
1. Press **T** (or let `auto_takeoff` run) — takeoff and hover.
2. **W** — fly forward slowly for a few seconds.
3. **Q** / **E** — climb / descend slightly.
4. **J** / **L** — yaw left / right to sweep the LiDAR/camera.
5. **S** — fly backward to return near the start.
6. **G** — land.

This pattern exercises all 6 DOF and gives FAST-LIVO2 both translation and rotation to observe.

### 4.5 Launch and Visualize

Terminal A — FAST-LIVO2 (+ RViz):
```bash
roslaunch fast_livo mapping_sim.launch   # or your custom launch file
```

> **Note:** `mapping_sim.launch` does **not** start the adapter. Run `rosrun basic_dev sim_sensor_adapter` in a separate terminal before or alongside this launch.

Terminal B — Manual flight:
```bash
rosrun basic_dev keyboard_teleop.py
```

Terminal C — RViz (already started by `mapping_sim.launch`, but if you disabled it):
```bash
rviz -d $(rospack find fast_livo)/rviz_cfg/M300.rviz
```

**In RViz, verify:**
- Fixed Frame = `camera_init`
- PointCloud2 display on `/cloud_registered` shows red points accumulating as the drone moves
- Path display on `/path` shows a cyan trajectory line
- Odometry display on `/aft_mapped_to_init` shows axes following the drone
- If camera is enabled and working, `/rgb_img` shows the republished image with tracking overlay

### 4.6 Quantitative Validation

Run the ground-truth comparison:

```bash
# Record both trajectories
rosbag record /aft_mapped_to_init /airsim_node/drone_1/debug/pose_gt -O open_loop_test.bag

# After flight, convert to TUM format and evaluate with evo
# (write a small Python script to dump nav_msgs/Odometry to TUM txt)

evo_ape tum gt.tum slam.tum --plot
```

Success criteria for this milestone:
- **Visual:** Point cloud looks structurally consistent (walls/floors are flat, no obvious shearing).
- **Trajectory:** `/path` closely follows the known flight pattern (forward, climb, sweep, return).
- **Drift:** APE (Absolute Pose Error) < 0.5 m over a 30-second open-loop flight.
- **Frame stability:** No TF jumps or `camera_init` frame flickering in RViz.

---

## 5. RViz Cheat Sheet for This Milestone

| What to check | RViz Display | Topic | Expected Observation |
|---------------|--------------|-------|---------------------|
| **Map building** | PointCloud2 | `/cloud_registered` | Red points accumulate into walls/floor as drone moves. No smearing. |
| **Trajectory** | Path | `/path` | Cyan line traces the pilot's forward-climb-sweep-return pattern. |
| **Live pose** | Odometry | `/aft_mapped_to_init` | 3D axes follow the drone. Covariance ellipsoids stay small. |
| **High-rate pose** | Odometry | `/imu_prop_odom` | Smoother, higher-frequency axes. Good sanity check for timing. |
| **Visual tracking** | Image | `/rgb_img` | If LIVO is active, image shows tracked patches or republished frames. |
| **Ground truth** | Odometry | `/airsim_node/drone_1/debug/pose_gt` | Overlay axes for direct visual comparison (remap frame_id to `camera_init` first). |

**Pro tip:** Add a `tf` display and verify the tree is `camera_init → body → livox_frame` with no breaks.

---

## 6. If It Does Not Work

| Symptom | Likely Cause | Fix |
|---------|--------------|-----|
| RViz shows nothing, no point cloud | Adapter not publishing / FAST-LIVO2 not subscribing | `rostopic hz /sim/lidar` and `rostopic hz /cloud_registered` |
| Point cloud smears / trails behind | NED→ENU not applied to LiDAR points | Check adapter rotation matrix |
| FAST-LIVO2 crashes on startup | Wrong `lidar_type` or missing camera intrinsics | Verify `lidar_type: 3` and all `Rcl`/`Pcl`/`fx`/`fy` values |
| Trajectory drifts rapidly | IMU NED→ENU not applied | Check accel/gyro signs in adapter |
| Image topic kills performance | LIVO mode consuming large uncompressed images | Reduce simulator camera resolution or switch to `img_en: 0` for LIO-only sanity check |
| `camera_init` TF missing | FAST-LIVO2 not initialized (waiting for sync) | Ensure LiDAR, IMU, and image timestamps are within sync window |

---

## 7. Exit Criteria

This milestone is complete when you can:
1. Launch the simulator, adapter, FAST-LIVO2, and `keyboard_teleop.py`.
2. Fly the drone manually and watch a coherent 3D point cloud map build in RViz in real time.
3. Compare the FAST-LIVO2 trajectory to ground truth and confirm drift is within acceptable bounds.
4. Kill everything cleanly with Ctrl+C and have a recorded bag for replay.

**Once this milestone passes**, the next step is to freeze the adapter and FAST-LIVO2 YAML, then write the `fastlivo2_to_ego_bridge` and resume the closed-loop plan.

---

---

## Changelog

### 2026-05-31 — Camera resolution correction
- **Discovered:** The AirSim `settings.json` (`simulator_12.0.0.4`) defines `front_left` camera as **960 × 720**, not 640 × 480.
- **Updated:** `camera_fast_livo2_sim.yaml` changed from 640×480/554.26 to **960×720/831.38**.
- **Updated:** `vio.cpp` resize logic hardened to print a warning and resize to expected dimensions instead of throwing an uncaught exception.
- **Impact:** `mapping_sim.launch` no longer crashes on first VIO frame with `Frame: provided image has not the same size as the camera model`.

*Last updated: 2026-05-31*
