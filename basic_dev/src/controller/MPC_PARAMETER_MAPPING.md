# controller_test 当前实现说明

本文档说明当前 `controller_test` 节点的主要控制流程。环境为 ROS Noetic + AirSim，默认通过 `airsim_ros::VelCmd` 发布速度控制命令到 `/airsim_node/drone_1/vel_body_cmd`。

## 1. 启动入口

推荐启动文件：

```bash
roslaunch controller_test controller_test.launch
```

`controller_test.launch` 同时启动：

- `imu_gps_odometry`：订阅 AirSim GPS/IMU，发布 `/eskf_odom`
- `controller_test`：加载轨迹、运行 MPC、发布 body velocity command

轨迹文件使用相对路径：

```xml
$(find controller_test)/../../../trajectory_for_mpc.txt
```

对应仓库内文件：

```text
IntelligentUAVChampionshipBase/trajectory_for_mpc.txt
```

代码里也有 `resolveTrajectoryPath()`，当 `trajectory_file` 不是绝对路径时，会先按当前工作目录查找，再按 `controller_test` 包路径拼接查找，方便别人 clone 后直接运行。

## 2. 坐标系处理

`/eskf_odom` 默认按 NED 输入处理，`odom_ned_to_enu=true` 时会转换为控制内部使用的 ENU/FLU：

- position：NED world -> ENU world
- velocity：NED world -> ENU world
- orientation：NED/FRD -> ENU/FLU
- bodyrates：FRD -> FLU

当前控制逻辑内部统一按 ENU 世界系计算轨迹、参考和 MPC 输出。最终发布 `VelCmd` 前，再转换成 body velocity。

## 3. 轨迹加载与样条

轨迹文件由 `loadTrajectorySpline()` 读取，支持几种格式：

- 三行格式：第一行全 x，第二行全 y，第三行全 z
- 每行一个点：`x y z`
- 单行连续 triplet：`x1 y1 z1 x2 y2 z2 ...`

加载后会执行：

1. 如果 `align_trajectory_to_start=true`，把轨迹首点平移到局部原点。
2. 按 `waypoint_max_segment_m` 做必要 densify。
3. 按 `waypoint_stride` 抽样。
4. 按 `trajectory_smooth_window` 平滑。
5. 构建弧长参数化的 `PathSpline3D`。

控制开始时，如果 `align_path_yaw_to_body=true`，代码会根据无人机当前 yaw 和路径起点切线方向计算 `g_path_yaw_offset`，将路径整体旋转到和机体朝向一致，避免一开始出现 90 度方向冲突。

## 4. 路径进度更新

每个控制周期会调用 `updatePathProgress(current_pos)`：

1. 把当前世界位置变换到路径局部坐标。
2. 在最近搜索窗口内找最近 waypoint 和最近弧长 `nearest_s`。
3. 根据 `path_lookahead_m` 生成前视进度 `g_path_progress_s`。
4. 如果偏离路径较大，使用 `cross_slowdown_m` 缩短前视。
5. 如果偏离超过 `cross_progress_hold_m`，冻结前进目标到最近点附近，避免大偏差时继续向前“抄近道”。

日志里的 `s` 是当前路径进度，`nearest_s` 是最近投影点位置。

## 5. MPC 参考轨迹

`makeSplineMpcReference()` 根据当前路径进度生成 MPC horizon：

```cpp
arc_s = g_path_progress_s + cruise_speed * t
sample = sampleSplineWorld(origin, arc_s, cruise_speed)
```

每个参考点包含：

- `position`
- `velocity`
- `acceleration`
- `heading`

`heading` 由样条速度切线方向得到：

```cpp
heading = atan2(sample.velocity.y(), sample.velocity.x())
```

启用控制后的前几秒，代码会先锁定参考高度到起飞高度，然后再线性过渡到轨迹高度，避免刚进入控制时 z 参考突然跳变。

## 6. MPC 求解

控制循环中会调用：

```cpp
quadrotor_common::ControlCommand cmd =
    g_mpc->run(state, reference, g_mpc_params);
```

在 `control_output=velocity` 时，当前不直接发布 MPC 的 thrust/bodyrates，而是读取 MPC 预测状态中的世界系速度：

```cpp
v_mpc = g_mpc->getPredictedVelocity(vel_mpc_node_index)
```

默认 `vel_mpc_node_index=1`，表示取预测轨迹的下一拍速度。这个速度是 ENU 世界系速度。

PWM 分支仍保留。如果 `control_output=pwm`，会走 `mpcCommandToPwm()`，把 MPC thrust/bodyrates 转换为 `RotorPWM`。

## 7. MpcBodyCmdAdapter

当前 velocity 模式的核心新增层是 `MpcBodyCmdAdapter`。它负责把 MPC 世界系速度安全转换为 AirSim body velocity command。

输入：

- `DroneState`
  - `position`
  - `velocity`
  - `yaw`
  - `stamp`
- `MpcReference`
  - `target_position`
  - `target_velocity`
  - `target_yaw`
- `mpc_success`
- `mpc_velocity_world`

输出：

- `airsim_ros::VelCmd`
- 调试信息：`v_world_cmd`、`v_body_cmd`、`position_error`、`yaw_error`、`fallback_used`

### 7.1 world-frame velocity command

如果 MPC 输出有效：

```cpp
v_world_cmd =
    mpc_velocity_world
    + kp_pos_xy * xy_position_error
    + kp_pos_z  * z_position_error
    + kd_vel    * velocity_error;
```

如果 MPC 输出无效，则 fallback：

```cpp
v_world_cmd =
    reference.target_velocity
    + kp_pos_xy * xy_position_error
    + kp_pos_z  * z_position_error
    + kd_vel    * velocity_error;
```

当前默认参数：

```text
mpc_body_kp_pos_xy = 0.8
mpc_body_kp_pos_z  = 0.6
mpc_body_kd_vel    = 0.15
mpc_body_kp_yaw    = 1.8
```

### 7.2 world -> body

转换只使用当前 yaw，不使用完整姿态矩阵，避免 roll/pitch 引入重复旋转：

```cpp
vx_body =  cos(yaw) * vx_world + sin(yaw) * vy_world;
vy_body = -sin(yaw) * vx_world + cos(yaw) * vy_world;
vz_body =  vz_world;
```

变量命名保持明确：

- `v_world_cmd`：世界系速度
- `v_body_cmd`：机体系速度

如果 `vel_cmd_flip_body_y=true`，最终发布前会翻转 body y，以适配当前 RMUA/AirSim `vel_body_cmd` 方向。

### 7.3 yawRate

目标 yaw 来自路径切线方向：

```cpp
yaw_error = normalizeAngle(reference.target_yaw - current_state.yaw);
yawRate = mpc_body_kp_yaw * yaw_error;
```

再按 `body_cmd_max_yaw_rate` 限幅。

### 7.4 动态限速

基础限幅：

```text
body_cmd_max_vx = 10.0
body_cmd_max_vy = 4.0
body_cmd_max_vz = 2.0
body_cmd_max_yaw_rate = 1.5
```

前向速度 `vx` 会根据状态动态降低：

- `|yaw_error| > 0.7` 时，`max_vx = 5.0`
- `|yaw_error| > 1.0` 时，`max_vx = 3.5`
- `xy_position_error > 8.0` 时，`max_vx` 不超过 `5.0`

这样直线段可以保持较高速度，弯道或大误差时自动降速。

### 7.5 命令平滑

adapter 保存上一帧 `v_body_cmd` 和 `yawRate`，每次发布前做 step limit：

```cpp
new_value = clamp(
    new_value,
    last_value - body_cmd_max_acc_step,
    last_value + body_cmd_max_acc_step);
```

默认 `body_cmd_max_acc_step=1.0`。

## 8. 安全保护

以下情况不会发布正常速度指令：

- 未 takeoff
- 未进入 tracking 状态
- odom 超时超过 0.2 秒
- 当前状态包含 NaN/inf
- 参考目标包含 NaN/inf
- MPC 输出包含 NaN/inf

一般未 ready 时发布 0 速度：

```text
vx=0, vy=0, vz=0, yawRate=0, va=0, stop=0
```

如果状态或参考严重不可用，发布 0 速度且 `stop=1`。

## 9. 控制循环总结

当前 `controlTimerCallback()` 的主流程：

```text
等待 odom 和 takeoff
  ↓
odomToQuadState()
  ↓
首次进入控制：设置 reference origin、路径 yaw 对齐
  ↓
makeSplineMpcReference()
  ↓
g_mpc->run(state, reference)
  ↓
velocity 模式：
    getPredictedVelocity(vel_mpc_node_index)
    构造 DroneState / MpcReference
    MpcBodyCmdAdapter.computeCommand()
    发布 /airsim_node/drone_1/vel_body_cmd

pwm 模式：
    mpcCommandToPwm()
    发布 /airsim_node/drone_1/rotor_pwm_cmd
```

## 10. 日志说明

velocity 模式每 0.2 秒打印：

```text
[MPC_BODY_CMD]
cur        当前 ENU 位置
tgt        当前 MPC 参考首点
err_xy     当前 xy 位置误差
v_world    adapter 输出的世界系速度
v_body     最终下发的 body velocity
yaw        当前 yaw
tgt_yaw    目标 yaw
yaw_err    yaw 误差
yawRate    下发 yawRate
mpc_success MPC 输出是否有效
fallback   是否走 reference velocity + feedback fallback
mpc_vel    MPC 原始预测世界系速度
nearest_s  当前最近路径弧长
```

调试时重点看：

- `err_xy` 是否持续增大
- `v_body.x` 是否在弯道过大
- `v_body.y` 是否朝正确方向拉回
- `cur.z` 和 `tgt.z` 是否持续偏离
- `fallback` 是否长期为 1

## 11. 当前需要注意的点

当前版本已经形成稳定的“MPC world velocity -> body cmd adapter -> VelCmd”结构，但实际赛道中仍建议继续调：

- 弯道降速逻辑：如果撞墙，降低 `body_cmd_max_vx` 或增加基于 `err_xy` 的降速。
- 高度通道：如果 z 过冲，降低 `body_cmd_max_vz`、`mpc_body_kp_pos_z` 或 `Q_pos_z`。
- 横向符号：如果 `v_body.y` 方向反了，检查 `vel_cmd_flip_body_y`。
