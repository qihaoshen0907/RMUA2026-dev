# RPG MPC -> IntelligentUAVChampionshipBase 参数映射表

本文档描述 `controller_test` 中已完成的迁移映射，目标是完全使用 `rpg_mpc` 做轨迹控制，并通过比赛接口输出 `RotorPWM`。

## 1) 状态与坐标系映射

- 输入里程计: `/eskf_odom` (`nav_msgs/Odometry`)
- 初始位姿: `/airsim_node/initial_pose` (`geometry_msgs/PoseStamped`)
- 变换:
  - 比赛侧状态常用 NED
  - MPC 控制状态统一到 FLU 局部坐标
  - 使用固定变换 `T_flu_ned = diag(1, -1, -1, 1)`
  - 局部状态定义为 `T_0_b = (T_w_0)^-1 * T_w_b` (均在 FLU 下)

MPC 状态向量 (`kStateSize=10`) 对应:

- `state[0:3]` <- 局部位置 `p_local(x,y,z)`
- `state[3:7]` <- 局部姿态四元数 `q_local(w,x,y,z)`
- `state[7:10]` <- 局部线速度 `v_local(x,y,z)`

## 2) 轨迹输入映射 (`smoothed_63.txt`)

- 文件路径参数: `~trajectory_file`，默认 `/home/bai/RUMA_mpc/smoothed_63.txt`
- 解析规则: 文件内所有数字按顺序每 3 个分组为 `(x, y, z)` waypoint
- 约束:
  - 数值总数必须是 3 的倍数
  - 至少 1 个 waypoint
- 轨迹时间参数:
  - `~trajectory_dt` (默认 `0.1`)
  - `~waypoint_stride` (默认 `1`)
- 导数构造:
  - 速度: 中心差分 `v = (p[i+1]-p[i-1])/(2*dt)`
  - 加速度: 二阶差分 `a = (p[i+1]-2p[i]+p[i-1])/(dt^2)`
  - 限幅参数: `~max_speed`, `~max_accel`

## 3) MPC 权重映射

来源: `rpg_mpc/parameters/default.yaml`，在 launch 里显式参数化:

- `Q_pos_xy` -> `~Q_pos_xy`
- `Q_pos_z` -> `~Q_pos_z`
- `Q_attitude` -> `~Q_attitude`
- `Q_velocity` -> `~Q_velocity`
- `Q_perception` -> `~Q_perception`
- `R_thrust` -> `~R_thrust`
- `R_pitchroll` -> `~R_pitchroll`
- `R_yaw` -> `~R_yaw`

构造矩阵:

- `Q = diag([Q_pos_xy, Q_pos_xy, Q_pos_z, Q_attitude*4, Q_velocity*3, Q_perception*2])`
- `R = diag([R_thrust, R_pitchroll, R_pitchroll, R_yaw])`

## 4) MPC 输入约束映射

对应 `rpg_mpc::MpcWrapper::setLimits()`:

- `~min_thrust` -> 最小总推力加速度量级
- `~max_thrust` -> 最大总推力加速度量级
- `~max_bodyrate_xy` -> 横滚/俯仰角速度限幅
- `~max_bodyrate_z` -> 偏航角速度限幅

默认值:

- `min_thrust=2.0`
- `max_thrust=20.0`
- `max_bodyrate_xy=3.0`
- `max_bodyrate_z=2.0`

## 5) 动力学参数映射（比赛机体参数）

来自比赛 README 的系统参数，已写入 launch:

- `~mass = 0.9`
- `~arm_length = 0.18`
- `~Ixx = 0.0046890742`
- `~Iyy = 0.0069312`
- `~Izz = 0.010421166`
- `~Ct = 0.00036771704516278653`
- `~Cq = 4.888486266072161e-06`
- `~Fmax_per_rotor = 12.538338804`

## 6) 控制输出映射 (MPC -> RotorPWM)

`rpg_mpc` 输出:

- `u[0]`: collective thrust (加速度量纲)
- `u[1:3]`: body rates (rad/s)

比赛发布接口:

- `/airsim_node/drone_1/rotor_pwm_cmd` (`airsim_ros/RotorPWM`)

变换流程:

1. `T_total(N) = mass * u[0]`
2. 角速度误差 `e_w = w_cmd - w_meas`
3. 力矩: `tau = I * Kp_rate * e_w`
4. 使用四旋翼混控矩阵求每个电机推力
5. `pwm_i = clamp(thrust_i / Fmax_per_rotor, min_pwm, max_pwm)`

相关参数:

- `~rate_kp_x`, `~rate_kp_y`, `~rate_kp_z`
- `~min_pwm`, `~max_pwm`

## 7) 话题/节点映射

- 控制节点: `controller_test`
- 订阅:
  - `/eskf_odom`
  - `/airsim_node/initial_pose`
- 发布:
  - `/airsim_node/drone_1/rotor_pwm_cmd`
- 服务:
  - `/airsim_node/drone_1/takeoff` (可选，`~auto_takeoff=true` 时调用)

## 8) 本次迁移涉及文件

- `basic_dev/src/controller/src/controllerTest.cpp`
- `basic_dev/src/controller/CMakeLists.txt`
- `basic_dev/src/controller/package.xml`
- `basic_dev/src/controller/launch/controller_test.launch`
- `basic_dev/src/controller/MPC_PARAMETER_MAPPING.md`

## 9) 当前最终稳定参数（与 launch 一致）

下面是当前建议并已写入 `controller_test.launch` 的参数（用于你当前轨迹与仿真环境）：

- 轨迹相关
  - `trajectory_file=//home/bai/RUMA_mpc1/trajectory_for_mpc.txt`
  - `trajectory_dt=0.2`
  - `waypoint_stride=1`
  - `trajectory_smooth_window=3`
  - `use_file_velocity=false`
  - `align_trajectory_to_start=true`
  - `nearest_search_back=80`
  - `nearest_search_ahead=220`
  - `max_speed=2.0`
  - `max_accel=2.0`

- 起飞与切换
  - `auto_takeoff=true`
  - `takeoff_wait_sec=3.0`
  - `takeoff_min_height=1.0`

- 控制与混控
  - `Fmax_per_rotor=12.538338804`
  - `min_thrust=8.8`
  - `max_thrust=14.0`
  - `max_bodyrate_xy=0.5`
  - `max_bodyrate_z=0.4`
  - `rate_kp_x=3.0`
  - `rate_kp_y=3.0`
  - `rate_kp_z=1.5`
  - `min_pwm=0.1`
  - `max_pwm=1.0`

- 运行日志
  - `enable_runtime_log=true`
  - `runtime_log_period=0.2`

