# controller_test — 设计说明

## velocity 模式数据流

1. 样条 → `makeSplineMpcReference()`（MPC 参考轨迹）
2. `g_mpc->run(state, reference)` 求解 NMPC
3. **水平**：`getPredictedVelocity(node)` 的 `vx, vy`（ENU 世界系）
4. **竖直**：`|e_z| > vel_z_override_m` 或 MPC `vz` 符号与纠偏方向相反时，改用样条 `v_ref_z + Kp·e_z + Kv·ev_z`；否则用 MPC `vz`
5. 限幅 → `R^T v` → `vel_body_cmd`（`flip_body_y`）

启用后 **4 s** 参考高度锁定起飞高度，再 **3 s** 线性过渡到路径高度，避免参考 `z` 突变。

## 关键参数

| 参数 | 作用 |
|------|------|
| `Q_pos_xy`, `Q_velocity` | 水平 MPC 跟踪 |
| `Q_pos_z` | 竖直 MPC（过大易数值过激；launch 建议 ~300） |
| `vel_z_override_m` | 高度误差超此值改走路径 PD |
| `max_speed_z`, `max_mpc_vel_z` | 竖直下发限幅（建议 ≤1.5） |
| `max_altitude_above_path_m` | 高于路径过多强制下降 |

## 日志

`mpc_vel` = MPC 原始预测；`cmd_body` = 实际下发。若 `err_near.z` 为负（飞太高）但 `mpc_vel.z` 仍为正，会触发 z 覆盖。
