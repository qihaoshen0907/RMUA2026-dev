# 与模拟器 VelCmdmsg 目录下定义一致（见 IntelligentUAVChampionshipSimulator README）
# va: xy 平面跟踪目标速度的加速度上限 (0~8 m/s^2)
# stop: 1 为急停（忽略其它量），正常控制时为 0
std_msgs/Header header
float64 vx
float64 vy
float64 vz
float64 yawRate
uint8 va
uint8 stop
