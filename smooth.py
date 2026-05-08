import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.signal import savgol_filter

# 1. 配置文件路径
input_filename = 'sampled_63.txt'      # 你的原始数据文件
output_filename = 'smoothed_63.txt'    # 平滑后导出的数据文件

try:
    with open(input_filename, 'r', encoding='utf-8') as f:
        # 直接读取整个文件的所有内容
        data_str = f.read()
except FileNotFoundError:
    print(f"找不到文件 {input_filename}，请检查路径。")
    exit()

# 将字符串按空格/换行分割，全部直接转换为浮点数
data = list(map(float, data_str.strip().split()))

# 按 1/3 的步长提取出所有的 X, Y, Z
x_orig = np.array(data[0::3])
y_orig = np.array(data[1::3])
z_orig = np.array(data[2::3])

total_points = len(x_orig)
print(f"成功读取数据！共载入 {total_points} 个原始 3D 坐标点。")

# 2. 轨迹平滑处理 (使用 Savitzky-Golay 滤波器)
window_length = min(total_points // 5, 51) 
if window_length % 2 == 0:  
    window_length += 1
if window_length < 3:
    window_length = 3

x_smooth = savgol_filter(x_orig, window_length, 3)
y_smooth = savgol_filter(y_orig, window_length, 3)
z_smooth = savgol_filter(z_orig, window_length, 3)

# ================= 新增：保存纯数据，无表头 =================
smoothed_points = []
for i in range(total_points):
    # 将平滑后的数据保留2位小数并拼接
    smoothed_points.append(f"{x_smooth[i]:.2f} {y_smooth[i]:.2f} {z_smooth[i]:.2f}")

# 组合字符串并直接写入新文件（没有任何多余的文字）
with open(output_filename, 'w', encoding='utf-8') as f:
    f.write(" ".join(smoothed_points))

print(f"平滑后的纯坐标已成功保存至: {output_filename}")
# ==========================================================

# 3. 开始 3D 绘图
fig = plt.figure(figsize=(12, 9))
ax = fig.add_subplot(111, projection='3d')

# 绘制原始的所有散点（灰色，半透明）
ax.plot(x_orig, y_orig, z_orig, 'o-', color='gray', markersize=2, linewidth=0.5, alpha=0.3, label='Original Raw Points')

# 绘制平滑后的飞行轨迹（蓝色实线）
ax.plot(x_smooth, y_smooth, z_smooth, '-', color='#1f77b4', linewidth=2.5, label='Smoothed Flight Path')

# 标记起点 (绿色) 和 终点 (红色)
ax.scatter(x_orig[0], y_orig[0], z_orig[0], color='green', s=150, zorder=5, label='START')
ax.scatter(x_orig[-1], y_orig[-1], z_orig[-1], color='red', s=150, zorder=5, label='END')

# 设置坐标轴标签
ax.set_xlabel('X Axis (m)')
ax.set_ylabel('Y Axis (m)')
ax.set_zlabel('Z Axis (m)')
ax.set_title(f'Drone 3D Flight Path (Total {total_points} Points)')

ax.legend()
ax.view_init(elev=20, azim=45)

# 显示图像
plt.show()