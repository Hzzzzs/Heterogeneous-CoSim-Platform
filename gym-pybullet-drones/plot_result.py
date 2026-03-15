import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D # 解决可能报错的3D环境
import os

# 1. 检查 CSV 文件
file_name = "drone_trajectories.csv"
if not os.path.exists(file_name):
    print(f"❌ 找不到数据文件 {file_name}！")
    exit()

# 2. 读取 CSV 数据
print("⏳ 正在读取 CSV 数据，请稍候...")
data_2d = np.loadtxt(file_name, delimiter=",")
total_steps = data_2d.shape[0]
n = 4 # 4架飞机

print(f"📊 成功读取 CSV，共 {total_steps} 步轨迹。")

# 3. 将 (步数, 12) 的 2D 数据，重新变回 (步数, 4, 3) 的 3D 矩阵
data = data_2d.reshape(total_steps, n, 3)

# 提取 PyBullet 的原始 X, Y, Z
d1 = data[:, :, 0] # PyBullet X 
d2 = data[:, :, 1] # PyBullet Y 
d3 = data[:, :, 2] # PyBullet Z 

last_idx = total_steps - 1

# ======================================================
# 4. 画 3D 轨迹
# ======================================================
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

for j in range(n):
    # 起点
    ax.plot([d1[0, j]], [d2[0, j]], [d3[0, j]], 'kx', markersize=6, linewidth=2)

    # 轨迹线
    if (j + 1) % 2 == 1:
        ax.plot(d1[:, j], d2[:, j], d3[:, j], 'r-', linewidth=1.3)
    else:
        ax.plot(d1[:, j], d2[:, j], d3[:, j], 'b-', linewidth=1.3)

    # 终点
    if j in [0, 1]:  # 领航员
        ax.plot([d1[last_idx, j]], [d2[last_idx, j]], [d3[last_idx, j]],
                'o', markersize=6, linewidth=2, color=(0.75, 0, 0))
    else:            # 跟随者
        ax.plot([d1[last_idx, j]], [d2[last_idx, j]], [d3[last_idx, j]],
                'o', markersize=6, linewidth=2, color=(0.5, 0.5, 0.5))

    ax.text(d1[last_idx, j] + 0.05, d2[last_idx, j] + 0.05, d3[last_idx, j] + 0.05,
            str(j + 1), color='k', fontsize=12, fontweight='bold')

# 连线
pairs = [(0, 1), (0, 2), (0, 3), (1, 2), (1, 3), (2, 3)]
for i, j in pairs:
    ax.plot([d1[last_idx, i], d1[last_idx, j]],
            [d2[last_idx, i], d2[last_idx, j]],
            [d3[last_idx, i], d3[last_idx, j]],
            '-', linewidth=1.3, color=(0.5, 0.5, 0.5))

# 🌟 核心修复 1：锁定 3D 空间的 1:1:1 真实物理比例，防止画面被压扁拉长
# 找到所有数据的最大范围
max_range = np.array([d1.max()-d1.min(), d2.max()-d2.min(), d3.max()-d3.min()]).max() / 2.0
mid_x = (d1.max()+d1.min()) * 0.5
mid_y = (d2.max()+d2.min()) * 0.5
mid_z = (d3.max()+d3.min()) * 0.5
ax.set_xlim(mid_x - max_range, mid_x + max_range)
ax.set_ylim(mid_y - max_range, mid_y + max_range)
ax.set_zlim(mid_z - max_range, mid_z + max_range)

# 🌟 核心修复 2：为了防止跟 Unity 搞混，特别标注出它们在 Unity 里的方向
ax.set_xlabel('PyBullet X (Unity X)')
ax.set_ylabel('PyBullet Y (Unity Z)')
ax.set_zlabel('PyBullet Z (Unity Y)') # 记住，这张图里的高度，就是 Unity 里朝上的轴

ax.grid(True)
plt.title("Trajectory (1:1 Real Aspect Ratio)")
plt.tight_layout()
plt.show()