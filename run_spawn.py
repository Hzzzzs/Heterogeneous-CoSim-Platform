import time
import socket
import numpy as np
import pybullet as p
from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
from gym_pybullet_drones.utils.enums import DroneModel, Physics

# === 网络配置 ===
UDP_IP = "172.21.96.1" # 记得改成你的 WSL 宿主 IP
UDP_PORT = 5005
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# === 舰队设置 ===
num_drones = 3
# 初始位置：排成一排，高度 1米
# ID 0: [0, 0], ID 1: [0.5, 0], ID 2: [-0.5, 0]
init_xyzs = np.array([
    [0.0, 0.0, 1.0],
    [0.5, 0.5, 1.0], # 右前方一点
    [-0.5, 0.5, 1.0] # 左前方一点
])

# 初始化环境 (注意 num_drones 参数)
env = CtrlAviary(drone_model=DroneModel.CF2X,
                 num_drones=num_drones,
                 initial_xyzs=init_xyzs,
                 physics=Physics.PYB,
                 gui=False)

# 初始化控制器 (每个飞机需要一个控制器)
ctrls = [DSLPIDControl(drone_model=DroneModel.CF2X) for i in range(num_drones)]

print(f"🚀 启动 {num_drones} 机编队仿真！")

try:
    for i in range(10000): # 运行步数
        t = i / 240.0
        
        # === A. 定义队形目标 (三角形旋转) ===
        # 让整个三角形绕着中心旋转
        center_x = 0.5 * np.cos(t * 0.5)
        center_y = 0.5 * np.sin(t * 0.5)
        center_z = 1.0

        # 定义每个飞机的相对偏移 (Offset)
        # 0号在中心，1号在右，2号在左
        offsets = [
            np.array([0, 0, 0]),       # 领机
            np.array([0.3, 0.3, 0]),   # 僚机1
            np.array([-0.3, 0.3, 0])   # 僚机2
        ]
        
        # 准备动作矩阵 (num_drones, 4)
        action_matrix = np.zeros((num_drones, 4))
        
        # === B. 计算每一架飞机的控制 ===
        # 获取所有飞机状态
        obs_multi = [env._getDroneStateVector(j) for j in range(num_drones)]

        for j in range(num_drones):
            # 1. 计算当前飞机的目标
            target_pos = np.array([center_x, center_y, center_z]) + offsets[j]
            target_rpy = np.array([0, 0, 0])

            # 2. PID 计算
            action, _, _ = ctrls[j].computeControlFromState(env.CTRL_TIMESTEP,
                                                            obs_multi[j],
                                                            target_pos,
                                                            target_rpy)
            action_matrix[j] = action

            # 3. 发送 UDP 数据 (带上 ID)
            # 格式: "ID,x,y,z,r,p,y"
            pos = obs_multi[j][0:3]
            rpy = obs_multi[j][7:10]
            msg = f"{j},{pos[0]:.4f},{pos[1]:.4f},{pos[2]:.4f},{rpy[0]:.4f},{rpy[1]:.4f},{rpy[2]:.4f}"
            sock.sendto(msg.encode(), (UDP_IP, UDP_PORT))

        # === C. 物理引擎步进 ===
        env.step(action_matrix)
        
        # 稍微降频
        if i % 2 == 0: time.sleep(1/240.0 * 2)

except KeyboardInterrupt:
    pass
finally:
    env.close()
    sock.close()