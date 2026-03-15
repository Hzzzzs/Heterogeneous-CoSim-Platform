import time
import socket
import csv
import numpy as np
import pybullet as p
from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
from gym_pybullet_drones.utils.enums import DroneModel, Physics

# ============================
# 1. 网络配置
# ============================
UDP_IP = "172.21.96.1"  # 请确认这是你的 WSL 网卡 IP
UDP_PORT = 5005
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# ============================
# 2. 初始化
# ============================
env = CtrlAviary(drone_model=DroneModel.CF2X, 
                 initial_xyzs=np.array([[0, 0, 1.0]]), # PyBullet Z=1 (高度)
                 physics=Physics.PYB,
                 gui=False) 

ctrl = DSLPIDControl(drone_model=DroneModel.CF2X)

# ============================
# 3. CSV 设置
# ============================
csv_file = open('sim_data.csv', 'w', newline='')
writer = csv.writer(csv_file)
writer.writerow(["Time", "Sim_X", "Sim_Y", "Sim_Z", "Sim_Roll", "Sim_Pitch", "Sim_Yaw"])

print("🚀 仿真开始！(已恢复原始协议: x,y,z,r,p,y)")

# ============================
# 4. 主循环
# ============================
try:
    for i in range(2000):
        # 生成轨迹 (PyBullet坐标系: Z是高度)
        t = i / 240.0
        target_pos = np.array([0.5 * np.cos(t), 0.5 * np.sin(t), 1.0])
        target_rpy = np.array([0, 0, 0])
        
        # 物理步进
        obs = env._getDroneStateVector(0)
        action, _, _ = ctrl.computeControlFromState(env.CTRL_TIMESTEP, obs, target_pos, target_rpy)
        env.step(np.array([action]))
        
        # 获取当前数据
        pos = obs[0:3] 
        rpy = obs[7:10]

        # 记录 CSV
        writer.writerow([t, pos[0], pos[1], pos[2], rpy[0], rpy[1], rpy[2]])

        # === 【关键修正】 ===
        # 恢复原始协议，不带 ID，直接发坐标
        # Unity 接收端逻辑: PyBullet X -> Unity Z, PyBullet Y -> Unity -X, PyBullet Z -> Unity Y
        msg = f"{pos[0]:.4f},{pos[1]:.4f},{pos[2]:.4f},{rpy[0]:.4f},{rpy[1]:.4f},{rpy[2]:.4f}"
        sock.sendto(msg.encode(), (UDP_IP, UDP_PORT))
        
        if i % 2 == 0:
            time.sleep(1/240.0 * 2)

except KeyboardInterrupt:
    print("\n🛑 停止")
finally:
    env.close()
    sock.close()
    csv_file.close()
    print("💾 数据已保存")