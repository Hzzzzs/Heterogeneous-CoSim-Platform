import time
import socket
import os
import numpy as np
import pybullet as p
from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
from gym_pybullet_drones.utils.enums import DroneModel, Physics

from three_similar import get_complex_laplacian_formation

# 1. 加载算法参数
M_sys, x0, num_drones = get_complex_laplacian_formation()

# 2. 网络与环境配置
UDP_IP = "172.21.96.1"
UDP_PORT = 5006  
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

init_xyzs = x0.reshape((num_drones, 3))
env = CtrlAviary(drone_model=DroneModel.CF2X,
                 num_drones=num_drones,
                 initial_xyzs=init_xyzs,
                 physics=Physics.PYB,
                 gui=False) 

ctrls = [DSLPIDControl(drone_model=DroneModel.CF2X) for i in range(num_drones)]

# 3. 仿真步进与记录参数
formation_dt = 0.01  
TOTAL_STEPS = 10000
pos_history = np.zeros((TOTAL_STEPS, num_drones, 3))
actual_steps_run = 0 

print(f"🚀 启动 {num_drones} 机编队仿真 (正在记录轨迹数据...)")

try:
    for i in range(TOTAL_STEPS):
        actual_steps_run = i + 1
        
        # 获取无人机实时状态 (obs_multi[j] 包含位姿反馈)
        obs_multi = [env._getDroneStateVector(j) for j in range(num_drones)]

        # --- 闭环反馈调节逻辑 ---
        real_x = np.zeros(num_drones * 3)
        for j in range(num_drones):
            real_x[j*3 : j*3+3] = obs_multi[j][0:3]
        
        # 外环反馈：基于实时位置计算期望速度
        velocities = M_sys @ real_x 
        targets = (real_x + formation_dt * velocities).reshape((num_drones, 3))
        target_vels = velocities.reshape((num_drones, 3)) 

        action_matrix = np.zeros((num_drones, 4))

        # 遍历各无人机下发控制指令
        for j in range(num_drones):
            target_pos = targets[j]
            target_vel = target_vels[j]  
            target_rpy = np.array([0, 0, 0]) 

            # 内环反馈：PID 控制器根据状态残差进行动力学补偿
            action, _, _ = ctrls[j].computeControlFromState(
                control_timestep=env.CTRL_TIMESTEP,
                state=obs_multi[j],
                target_pos=target_pos,
                target_rpy=target_rpy,
                target_vel=target_vel
            )
            action_matrix[j] = action

            # 记录数据
            pos = obs_multi[j][0:3]
            pos_history[i, j, :] = pos

            # UDP 发送数据至 Unity 实时渲染
            rpy = obs_multi[j][7:10]
            msg = f"{j},{pos[0]:.4f},{pos[1]:.4f},{pos[2]:.4f},{rpy[0]:.4f},{rpy[1]:.4f},{rpy[2]:.4f}"
            sock.sendto(msg.encode(), (UDP_IP, UDP_PORT))

        env.step(action_matrix)
        
        # 维持仿真步长稳定性
        if i % 2 == 0: 
            time.sleep(1/240.0 * 2)

except KeyboardInterrupt:
    print("\n⚠️ 收到中断信号，准备保存数据并关闭...")

finally:
    # 🌟 数据持久化逻辑 (保存为 CSV)
    if actual_steps_run > 0:
        valid_pos_history = pos_history[:actual_steps_run, :, :]
        csv_data = valid_pos_history.reshape(actual_steps_run, num_drones * 3)
        save_path = os.path.join(os.getcwd(), "drone_trajectories.csv")
        np.savetxt(save_path, csv_data, delimiter=",", fmt="%.4f")
        print(f"✅ 数据记录完成。共 {actual_steps_run} 步数据已存入: {save_path}")

    env.close()
    sock.close()
    print("👋 仿真环境已关闭。")