"""
Main Script: main_hetero_sim.py
Description: UAV-UGV 全局一致性闭环仿真 (空地联动响应版)
Author: Zishuo Huang
"""
import time
import socket
import numpy as np
import pybullet as p
import pybullet_data
from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
from gym_pybullet_drones.utils.enums import DroneModel, Physics

from tests.simple_yigou import get_heterogeneous_consensus_params
from tests.ugv_chassis_control import UGVController

num_uavs, num_ugvs, total_agents, init_pos, H, L = get_heterogeneous_consensus_params()

UDP_IP = "172.21.96.1"  # 保持你的专属通信 IP 不变
UDP_PORT = 5006  
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

uav_init = init_pos[0:num_uavs]
ugv_init = init_pos[num_uavs:total_agents]

env = CtrlAviary(drone_model=DroneModel.CF2X,
                 num_drones=num_uavs,
                 initial_xyzs=uav_init,
                 physics=Physics.PYB,
                 gui=True) 

ctrls = [DSLPIDControl(drone_model=DroneModel.CF2X) for _ in range(num_uavs)]

# 注入 UGV 并初始化控制器
p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=env.CLIENT)
ugv_ids = []
ugv_controllers = [] # 🌟 新增：存放小车控制器的列表

for i in range(num_ugvs):
    ugv_id = p.loadURDF("husky/husky.urdf", basePosition=ugv_init[i], physicsClientId=env.CLIENT)
    ugv_ids.append(ugv_id)
    # 🌟 新增：为每一辆小车绑定一个独立的动力学控制器
    ugv_controllers.append(UGVController(ugv_id, env.CLIENT))

formation_dt = 0.01
TOTAL_STEPS = 10000

print(f"🚀 启动全局空地联动仿真: 6x6 异构拓扑网已激活！")

try:
    for i in range(TOTAL_STEPS):
        
        # ==========================================================
        # 🌟 1. 状态采集：构建全局 6x3 真实位姿矩阵
        # ==========================================================
        real_x_global = np.zeros((total_agents, 3))
        
        # 采无人机
        obs_multi = [env._getDroneStateVector(j) for j in range(num_uavs)]
        for j in range(num_uavs):
            real_x_global[j] = obs_multi[j][0:3]
            
        # 采无人车
        for j in range(num_ugvs):
            pos, quat = p.getBasePositionAndOrientation(ugv_ids[j], physicsClientId=env.CLIENT)
            real_x_global[num_uavs + j] = pos

        # ==========================================================
        # 🌟 2. 大统一算法：计算全局误差与期望速度
        # 只要任意一架飞机/车被拖动，全网都会通过矩阵乘法做出响应
        # ==========================================================
        error_global = real_x_global - H
        K_global = 1.5 
        
        # 核心控制律
        V_global = -K_global * (L @ error_global)
        
        # 拆分为 UAV 和 UGV 各自的速度向量，并进行防坠机限幅
        V_uav = V_global[0:num_uavs]
        V_uav = np.clip(V_uav, -1.5, 1.5)
        
        V_ugv = V_global[num_uavs:total_agents]
        V_ugv = np.clip(V_ugv, -1.0, 1.0)
        V_ugv[:, 2] = 0.0  # 绝对限制：死死按住无人车的 Z 轴，防止它想飞上天去追无人机
        
        # ==========================================================
        # 🌟 3. 指令下发与 UDP 通信
        # ==========================================================
        targets_uav = real_x_global[0:num_uavs] + V_uav * formation_dt
        action_matrix = np.zeros((num_uavs, 4))
        
        # UAV 执行
        for j in range(num_uavs):
            action, _, _ = ctrls[j].computeControlFromState(
                control_timestep=env.CTRL_TIMESTEP,
                state=obs_multi[j],
                target_pos=targets_uav[j],
                target_vel=V_uav[j] 
            )
            action_matrix[j] = action

            pos = obs_multi[j][0:3]
            rpy = obs_multi[j][7:10]
            msg = f"UAV,{j},{pos[0]:.4f},{pos[1]:.4f},{pos[2]:.4f},{rpy[0]:.4f},{rpy[1]:.4f},{rpy[2]:.4f}"
            sock.sendto(msg.encode(), (UDP_IP, UDP_PORT))

        # --- UGV 执行 ---
        for j in range(num_ugvs):
            pos, quat = p.getBasePositionAndOrientation(ugv_ids[j], physicsClientId=env.CLIENT)
            rpy = p.getEulerFromQuaternion(quat)
            msg = f"UGV,{j},{pos[0]:.4f},{pos[1]:.4f},{pos[2]:.4f},{rpy[0]:.4f},{rpy[1]:.4f},{rpy[2]:.4f}"
            sock.sendto(msg.encode(), (UDP_IP, UDP_PORT))
            ugv_controllers[j].compute_and_apply_control(quat, V_ugv[j])

        env.step(action_matrix)
        if i % 2 == 0: 
            time.sleep(1/240.0 * 2)

except KeyboardInterrupt:
    print("\n⚠️ 仿真中止！")
finally:
    env.close()
    sock.close()