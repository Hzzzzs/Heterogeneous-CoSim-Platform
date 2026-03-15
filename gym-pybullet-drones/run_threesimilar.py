import time
import socket
import numpy as np
from numpy.linalg import eig, det
import pybullet as p
from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
from gym_pybullet_drones.utils.enums import DroneModel, Physics

# ==============================================================================
# 第一部分：从 threesimilar.py 移植过来的数学计算 (生成矩阵 M 和初始状态 x0)
# ==============================================================================
def get_dynamics_and_init():
    n = 4      
    d = 3      

    L = np.array([
        [0, 0, 0, 0],
        [0, 0, 0, 0],
        [1, 1, -3 - np.sqrt(2), 1 + np.sqrt(2)],
        [1, 1, 1, -3]
    ], dtype=float)

    theta31, theta32, theta34 = 5 * np.pi / 4, 0, 5 * np.pi / 4
    theta41, theta42, theta43 = 0, 0, np.pi / 4

    Lxy = np.zeros((n, n), dtype=complex)
    Lxy[2, 0] = L[2, 0] * np.exp(1j * theta31)
    Lxy[2, 1] = L[2, 1] * np.exp(1j * theta32)
    Lxy[2, 3] = L[2, 3] * np.exp(1j * theta34)
    Lxy[2, 2] = -(Lxy[2, 0] + Lxy[2, 1] + Lxy[2, 3])
    
    Lxy[3, 0] = L[3, 0] * np.exp(1j * theta41)
    Lxy[3, 1] = L[3, 1] * np.exp(1j * theta42)
    Lxy[3, 2] = L[3, 2] * np.exp(1j * theta43)
    Lxy[3, 3] = -(Lxy[3, 0] + Lxy[3, 1] + Lxy[3, 2])

    A = Lxy[2:4, 2:4]
    m_dim = 2
    e = np.zeros(m_dim, dtype=complex)
    e[0] = 1.0 / det(A[0:1, 0:1])
    lambda_vals = eig(np.diag([e[0]]) @ A[0:1, 0:1])[0]
    e_diag = np.array([e[0]], dtype=complex)
    epsilon = 0.1
    for i in range(1, m_dim):
        e_prod = np.prod(e[:i])
        lambda_prod = np.prod(lambda_vals[:i])
        val = e_prod * det(A[:i+1, :i+1]) / lambda_prod
        e[i] = epsilon * np.exp(-1j * np.angle(val))
        e_diag = np.append(e_diag, e[i])
        lambda_vals = eig(np.diag(e_diag) @ A[:i+1, :i+1])[0]
    
    e = -e 

    def R_from_angle(phi):
        return np.array([
            [np.cos(phi), -np.sin(phi), 0],
            [np.sin(phi),  np.cos(phi), 0],
            [0,           0,           1]
        ])

    R1 = R_from_angle(np.angle(e[0]))
    R2 = R_from_angle(np.angle(e[1]))
    
    D = np.zeros((12, 12), dtype=float)
    D[:6, :6] = 0.1 * np.eye(6)
    D[6:9, 6:9] = np.abs(e[0]) * R1
    D[9:12, 9:12] = np.abs(e[1]) * R2

    R31, R32, R34 = R_from_angle(theta31), R_from_angle(theta32), R_from_angle(theta34)
    R41, R42, R43 = R_from_angle(theta41), R_from_angle(theta42), R_from_angle(theta43)

    LL = np.zeros((12, 12), dtype=float)
    LL[6:9, 0:3] = L[2, 0] * R31
    LL[6:9, 3:6] = L[2, 1] * R32
    LL[6:9, 6:9] = -(L[2, 0] * R31 + L[2, 1] * R32 + L[2, 3] * R34)
    LL[6:9, 9:12] = L[2, 3] * R34
    
    LL[9:12, 0:3] = L[3, 0] * R41
    LL[9:12, 3:6] = L[3, 1] * R42
    LL[9:12, 6:9] = L[3, 2] * R43
    LL[9:12, 9:12] = -(L[3, 0] * R41 + L[3, 1] * R42 + L[3, 2] * R43)

    M = D @ LL  

    scaling = 1.0
    thetaleader = 0.0
    Rleader = np.array([
        [np.cos(thetaleader),  np.sin(thetaleader), 0],
        [-np.sin(thetaleader), np.cos(thetaleader), 0],
        [0,                    0,                   1]
    ])
    trans = np.array([0, 0, 0]) 
    
    x0 = np.random.rand(n * d) 
    x0[0:3] = scaling * (Rleader @ np.array([0, 0, 3])) + trans
    x0[3:6] = scaling * (Rleader @ np.array([1, 0, 1])) + trans
    
    if x0[8] < 0.2: x0[8] = 0.5
    if x0[11] < 0.2: x0[11] = 0.5

    return M, x0, n

# ==============================================================================
# 第二部分：PyBullet 仿真主程序 (闭环 + 速度前馈 + 数据记录)
# ==============================================================================

M_sys, x0, num_drones = get_dynamics_and_init()

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

formation_dt = 0.01  
TOTAL_STEPS = 10000

# 🌟 新增：预先分配一个大矩阵用于记录数据，形状为 [最大步数, 无人机数量, 3维度]
pos_history = np.zeros((TOTAL_STEPS, num_drones, 3))
actual_steps_run = 0 # 记录实际跑了多少步

print(f"🚀 启动 {num_drones} 机编队仿真 (正在记录轨迹数据...)")

try:
    for i in range(TOTAL_STEPS):
        actual_steps_run = i + 1 # 更新实际运行步数
        
        obs_multi = [env._getDroneStateVector(j) for j in range(num_drones)]

        # 闭环：实机位置算目标
        real_x = np.zeros(num_drones * 3)
        for j in range(num_drones):
            real_x[j*3 : j*3+3] = obs_multi[j][0:3]
        
        velocities = M_sys @ real_x 
        targets = (real_x + formation_dt * velocities).reshape((num_drones, 3))
        target_vels = velocities.reshape((num_drones, 3)) 

        action_matrix = np.zeros((num_drones, 4))

        for j in range(num_drones):
            target_pos = targets[j]
            target_vel = target_vels[j]  
            target_rpy = np.array([0, 0, 0]) 

            action, _, _ = ctrls[j].computeControlFromState(
                control_timestep=env.CTRL_TIMESTEP,
                state=obs_multi[j],
                target_pos=target_pos,
                target_rpy=target_rpy,
                target_vel=target_vel
            )
            action_matrix[j] = action

            pos = obs_multi[j][0:3]
            
            # 🌟 新增：把当前的真实物理坐标写入历史记录本
            pos_history[i, j, :] = pos

            rpy = obs_multi[j][7:10]
            msg = f"{j},{pos[0]:.4f},{pos[1]:.4f},{pos[2]:.4f},{rpy[0]:.4f},{rpy[1]:.4f},{rpy[2]:.4f}"
            sock.sendto(msg.encode(), (UDP_IP, UDP_PORT))

        env.step(action_matrix)
        
        if i % 2 == 0: 
            time.sleep(1/240.0 * 2)

except KeyboardInterrupt:
    print("\n⚠️ 收到中断信号，提前结束仿真！")
finally:
    env.close()
    sock.close()
    
    # 🌟 新增：数据保存逻辑
    # 截取实际跑过的数据（丢弃后面全是 0 的空白部分）
    valid_pos_history = pos_history[:actual_steps_run, :, :]
    
# === 🌟 替换为保存 CSV 的代码 ===
    # 我们把 3D 矩阵 (步数, 4, 3) 压平为 2D 矩阵 (步数, 12) 以便存入 CSV
    # 每一行的 12 个数字代表：飞机1的x,y,z, 飞机2的x,y,z, 飞机3的x,y,z, 飞机4的x,y,z
    csv_data = valid_pos_history.reshape(actual_steps_run, num_drones * 3)
    save_path = "drone_trajectories.csv"
    
    # 存入 CSV，保留 4 位小数
    np.savetxt(save_path, csv_data, delimiter=",", fmt="%.4f")
    
    print(f"✅ 仿真停止。共记录了 {actual_steps_run} 步的轨迹数据。")
    print(f"✅ 数据已成功保存为 CSV 文件：{save_path}")