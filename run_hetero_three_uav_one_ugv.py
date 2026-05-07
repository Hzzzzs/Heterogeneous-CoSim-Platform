import time
import socket
import numpy as np
from numpy.linalg import eig, det
import pybullet as p
import pybullet_data

from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
from gym_pybullet_drones.utils.enums import DroneModel, Physics

from tests.ugv_chassis_control import UGVController


# ======================================================================
# Part 1: 使用 run_threesimilar.py 的矩阵控制律 (4 节点)
# 节点顺序固定为: [UAV_leader, UGV, UAV_follower_1, UAV_follower_2]
# ======================================================================
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
        val = e_prod * det(A[:i + 1, :i + 1]) / lambda_prod
        e[i] = epsilon * np.exp(-1j * np.angle(val))
        e_diag = np.append(e_diag, e[i])
        lambda_vals = eig(np.diag(e_diag) @ A[:i + 1, :i + 1])[0]

    e = -e

    def R_from_angle(phi):
        return np.array([
            [np.cos(phi), -np.sin(phi), 0],
            [np.sin(phi), np.cos(phi), 0],
            [0, 0, 1]
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

    # 初始化位置：注意如果你把树放在了 (0,0,0)，这几架飞机可能会直接刷在树上！
    # 建议在 Unity 里把树稍微往前挪一点，比如放在 X=5 的位置。
    x0 = np.random.rand(n * d)
    x0[0:3] = np.array([0.0, 0.0, 2.0])      # UAV leader
    x0[3:6] = np.array([1.0, 0.0, 0.08])     # UGV
    if x0[8] < 0.2:
        x0[8] = 0.6
    if x0[11] < 0.2:
        x0[11] = 0.6

    return M, x0


# ======================================================================
# Part 2: PyBullet 仿真主程序 (3 UAV + 1 UGV + 树木避障场景)
# ======================================================================
M_sys, x0 = get_dynamics_and_init()

NUM_UAVS = 3
NUM_UGVS = 1
TOTAL_AGENTS = NUM_UAVS + NUM_UGVS

IDX_UAV_LEADER = 0
IDX_UGV = 1
IDX_UAV_F1 = 2
IDX_UAV_F2 = 3

uav_init_xyzs = np.vstack([
    x0[0:3],
    x0[6:9],
    x0[9:12]
])
ugv_init_xyz = x0[3:6]

def _resolve_unity_host():
    """自动解析 Unity (Windows 主机) 的 IP，避免每次 WSL 重启后 IP 变化。
    优先级：环境变量 UNITY_HOST > mirrored 模式下的 127.0.0.1 > NAT 模式下默认网关。
    """
    import os, subprocess
    env_ip = os.environ.get("UNITY_HOST")
    if env_ip:
        return env_ip
    # 检测 WSL 网络模式
    try:
        mode = subprocess.check_output(["wslinfo", "--networking-mode"],
                                       stderr=subprocess.DEVNULL, timeout=2).decode().strip()
        if mode == "mirrored":
            return "127.0.0.1"
    except Exception:
        pass
    # NAT 模式：Windows 主机 = 默认网关
    try:
        out = subprocess.check_output(["ip", "route", "show", "default"], timeout=2).decode()
        for tok in out.split():
            if tok.count(".") == 3:
                return tok
    except Exception:
        pass
    return "127.0.0.1"


UDP_IP = _resolve_unity_host()
UDP_PORT = 5006
print(f"📡 UDP target (Unity host): {UDP_IP}:{UDP_PORT}")
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

env = CtrlAviary(
    drone_model=DroneModel.CF2X,
    num_drones=NUM_UAVS,
    initial_xyzs=uav_init_xyzs,
    physics=Physics.PYB,
    gui=True # 开启物理引擎自带的可视化界面，方便观察碰撞
)

ctrls = [DSLPIDControl(drone_model=DroneModel.CF2X) for _ in range(NUM_UAVS)]

# 加载 UGV (Husky)
p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=env.CLIENT)
ugv_id = p.loadURDF(
    "husky/husky.urdf",
    basePosition=ugv_init_xyz,
    physicsClientId=env.CLIENT
)
ugv_controller = UGVController(ugv_id, env.CLIENT)

# ======================================================================
# 🌲 核心新增：注入 Unity 导出的障碍物 (树木)
# ======================================================================
try:
    tree_obj_path = "PyBullet_Terrain.obj"
    tree_collision_id = p.createCollisionShape(shapeType=p.GEOM_MESH, fileName=tree_obj_path, meshScale=[1, 1, 1], physicsClientId=env.CLIENT)
    tree_visual_id = p.createVisualShape(shapeType=p.GEOM_MESH, fileName=tree_obj_path, meshScale=[1, 1, 1], physicsClientId=env.CLIENT)
    
    # mass=0 代表它是一个绝对静态的刚体障碍物
    tree_id = p.createMultiBody(baseMass=0,
                                baseCollisionShapeIndex=tree_collision_id,
                                baseVisualShapeIndex=tree_visual_id,
                                basePosition=[0, 0, 0], 
                                baseOrientation=[0, 0, 0, 1],
                                physicsClientId=env.CLIENT)
    print("🌲 成功加载树木障碍物模型！")
except Exception as e:
    print(f"⚠️ 加载树木模型失败，请检查 tree_obstacle.obj 是否存在！错误: {e}")
    tree_id = -1 # 防止后续找不到变量报错

formation_dt = 0.01
TOTAL_STEPS = 10000

# ======================================================================
# 🚀 核心新增：全局巡航速度前馈 (Feedforward Velocity)
# 设定编队的整体行进速度，例如沿 X 轴以 0.5 m/s 前进
# ======================================================================
base_vel = np.array([0.5, 0.0, 0.0]) 
vel_feedforward = np.tile(base_vel, TOTAL_AGENTS) # 扩展为 12 维向量

# 轨迹历史记录
pos_history = np.zeros((TOTAL_STEPS, TOTAL_AGENTS, 3))
actual_steps_run = 0

print("Starting heterogeneous formation simulation: 3 UAV + 1 UGV (with collision detection)")

try:
    for i in range(TOTAL_STEPS):
        actual_steps_run = i + 1

        # UAV physical order: [leader, follower_1, follower_2]
        obs_multi = [env._getDroneStateVector(j) for j in range(NUM_UAVS)]
        ugv_pos, ugv_quat = p.getBasePositionAndOrientation(ugv_id, physicsClientId=env.CLIENT)

        # 构建虚拟状态
        real_x_virtual = np.zeros(TOTAL_AGENTS * 3)
        real_x_virtual[IDX_UAV_LEADER * 3: IDX_UAV_LEADER * 3 + 3] = obs_multi[0][0:3]
        real_x_virtual[IDX_UGV * 3: IDX_UGV * 3 + 3] = np.array(ugv_pos)
        real_x_virtual[IDX_UAV_F1 * 3: IDX_UAV_F1 * 3 + 3] = obs_multi[1][0:3]
        real_x_virtual[IDX_UAV_F2 * 3: IDX_UAV_F2 * 3 + 3] = obs_multi[2][0:3]

        # 闭环反馈修正速度 + 期望巡航前馈速度
        velocities = M_sys @ real_x_virtual + vel_feedforward
        
        targets = (real_x_virtual + formation_dt * velocities).reshape((TOTAL_AGENTS, 3))
        target_vels = velocities.reshape((TOTAL_AGENTS, 3))

        target_pos_uav = np.vstack([targets[IDX_UAV_LEADER], targets[IDX_UAV_F1], targets[IDX_UAV_F2]])
        target_vel_uav = np.vstack([target_vels[IDX_UAV_LEADER], target_vels[IDX_UAV_F1], target_vels[IDX_UAV_F2]])
        target_vel_uav = np.clip(target_vel_uav, -1.5, 1.5)

        action_matrix = np.zeros((NUM_UAVS, 4))
        for j in range(NUM_UAVS):
            action, _, _ = ctrls[j].computeControlFromState(
                control_timestep=env.CTRL_TIMESTEP,
                state=obs_multi[j],
                target_pos=target_pos_uav[j],
                target_rpy=np.zeros(3),
                target_vel=target_vel_uav[j]
            )
            action_matrix[j] = action

            pos = obs_multi[j][0:3]
            rpy = obs_multi[j][7:10]
            pos_history[i, j, :] = pos
            msg = f"UAV,{j},{pos[0]:.4f},{pos[1]:.4f},{pos[2]:.4f},{rpy[0]:.4f},{rpy[1]:.4f},{rpy[2]:.4f}"
            sock.sendto(msg.encode(), (UDP_IP, UDP_PORT))

        # UGV 动力学指令下发
        target_vel_ugv = target_vels[IDX_UGV].copy()
        target_vel_ugv = np.clip(target_vel_ugv, -1.0, 1.0)
        target_vel_ugv[2] = 0.0
        ugv_controller.compute_and_apply_control(ugv_quat, target_vel_ugv)

        ugv_rpy = p.getEulerFromQuaternion(ugv_quat)
        pos_history[i, NUM_UAVS, :] = ugv_pos
        ugv_msg = f"UGV,0,{ugv_pos[0]:.4f},{ugv_pos[1]:.4f},{ugv_pos[2]:.4f},{ugv_rpy[0]:.4f},{ugv_rpy[1]:.4f},{ugv_rpy[2]:.4f}"
        sock.sendto(ugv_msg.encode(), (UDP_IP, UDP_PORT))

        # 步进物理引擎
        env.step(action_matrix)
        
        # ======================================================================
        # 💥 核心新增：全面碰撞检测系统 (Collision Detection)
        # ======================================================================
        collision_detected = False
        crash_reason = ""
        drone_ids = env.DRONE_IDS 

        # 1. 检查树木碰撞 (仅当树木成功加载时)
        if tree_id != -1:
            # 检查无人机撞树
            for d_id in drone_ids:
                contacts_with_tree = p.getContactPoints(bodyA=d_id, bodyB=tree_id, physicsClientId=env.CLIENT)
                if len(contacts_with_tree) > 0:
                    collision_detected = True
                    crash_reason = f"💥 炸机事故：无人机 [ID: {d_id}] 撞毁在了树上！"
                    break
            
            # 检查无人车撞树
            if not collision_detected:
                contacts_ugv_tree = p.getContactPoints(bodyA=ugv_id, bodyB=tree_id, physicsClientId=env.CLIENT)
                if len(contacts_ugv_tree) > 0:
                    collision_detected = True
                    crash_reason = f"💥 撞车事故：无人车 (UGV) 撞到了树的实体网格上！"

        # 2. 检查无人机空中相撞 (Mid-air Collision)
        if not collision_detected:
            for n in range(len(drone_ids)):
                for m in range(n + 1, len(drone_ids)):
                    contacts_between_drones = p.getContactPoints(bodyA=drone_ids[n], bodyB=drone_ids[m], physicsClientId=env.CLIENT)
                    if len(contacts_between_drones) > 0:
                        collision_detected = True
                        crash_reason = f"💥 炸机事故：无人机 [ID: {drone_ids[n]}] 与 无人机 [ID: {drone_ids[m]}] 发生空中相撞！"
                        break
                if collision_detected:
                    break

        # 如果检测到任何碰撞，立刻终止当前仿真循环
        if collision_detected:
            print(f"\n=========================================")
            print(f"🛑 仿真紧急终止 (EMERGENCY STOP)")
            print(f"{crash_reason}")
            print(f"发生时间步: {i} (约 {i * env.CTRL_TIMESTEP:.2f} 秒)")
            print(f"=========================================\n")
            break # 打断 for 循环，直接跳到 finally 保存数据

        # 同步物理与现实时间
        if i % 2 == 0:
            time.sleep(1 / 240.0 * 2)

except KeyboardInterrupt:
    print("\nSimulation interrupted by user.")
finally:
    env.close()
    sock.close()

    # 保存安全范围内的轨迹数据
    valid_pos_history = pos_history[:actual_steps_run, :, :]
    csv_data = valid_pos_history.reshape(actual_steps_run, TOTAL_AGENTS * 3)
    save_path = "hetero_trajectories_3uav_1ugv.csv"
    np.savetxt(save_path, csv_data, delimiter=",", fmt="%.4f")

    print(f"Simulation finished. Steps recorded: {actual_steps_run}")
    print(f"Trajectory saved to: {save_path}")