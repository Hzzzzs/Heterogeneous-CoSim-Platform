"""Heterogeneous formation: 3 UAV + 1 UGV (Ackermann racecar).

This script is identical in formation control logic to
run_hetero_three_uav_one_ugv.py, the ONLY difference is the UGV model
and its low-level controller:

    Husky differential drive   ->   PyBullet racecar (Ackermann steering)

Racecar joint layout (pybullet_data/racecar/racecar.urdf):
    2: left_rear_wheel_joint        (drive)
    3: right_rear_wheel_joint       (drive)
    4: left_steering_hinge_joint    (steer)
    5: left_front_wheel_joint       (passive)
    6: right_steering_hinge_joint   (steer)
    7: right_front_wheel_joint      (passive)

Everything else (matrix law, base_vel feedforward, UAV PID, collision logic,
Unity UDP, CSV save) stays unchanged from the Husky version.
"""

import time
import socket
import os
import numpy as np
from numpy.linalg import eig, det
import pybullet as p
import pybullet_data

from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
from gym_pybullet_drones.utils.enums import DroneModel, Physics


# ======================================================================
# Ackermann UGV controller (inline, racecar-specific)
# ======================================================================
class RacecarAckermannController:
    """World-frame velocity command -> Ackermann steering + rear-wheel speed.

    Approximates a bicycle model:
        target_yaw   = atan2(vy, vx)
        yaw_error    = wrap(target_yaw - current_yaw)
        delta        = clip(Kp_steer * yaw_error, -delta_max, +delta_max)
        v_target     = ||(vx, vy)|| * cos(yaw_error)
        omega_wheel  = v_target / wheel_radius

    delta is a single bicycle-equivalent steering angle; both front hinges are
    set to the same value (PyBullet's racecar URDF lacks a steering linkage,
    so this is a fair single-track approximation).
    """

    DRIVE_JOINT_NAMES = ("left_rear_wheel_joint", "right_rear_wheel_joint")
    STEER_JOINT_NAMES = ("left_steering_hinge_joint", "right_steering_hinge_joint")

    def __init__(self, ugv_id, physics_client,
                 wheel_radius=0.05,
                 delta_max=1.0,           # ≈ 57° max steering (was 0.6)
                 Kp_steer=5.0,            # was 2.0
                 max_drive_force=20.0,
                 max_steer_force=30.0):   # was 10.0
        self.ugv_id = ugv_id
        self.CLIENT = physics_client
        self.wheel_radius   = wheel_radius
        self.delta_max      = delta_max
        self.Kp_steer       = Kp_steer
        self.max_drive_force = max_drive_force
        self.max_steer_force = max_steer_force

        self.drive_joints = []
        self.steer_joints = []
        for j in range(p.getNumJoints(ugv_id, physicsClientId=self.CLIENT)):
            name = p.getJointInfo(ugv_id, j, physicsClientId=self.CLIENT)[1].decode()
            if name in self.DRIVE_JOINT_NAMES:
                self.drive_joints.append(j)
            elif name in self.STEER_JOINT_NAMES:
                self.steer_joints.append(j)
        if len(self.drive_joints) != 2 or len(self.steer_joints) != 2:
            raise RuntimeError(
                f"racecar joints not found as expected — drive={self.drive_joints}, "
                f"steer={self.steer_joints}")

    def compute_and_apply_control(self, current_quat, target_vel_world):
        rpy = p.getEulerFromQuaternion(current_quat)
        yaw = rpy[2]

        vx, vy = target_vel_world[0], target_vel_world[1]
        speed = float(np.hypot(vx, vy))

        # Below this threshold treat as stop — release drive, hold steering centered
        if speed < 0.05:
            self._drive(0.0)
            self._steer(0.0)
            return

        target_yaw = np.arctan2(vy, vx)
        yaw_error = (target_yaw - yaw + np.pi) % (2 * np.pi) - np.pi

        delta = np.clip(self.Kp_steer * yaw_error, -self.delta_max, self.delta_max)
        v_cmd = speed * np.cos(yaw_error)
        omega_wheel = v_cmd / self.wheel_radius

        self._drive(omega_wheel)
        self._steer(float(delta))

    def _drive(self, omega):
        for j in self.drive_joints:
            p.setJointMotorControl2(self.ugv_id, j, p.VELOCITY_CONTROL,
                                    targetVelocity=omega,
                                    force=self.max_drive_force,
                                    physicsClientId=self.CLIENT)

    def _steer(self, delta):
        for j in self.steer_joints:
            p.setJointMotorControl2(self.ugv_id, j, p.POSITION_CONTROL,
                                    targetPosition=delta,
                                    force=self.max_steer_force,
                                    physicsClientId=self.CLIENT)


# ======================================================================
# Part 1: dynamics + initial state (4 nodes, identical to Husky version)
# Node order: [UAV_leader, UGV, UAV_follower_1, UAV_follower_2]
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

    x0 = np.random.rand(n * d)
    x0[0:3] = np.array([-1.0, 0.0, 2.0])      # UAV leader
    # racecar chassis sits low — spawn near ground
    x0[3:6] = np.array([2.0, 0.0, 0.05])     # UGV (racecar)
    if x0[8] < 0.2:
        x0[8] = 0.6
    if x0[11] < 0.2:
        x0[11] = 0.6

    return M, x0


# ======================================================================
# Part 2: PyBullet sim (3 UAV + 1 racecar UGV + tree obstacle)
# ======================================================================
M_sys, x0 = get_dynamics_and_init()

NUM_UAVS = 3
NUM_UGVS = 1
TOTAL_AGENTS = NUM_UAVS + NUM_UGVS

IDX_UAV_LEADER = 0
IDX_UGV = 1
IDX_UAV_F1 = 2
IDX_UAV_F2 = 3

uav_init_xyzs = np.vstack([x0[0:3], x0[6:9], x0[9:12]])
ugv_init_xyz = x0[3:6]


def _resolve_unity_host():
    env_ip = os.environ.get("UNITY_HOST")
    if env_ip:
        return env_ip
    try:
        import subprocess
        mode = subprocess.check_output(["wslinfo", "--networking-mode"],
                                       stderr=subprocess.DEVNULL, timeout=2).decode().strip()
        if mode == "mirrored":
            return "127.0.0.1"
    except Exception:
        pass
    try:
        import subprocess
        out = subprocess.check_output(["ip", "route", "show", "default"], timeout=2).decode()
        for tok in out.split():
            if tok.count(".") == 3:
                return tok
    except Exception:
        pass
    return "127.0.0.1"


HEADLESS  = os.environ.get("HETERO_HEADLESS", "0") == "1"
UDP_ENABLE = os.environ.get("HETERO_UDP", "1") == "1"
LOAD_TREE = os.environ.get("HETERO_TREE", "1") == "1"

UDP_IP = _resolve_unity_host()
UDP_PORT = 5006
print(f"📡 UDP target (Unity host): {UDP_IP}:{UDP_PORT}")
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

env = CtrlAviary(
    drone_model=DroneModel.CF2X,
    num_drones=NUM_UAVS,
    initial_xyzs=uav_init_xyzs,
    physics=Physics.PYB,
    gui=not HEADLESS,
)

ctrls = [DSLPIDControl(drone_model=DroneModel.CF2X) for _ in range(NUM_UAVS)]

# Load UGV (racecar — Ackermann steering)
p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=env.CLIENT)
ugv_id = p.loadURDF(
    "racecar/racecar.urdf",
    basePosition=ugv_init_xyz,
    physicsClientId=env.CLIENT,
)
ugv_controller = RacecarAckermannController(ugv_id, env.CLIENT)

# Tree obstacle
tree_id = -1
if LOAD_TREE:
    try:
        tree_obj_path = "PyBullet_Terrain.obj"
        tree_collision_id = p.createCollisionShape(shapeType=p.GEOM_MESH, fileName=tree_obj_path,
                                                   meshScale=[1, 1, 1],
                                                   flags=p.GEOM_FORCE_CONCAVE_TRIMESH,
                                                   physicsClientId=env.CLIENT)
        tree_visual_id = p.createVisualShape(shapeType=p.GEOM_MESH, fileName=tree_obj_path,
                                             meshScale=[1, 1, 1], physicsClientId=env.CLIENT)
        tree_id = p.createMultiBody(baseMass=0,
                                    baseCollisionShapeIndex=tree_collision_id,
                                    baseVisualShapeIndex=tree_visual_id,
                                    basePosition=[0, 0, 0],
                                    baseOrientation=[0, 0, 0, 1],
                                    physicsClientId=env.CLIENT)
        print("🌲 树木障碍物模型加载成功")
    except Exception as e:
        print(f"⚠️ 加载树木模型失败: {e}")

formation_dt = 0.01
TOTAL_STEPS = int(os.environ.get("HETERO_STEPS", "10000"))

base_vel = np.array([0.2, 0.02, 0.0])
vel_feedforward = np.tile(base_vel, TOTAL_AGENTS)

pos_history = np.zeros((TOTAL_STEPS, TOTAL_AGENTS, 3))
actual_steps_run = 0

print("Starting heterogeneous formation simulation: 3 UAV + 1 racecar UGV")

try:
    for i in range(TOTAL_STEPS):
        actual_steps_run = i + 1

        obs_multi = [env._getDroneStateVector(j) for j in range(NUM_UAVS)]
        ugv_pos, ugv_quat = p.getBasePositionAndOrientation(ugv_id, physicsClientId=env.CLIENT)

        real_x_virtual = np.zeros(TOTAL_AGENTS * 3)
        real_x_virtual[IDX_UAV_LEADER * 3: IDX_UAV_LEADER * 3 + 3] = obs_multi[0][0:3]
        real_x_virtual[IDX_UGV        * 3: IDX_UGV        * 3 + 3] = np.array(ugv_pos)
        real_x_virtual[IDX_UAV_F1     * 3: IDX_UAV_F1     * 3 + 3] = obs_multi[1][0:3]
        real_x_virtual[IDX_UAV_F2     * 3: IDX_UAV_F2     * 3 + 3] = obs_multi[2][0:3]

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
            if UDP_ENABLE:
                msg = f"UAV,{j},{pos[0]:.4f},{pos[1]:.4f},{pos[2]:.4f},{rpy[0]:.4f},{rpy[1]:.4f},{rpy[2]:.4f}"
                sock.sendto(msg.encode(), (UDP_IP, UDP_PORT))

        # UGV (racecar) — Ackermann command
        target_vel_ugv = target_vels[IDX_UGV].copy()
        target_vel_ugv = np.clip(target_vel_ugv, -1.0, 1.0)
        target_vel_ugv[2] = 0.0
        ugv_controller.compute_and_apply_control(ugv_quat, target_vel_ugv)

        ugv_rpy = p.getEulerFromQuaternion(ugv_quat)
        pos_history[i, NUM_UAVS, :] = ugv_pos
        if UDP_ENABLE:
            ugv_msg = f"UGV,0,{ugv_pos[0]:.4f},{ugv_pos[1]:.4f},{ugv_pos[2]:.4f},{ugv_rpy[0]:.4f},{ugv_rpy[1]:.4f},{ugv_rpy[2]:.4f}"
            sock.sendto(ugv_msg.encode(), (UDP_IP, UDP_PORT))

        env.step(action_matrix)

        # Collision detection (only when tree is loaded)
        collision_detected = False
        crash_reason = ""
        drone_ids = env.DRONE_IDS

        if tree_id != -1:
            for d_id in drone_ids:
                contacts_with_tree = p.getContactPoints(bodyA=d_id, bodyB=tree_id, physicsClientId=env.CLIENT)
                if len(contacts_with_tree) > 0:
                    collision_detected = True
                    crash_reason = f"💥 炸机：无人机 [ID: {d_id}] 撞树"
                    break

            if not collision_detected:
                contacts_ugv_tree = p.getContactPoints(bodyA=ugv_id, bodyB=tree_id, physicsClientId=env.CLIENT)
                if len(contacts_ugv_tree) > 0:
                    collision_detected = True
                    crash_reason = "💥 撞车：racecar 撞树"

        if not collision_detected:
            for n in range(len(drone_ids)):
                for m in range(n + 1, len(drone_ids)):
                    contacts_between_drones = p.getContactPoints(bodyA=drone_ids[n], bodyB=drone_ids[m],
                                                                 physicsClientId=env.CLIENT)
                    if len(contacts_between_drones) > 0:
                        collision_detected = True
                        crash_reason = f"💥 空中相撞：UAV {drone_ids[n]} 与 UAV {drone_ids[m]}"
                        break
                if collision_detected:
                    break

        if collision_detected:
            print(f"\n=========================================")
            print(f"🛑 仿真紧急终止")
            print(f"{crash_reason}")
            print(f"发生时间步: {i} (约 {i * env.CTRL_TIMESTEP:.2f} 秒)")
            print(f"=========================================\n")
            break

        if not HEADLESS and i % 2 == 0:
            time.sleep(1 / 240.0 * 2)

except KeyboardInterrupt:
    print("\nSimulation interrupted by user.")
finally:
    env.close()
    sock.close()

    valid_pos_history = pos_history[:actual_steps_run, :, :]
    csv_data = valid_pos_history.reshape(actual_steps_run, TOTAL_AGENTS * 3)
    save_path = "hetero_trajectories_3uav_1racecar.csv"
    np.savetxt(save_path, csv_data, delimiter=",", fmt="%.4f")

    print(f"Simulation finished. Steps recorded: {actual_steps_run}")
    print(f"Trajectory saved to: {save_path}")
