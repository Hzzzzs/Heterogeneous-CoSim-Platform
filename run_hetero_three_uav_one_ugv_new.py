"""Heterogeneous 3-UAV + 1-UGV formation, distributed local-frame controller.

Ported from run_hetero_three_uav_one_ugv.py to match the per-follower local-frame
control law from run_threesimilar_new.py:
  - Each follower computes its velocity in its own body-yaw frame from
    relative-position measurements to neighbors, then rotates the result back
    to the world frame. Leaders (UAV_leader, UGV) are commanded zero velocity
    from the formation law.
  - Global cruise feedforward (base_vel) removed; this script produces a
    static formation, matching run_threesimilar_new.py's behavior.
  - UGV has no natural body yaw for the observation model; it is a leader in
    the formation law, so it never observes anyone \N{EM DASH} no special handling needed.

Node order (12-dim virtual state): [UAV_leader, UGV, UAV_F1, UAV_F2].

Toggles for headless experiments:
  HEADLESS      : disable PyBullet GUI and real-time pacing
  UDP_ENABLE    : disable Unity UDP messages
  LOAD_TREE     : skip loading PyBullet_Terrain.obj (for headless runs)
"""

import os
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


HEADLESS   = os.environ.get("HETERO_HEADLESS", "0") == "1"
UDP_ENABLE = os.environ.get("HETERO_UDP", "0" if HEADLESS else "1") == "1"
LOAD_TREE  = os.environ.get("HETERO_TREE", "0" if HEADLESS else "1") == "1"
OUT_CSV    = os.environ.get("HETERO_OUT", "hetero_trajectories_3uav_1ugv_new.csv")
TOTAL_STEPS = int(os.environ.get("HETERO_STEPS", "10000"))


# ======================================================================
# Part 1: 4-node formation dynamics (identical to run_threesimilar_new.py)
# Node order: [UAV_leader, UGV, UAV_F1, UAV_F2]
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

    # Initial positions: use deterministic values (not random) so sim/hil
    # comparison is reproducible. Followers get fixed initial offsets.
    # NOTE: UGV spawn z is set to 0.0 (ground level). The Husky URDF rests at
    # z ≈ -0.001 m under gravity; giving it an elevated spawn z causes it to
    # drop and introduces an ~8 cm settling error in sim-vs-HIL comparisons.
    x0 = np.zeros(n * d)
    x0[0:3]   = np.array([0.0, 0.0, 2.0])      # UAV leader
    x0[3:6]   = np.array([1.0, 0.0, 0.0])      # UGV (spawn on ground, no drop)
    x0[6:9]   = np.array([0.5, 1.0, 1.5])      # UAV F1
    x0[9:12]  = np.array([1.5, 0.5, 1.5])      # UAV F2

    return M, x0


# ======================================================================
# Part 2: Simulation main
# ======================================================================
def main():
    M_sys, x0 = get_dynamics_and_init()

    NUM_UAVS = 3
    NUM_UGVS = 1
    TOTAL_AGENTS = NUM_UAVS + NUM_UGVS

    IDX_UAV_LEADER = 0
    IDX_UGV        = 1
    IDX_UAV_F1     = 2
    IDX_UAV_F2     = 3

    uav_init_xyzs = np.vstack([x0[0:3], x0[6:9], x0[9:12]])
    ugv_init_xyz  = x0[3:6]

    sock = None
    UDP_IP = "127.0.0.1"; UDP_PORT = 5006
    if UDP_ENABLE:
        def _resolve_unity_host():
            import subprocess
            env_ip = os.environ.get("UNITY_HOST")
            if env_ip:
                return env_ip
            try:
                mode = subprocess.check_output(
                    ["wslinfo", "--networking-mode"],
                    stderr=subprocess.DEVNULL, timeout=2).decode().strip()
                if mode == "mirrored":
                    return "127.0.0.1"
            except Exception:
                pass
            try:
                out = subprocess.check_output(
                    ["ip", "route", "show", "default"], timeout=2).decode()
                for tok in out.split():
                    if tok.count(".") == 3:
                        return tok
            except Exception:
                pass
            return "127.0.0.1"
        UDP_IP = _resolve_unity_host()
        print(f"UDP target (Unity host): {UDP_IP}:{UDP_PORT}")
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    env = CtrlAviary(
        drone_model=DroneModel.CF2X,
        num_drones=NUM_UAVS,
        initial_xyzs=uav_init_xyzs,
        physics=Physics.PYB,
        gui=not HEADLESS,
    )
    ctrls = [DSLPIDControl(drone_model=DroneModel.CF2X) for _ in range(NUM_UAVS)]

    # UGV
    p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=env.CLIENT)
    ugv_id = p.loadURDF("husky/husky.urdf", basePosition=ugv_init_xyz,
                        physicsClientId=env.CLIENT)
    ugv_controller = UGVController(ugv_id, env.CLIENT)

    # Tree obstacle (optional)
    tree_id = -1
    if LOAD_TREE:
        try:
            tree_path = "PyBullet_Terrain.obj"
            tcoll = p.createCollisionShape(shapeType=p.GEOM_MESH, fileName=tree_path,
                                           meshScale=[1, 1, 1], physicsClientId=env.CLIENT)
            tvis  = p.createVisualShape(shapeType=p.GEOM_MESH, fileName=tree_path,
                                        meshScale=[1, 1, 1], physicsClientId=env.CLIENT)
            tree_id = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=tcoll,
                                        baseVisualShapeIndex=tvis,
                                        basePosition=[0, 0, 0],
                                        baseOrientation=[0, 0, 0, 1],
                                        physicsClientId=env.CLIENT)
            print("Tree obstacle loaded.")
        except Exception as e:
            print(f"Tree load failed: {e}")
            tree_id = -1

    formation_dt = 0.01
    pos_history = np.zeros((TOTAL_STEPS, TOTAL_AGENTS, 3))
    actual_steps_run = 0

    print(f"Starting heterogeneous formation sim: {NUM_UAVS} UAV + {NUM_UGVS} UGV "
          f"(headless={HEADLESS}, steps={TOTAL_STEPS})")

    try:
        for i in range(TOTAL_STEPS):
            actual_steps_run = i + 1

            obs_multi = [env._getDroneStateVector(j) for j in range(NUM_UAVS)]
            ugv_pos, ugv_quat = p.getBasePositionAndOrientation(
                ugv_id, physicsClientId=env.CLIENT)

            # Build virtual 12-dim state in node order [leader, ugv, f1, f2]
            real_x_virtual = np.zeros(TOTAL_AGENTS * 3)
            real_x_virtual[IDX_UAV_LEADER*3 : IDX_UAV_LEADER*3+3] = obs_multi[0][0:3]
            real_x_virtual[IDX_UGV*3        : IDX_UGV*3+3]        = np.array(ugv_pos)
            real_x_virtual[IDX_UAV_F1*3     : IDX_UAV_F1*3+3]     = obs_multi[1][0:3]
            real_x_virtual[IDX_UAV_F2*3     : IDX_UAV_F2*3+3]     = obs_multi[2][0:3]

            # Per-follower local-frame control law.
            # UAV_leader (idx 0) and UGV (idx 1) are leaders: zero velocity.
            # F1 (idx 2) and F2 (idx 3) compute v in their own body-yaw frame.
            agent_yaws = {
                IDX_UAV_LEADER: obs_multi[0][9],
                IDX_UAV_F1:     obs_multi[1][9],
                IDX_UAV_F2:     obs_multi[2][9],
                # UGV yaw not used (it is a leader)
            }

            velocities_matrix = np.zeros((TOTAL_AGENTS, 3))
            for agent_idx in range(TOTAL_AGENTS):
                if agent_idx in (IDX_UAV_LEADER, IDX_UGV):
                    continue   # leader: zero velocity
                yaw_i = agent_yaws[agent_idx]
                R_zi_inv = np.array([
                    [np.cos(-yaw_i), -np.sin(-yaw_i), 0],
                    [np.sin(-yaw_i),  np.cos(-yaw_i), 0],
                    [0,               0,              1],
                ])
                R_zi = np.array([
                    [np.cos(yaw_i), -np.sin(yaw_i), 0],
                    [np.sin(yaw_i),  np.cos(yaw_i), 0],
                    [0,              0,             1],
                ])
                xi = real_x_virtual[3*agent_idx : 3*agent_idx+3]
                v_local = np.zeros(3)
                for nb in range(TOTAL_AGENTS):
                    if nb == agent_idx:
                        continue
                    M_ij = M_sys[3*agent_idx:3*agent_idx+3, 3*nb:3*nb+3]
                    if np.allclose(M_ij, 0):
                        continue
                    dp_global = real_x_virtual[3*nb:3*nb+3] - xi
                    dp_local  = R_zi_inv @ dp_global
                    v_local += M_ij @ dp_local
                velocities_matrix[agent_idx] = R_zi @ v_local

            velocities = velocities_matrix.flatten()
            targets    = (real_x_virtual + formation_dt * velocities).reshape((TOTAL_AGENTS, 3))
            target_vels = velocities.reshape((TOTAL_AGENTS, 3))

            target_pos_uav = np.vstack([targets[IDX_UAV_LEADER],
                                        targets[IDX_UAV_F1],
                                        targets[IDX_UAV_F2]])
            target_vel_uav = np.vstack([target_vels[IDX_UAV_LEADER],
                                        target_vels[IDX_UAV_F1],
                                        target_vels[IDX_UAV_F2]])
            target_vel_uav = np.clip(target_vel_uav, -1.5, 1.5)

            action_matrix = np.zeros((NUM_UAVS, 4))
            # Physical UAV index j -> virtual-state index.
            # obs_multi order is [leader, follower_1, follower_2], so the
            # virtual indices are [IDX_UAV_LEADER, IDX_UAV_F1, IDX_UAV_F2].
            uav_virtual_idx = [IDX_UAV_LEADER, IDX_UAV_F1, IDX_UAV_F2]
            for j in range(NUM_UAVS):
                action, _, _ = ctrls[j].computeControlFromState(
                    control_timestep=env.CTRL_TIMESTEP,
                    state=obs_multi[j],
                    target_pos=target_pos_uav[j],
                    target_rpy=np.zeros(3),
                    target_vel=target_vel_uav[j],
                )
                action_matrix[j] = action

                pos = obs_multi[j][0:3]
                rpy = obs_multi[j][7:10]
                # Write into pos_history at the VIRTUAL-state index so the
                # saved CSV column order matches the M matrix's node order
                # [UAV_leader, UGV, UAV_F1, UAV_F2].
                pos_history[i, uav_virtual_idx[j], :] = pos
                if sock is not None:
                    msg = f"UAV,{j},{pos[0]:.4f},{pos[1]:.4f},{pos[2]:.4f},{rpy[0]:.4f},{rpy[1]:.4f},{rpy[2]:.4f}"
                    sock.sendto(msg.encode(), (UDP_IP, UDP_PORT))

            # UGV: also leader (zero formation velocity). Let the chassis
            # controller hold it still.
            target_vel_ugv = target_vels[IDX_UGV].copy()
            target_vel_ugv = np.clip(target_vel_ugv, -1.0, 1.0)
            target_vel_ugv[2] = 0.0
            ugv_controller.compute_and_apply_control(ugv_quat, target_vel_ugv)

            ugv_rpy = p.getEulerFromQuaternion(ugv_quat)
            # UGV goes at its virtual-state index too.
            pos_history[i, IDX_UGV, :] = ugv_pos
            if sock is not None:
                ugv_msg = f"UGV,0,{ugv_pos[0]:.4f},{ugv_pos[1]:.4f},{ugv_pos[2]:.4f},{ugv_rpy[0]:.4f},{ugv_rpy[1]:.4f},{ugv_rpy[2]:.4f}"
                sock.sendto(ugv_msg.encode(), (UDP_IP, UDP_PORT))

            env.step(action_matrix)

            # Collision detection (only if tree loaded)
            if tree_id != -1:
                crashed = False
                drone_ids = env.DRONE_IDS
                for d_id in drone_ids:
                    if p.getContactPoints(bodyA=d_id, bodyB=tree_id,
                                          physicsClientId=env.CLIENT):
                        print(f"UAV {d_id} hit tree at step {i}")
                        crashed = True; break
                if not crashed:
                    if p.getContactPoints(bodyA=ugv_id, bodyB=tree_id,
                                          physicsClientId=env.CLIENT):
                        print(f"UGV hit tree at step {i}")
                        crashed = True
                if crashed:
                    break

            if not HEADLESS and i % 2 == 0:
                time.sleep(1 / 240.0 * 2)

    except KeyboardInterrupt:
        print("Simulation interrupted by user.")
    finally:
        env.close()
        if sock is not None:
            sock.close()

        valid_pos_history = pos_history[:actual_steps_run, :, :]
        csv_data = valid_pos_history.reshape(actual_steps_run, TOTAL_AGENTS * 3)
        np.savetxt(OUT_CSV, csv_data, delimiter=",", fmt="%.4f")
        print(f"Steps recorded: {actual_steps_run}")
        print(f"Trajectory saved to: {OUT_CSV}")


if __name__ == "__main__":
    main()
