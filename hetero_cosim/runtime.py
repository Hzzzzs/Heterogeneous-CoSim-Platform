"""Demo runtime for Unity-visualized heterogeneous formation experiments.

Each `demo_*.py` script wires together:
  - the dynamics from run_hetero_three_uav_one_ugv_new.get_dynamics_and_init
  - a per-step `compute_velocities(...)` callback that implements its specific
    control law (baseline, moving, noise, noise+kf)
  - this `run_demo(...)` helper, which handles PyBullet env, GUI, UGV, UDP and
    real-time pacing — so each demo file is small and only describes the law.

Env vars (forwarded to all demos):
    HETERO_STEPS    default 6000 (~25 s @ 240 Hz)
    HETERO_HEADLESS "1" -> no GUI, no real-time sleep
    UNITY_UDP       "0" -> disable UDP send
    UNITY_HOST      override Unity host IP (auto-resolved otherwise)
    UNITY_PORT      default 5006
"""

import os
import time

import numpy as np
import pybullet as p
import pybullet_data

from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
from gym_pybullet_drones.utils.enums import DroneModel, Physics

from hetero_cosim.formations.hetero_ugv import get_dynamics_and_init
from hetero_cosim.ugv_chassis_control import UGVController
from hetero_cosim.unity_bridge import UnityBridge


NUM_UAVS = 3
NUM_UGVS = 1
TOTAL_AGENTS = NUM_UAVS + NUM_UGVS

IDX_UAV_LEADER = 0
IDX_UGV        = 1
IDX_UAV_F1     = 2
IDX_UAV_F2     = 3

LOOKAHEAD_DT = 0.001
PYB_DT       = 1.0 / 240.0


def Rz(angle):
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def Rz_inv(angle):
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, s, 0.0], [-s, c, 0.0], [0.0, 0.0, 1.0]])


def follower_local_frame_velocity(M, real_x, agent_idx, agent_yaws,
                                  measurement_fn=None):
    """Per-follower local-frame velocity, given a measurement function.

    measurement_fn(agent_idx, nb, rel_local_truth) -> rel_local_estimate
    If None, measurement is noise-free pass-through (= rel_local_truth).
    """
    yaw_i = agent_yaws[agent_idx]
    R_zi     = Rz(yaw_i)
    R_zi_inv = Rz_inv(yaw_i)
    xi = real_x[3*agent_idx : 3*agent_idx+3]
    v_local = np.zeros(3)
    for nb in range(TOTAL_AGENTS):
        if nb == agent_idx:
            continue
        M_ij = M[3*agent_idx:3*agent_idx+3, 3*nb:3*nb+3]
        if np.allclose(M_ij, 0):
            continue
        dp_world = real_x[3*nb:3*nb+3] - xi
        rel_local_truth = R_zi_inv @ dp_world
        if measurement_fn is None:
            rel_local = rel_local_truth
        else:
            rel_local = measurement_fn(agent_idx, nb, rel_local_truth)
        v_local += M_ij @ rel_local
    return R_zi @ v_local


def run_demo(compute_velocities, *, name="demo"):
    """Generic demo loop. `compute_velocities(M, real_x, agent_yaws, step_idx)`
    returns a (TOTAL_AGENTS, 3) array of world-frame target velocities.

    `name` shows in the console banner.
    """
    HEADLESS    = os.environ.get("HETERO_HEADLESS", "0") == "1"
    TOTAL_STEPS = int(os.environ.get("HETERO_STEPS", "6000"))
    print(f"=== {name}: STEPS={TOTAL_STEPS} (~{TOTAL_STEPS*PYB_DT:.1f}s), "
          f"GUI={'OFF' if HEADLESS else 'ON'} ===")

    M, x0 = get_dynamics_and_init()
    uav_init_xyzs = np.vstack([x0[0:3], x0[6:9], x0[9:12]])
    ugv_init_xyz  = x0[3:6]

    env = CtrlAviary(drone_model=DroneModel.CF2X, num_drones=NUM_UAVS,
                     initial_xyzs=uav_init_xyzs, physics=Physics.PYB,
                     gui=not HEADLESS)
    ctrls = [DSLPIDControl(drone_model=DroneModel.CF2X) for _ in range(NUM_UAVS)]

    p.setAdditionalSearchPath(pybullet_data.getDataPath(),
                              physicsClientId=env.CLIENT)
    ugv_id = p.loadURDF("husky/husky.urdf", basePosition=ugv_init_xyz,
                        physicsClientId=env.CLIENT)
    ugv_controller = UGVController(ugv_id, env.CLIENT)

    # Static obstacles — auto-scan the directory set by HETERO_OBSTACLE_DIR
    # (default: "obstacles/" under the repo root) and load every .obj it finds
    # as a concave-trimesh static body. Each obstacle keeps its own world
    # coordinates from the OBJ file. Set HETERO_OBSTACLE_DIR="" to disable.
    obs_dir = os.environ.get("HETERO_OBSTACLE_DIR", "obstacles")
    obstacle_ids = {}   # body_id -> filename, for crash reporting
    if obs_dir and os.path.isdir(obs_dir):
        obj_files = sorted(f for f in os.listdir(obs_dir) if f.lower().endswith(".obj"))
        for fname in obj_files:
            path = os.path.join(obs_dir, fname)
            try:
                tc = p.createCollisionShape(p.GEOM_MESH, fileName=path,
                                            meshScale=[1, 1, 1],
                                            flags=p.GEOM_FORCE_CONCAVE_TRIMESH,
                                            physicsClientId=env.CLIENT)
                tv = p.createVisualShape(p.GEOM_MESH, fileName=path,
                                         meshScale=[1, 1, 1],
                                         physicsClientId=env.CLIENT)
                bid = p.createMultiBody(baseMass=0,
                                        baseCollisionShapeIndex=tc,
                                        baseVisualShapeIndex=tv,
                                        basePosition=[0, 0, 0],
                                        physicsClientId=env.CLIENT)
                obstacle_ids[bid] = fname
                print(f"[obstacle] loaded {path} (id={bid})")
            except Exception as exc:
                print(f"[obstacle] failed to load {path}: {exc}")
        if not obj_files:
            print(f"[obstacle] no .obj files in '{obs_dir}/' — running without obstacles")
    elif obs_dir:
        print(f"[obstacle] directory '{obs_dir}' not found — running without obstacles")

    bridge = UnityBridge()
    uav_virtual_idx = [IDX_UAV_LEADER, IDX_UAV_F1, IDX_UAV_F2]

    try:
        for i in range(TOTAL_STEPS):
            obs_multi = [env._getDroneStateVector(j) for j in range(NUM_UAVS)]
            ugv_pos, ugv_quat = p.getBasePositionAndOrientation(
                ugv_id, physicsClientId=env.CLIENT)

            real_x = np.zeros(TOTAL_AGENTS * 3)
            real_x[IDX_UAV_LEADER*3 : IDX_UAV_LEADER*3+3] = obs_multi[0][0:3]
            real_x[IDX_UGV*3        : IDX_UGV*3+3]        = np.array(ugv_pos)
            real_x[IDX_UAV_F1*3     : IDX_UAV_F1*3+3]     = obs_multi[1][0:3]
            real_x[IDX_UAV_F2*3     : IDX_UAV_F2*3+3]     = obs_multi[2][0:3]

            agent_yaws = {
                IDX_UAV_LEADER: obs_multi[0][9],
                IDX_UAV_F1:     obs_multi[1][9],
                IDX_UAV_F2:     obs_multi[2][9],
            }

            velocities_matrix = compute_velocities(M, real_x, agent_yaws, i)
            velocities = velocities_matrix.flatten()

            targets = (real_x + LOOKAHEAD_DT * velocities).reshape(TOTAL_AGENTS, 3)
            target_vels = velocities.reshape(TOTAL_AGENTS, 3)

            target_pos_uav = np.vstack([targets[IDX_UAV_LEADER],
                                        targets[IDX_UAV_F1],
                                        targets[IDX_UAV_F2]])
            target_vel_uav = np.vstack([target_vels[IDX_UAV_LEADER],
                                        target_vels[IDX_UAV_F1],
                                        target_vels[IDX_UAV_F2]])
            target_vel_uav = np.clip(target_vel_uav, -1.5, 1.5)

            actions = np.zeros((NUM_UAVS, 4))
            for j in range(NUM_UAVS):
                a, _, _ = ctrls[j].computeControlFromState(
                    control_timestep=env.CTRL_TIMESTEP,
                    state=obs_multi[j], target_pos=target_pos_uav[j],
                    target_rpy=np.zeros(3), target_vel=target_vel_uav[j])
                actions[j] = a
                bridge.send_uav(j, obs_multi[j][0:3], obs_multi[j][7:10])

            target_vel_ugv = target_vels[IDX_UGV].copy()
            target_vel_ugv = np.clip(target_vel_ugv, -1.0, 1.0)
            target_vel_ugv[2] = 0.0
            ugv_controller.compute_and_apply_control(ugv_quat, target_vel_ugv)
            bridge.send_ugv(0, ugv_pos, ugv_quat)

            env.step(actions)

            # Collision detection
            drone_ids = env.DRONE_IDS
            collision_msg = None

            # 1) UAV ↔ obstacle
            for d_id in drone_ids:
                for obs_id, obs_name in obstacle_ids.items():
                    if len(p.getContactPoints(bodyA=d_id, bodyB=obs_id,
                                              physicsClientId=env.CLIENT)) > 0:
                        collision_msg = (f"💥 UAV [id={d_id}] hit obstacle "
                                         f"'{obs_name}'")
                        break
                if collision_msg:
                    break

            # 2) UGV ↔ obstacle
            if not collision_msg:
                for obs_id, obs_name in obstacle_ids.items():
                    if len(p.getContactPoints(bodyA=ugv_id, bodyB=obs_id,
                                              physicsClientId=env.CLIENT)) > 0:
                        collision_msg = (f"💥 UGV hit obstacle '{obs_name}'")
                        break

            # 3) UAV ↔ UAV mid-air
            if not collision_msg:
                for n in range(len(drone_ids)):
                    for m in range(n + 1, len(drone_ids)):
                        if len(p.getContactPoints(bodyA=drone_ids[n],
                                                  bodyB=drone_ids[m],
                                                  physicsClientId=env.CLIENT)) > 0:
                            collision_msg = (f"💥 mid-air: UAV {drone_ids[n]} "
                                             f"<-> UAV {drone_ids[m]}")
                            break
                    if collision_msg:
                        break

            # 4) UAV ↔ UGV
            if not collision_msg:
                for d_id in drone_ids:
                    if len(p.getContactPoints(bodyA=d_id, bodyB=ugv_id,
                                              physicsClientId=env.CLIENT)) > 0:
                        collision_msg = (f"💥 UAV [id={d_id}] hit the UGV")
                        break

            if collision_msg:
                print(f"\n{collision_msg}  at step {i} (t = {i * PYB_DT:.2f} s)")
                break

            # Real-time pacing for GUI viewing
            if not HEADLESS and i % 2 == 0:
                time.sleep(PYB_DT * 2)

    except KeyboardInterrupt:
        print(f"\n[{name}] interrupted by user.")
    finally:
        env.close()
        bridge.close()
        print(f"[{name}] done.")
