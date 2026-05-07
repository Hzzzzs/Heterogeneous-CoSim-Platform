"""Noise robustness sweep for the heterogeneous 3-UAV + 1-UGV formation,
using the per-follower local-frame controller from
run_hetero_three_uav_one_ugv_new.py.

Node order (12-dim virtual state): [UAV_leader, UGV, UAV_F1, UAV_F2].
Leaders (indices 0, 1) have zero commanded velocity. Followers (indices 2, 3)
compute their velocity in their own body-yaw frame from noisy relative-position
measurements:
    z_ij_meas = R_z(-yaw_i) (x_j - x_i) + n_ij,   n_ij ~ N(0, sigma^2 I_3)
    u_i^local = sum_j M_ij @ z_ij_meas
    u_i^world = R_z(+yaw_i) u_i^local

Outputs go to hetero_noise/. Uses the clean HIL sigma=0 run as the baseline
for the plots (no ODE comparison).
"""

import csv as _csv
import os
import socket
import time

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pybullet as p
import pybullet_data

from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
from gym_pybullet_drones.utils.enums import DroneModel, Physics

from run_hetero_three_uav_one_ugv_new import get_dynamics_and_init
from tests.ugv_chassis_control import UGVController

OUT = "hetero_noise_moving"
RAW = os.path.join(OUT, "raw")
FIG = os.path.join(OUT, "figures")

SIGMAS = [0.0, 0.02, 0.05, 0.10, 0.15, 0.20, 0.30, 0.50]
PICK   = {"weak": 0.05, "medium": 0.15, "strong": 0.30}

STEPS = 6000
# LOOKAHEAD_DT is only used inside the per-step formation update for target_pos
# (target_pos = cur_pos + LOOKAHEAD_DT * v_cmd). It does NOT determine real time.
# Real per-step physics time is set by env.PYB_TIMESTEP (default 1/240 s); we
# use PYB_DT for time axes, settling-time, and total-time computations.
LOOKAHEAD_DT = 0.01
PYB_DT       = 1.0 / 240.0
SEED_NOISE = 42

# Common cruise velocity applied to ALL agents (including leaders), same as
# experiment_hetero_moving.py. The whole formation translates along +X at this
# speed; followers maintain the relative geometry via the local-frame law.
BASE_VEL = np.array([0.5, 0.0, 0.0])

NUM_UAVS = 3
NUM_UGVS = 1
TOTAL_AGENTS = NUM_UAVS + NUM_UGVS

IDX_UAV_LEADER = 0
IDX_UGV        = 1
IDX_UAV_F1     = 2
IDX_UAV_F2     = 3

LABELS = ["UAV_L", "UGV", "UAV_F1", "UAV_F2"]
PAIRS  = [(0, 2), (0, 3), (1, 2), (1, 3), (2, 3)]   # 5 communication edges


def Rz(angle):
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def run_hil(sigma, M, x0, steps=STEPS, seed=SEED_NOISE):
    """Run one PyBullet simulation with noise injected on each follower's
    pairwise relative-position measurement. Returns pos_history of shape
    (steps, TOTAL_AGENTS*3) in virtual-state column order
    [UAV_L, UGV, UAV_F1, UAV_F2].
    """
    rng = np.random.default_rng(seed)

    uav_init_xyzs = np.vstack([x0[0:3], x0[6:9], x0[9:12]])
    ugv_init_xyz  = x0[3:6]

    env = CtrlAviary(drone_model=DroneModel.CF2X, num_drones=NUM_UAVS,
                     initial_xyzs=uav_init_xyzs, physics=Physics.PYB, gui=False)
    ctrls = [DSLPIDControl(drone_model=DroneModel.CF2X) for _ in range(NUM_UAVS)]

    p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=env.CLIENT)
    ugv_id = p.loadURDF("husky/husky.urdf", basePosition=ugv_init_xyz,
                        physicsClientId=env.CLIENT)
    ugv_controller = UGVController(ugv_id, env.CLIENT)

    pos_hist = np.zeros((steps, TOTAL_AGENTS, 3))
    uav_virtual_idx = [IDX_UAV_LEADER, IDX_UAV_F1, IDX_UAV_F2]

    try:
        for i in range(steps):
            obs_multi = [env._getDroneStateVector(j) for j in range(NUM_UAVS)]
            ugv_pos, ugv_quat = p.getBasePositionAndOrientation(
                ugv_id, physicsClientId=env.CLIENT)

            # Build 12-dim virtual state
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

            velocities_matrix = np.zeros((TOTAL_AGENTS, 3))
            for agent_idx in range(TOTAL_AGENTS):
                if agent_idx in (IDX_UAV_LEADER, IDX_UGV):
                    continue
                yaw_i = agent_yaws[agent_idx]
                R_zi_inv = Rz(-yaw_i)
                R_zi     = Rz( yaw_i)
                xi = real_x[3*agent_idx : 3*agent_idx+3]
                v_local = np.zeros(3)
                for nb in range(TOTAL_AGENTS):
                    if nb == agent_idx:
                        continue
                    M_ij = M[3*agent_idx:3*agent_idx+3, 3*nb:3*nb+3]
                    if np.allclose(M_ij, 0):
                        continue
                    dp_world = real_x[3*nb:3*nb+3] - xi
                    rel_local = R_zi_inv @ dp_world
                    if sigma > 0:
                        rel_local = rel_local + rng.normal(0.0, sigma, size=3)
                    v_local += M_ij @ rel_local
                velocities_matrix[agent_idx] = R_zi @ v_local

            # Common feedforward: every agent (leaders and followers) gets
            # base_vel added on top of its formation feedback.
            velocities_matrix += BASE_VEL[None, :]

            velocities  = velocities_matrix.flatten()
            targets     = (real_x + LOOKAHEAD_DT * velocities).reshape((TOTAL_AGENTS, 3))
            target_vels = velocities.reshape((TOTAL_AGENTS, 3))

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
                pos_hist[i, uav_virtual_idx[j], :] = obs_multi[j][0:3]

            target_vel_ugv = target_vels[IDX_UGV].copy()
            target_vel_ugv = np.clip(target_vel_ugv, -1.0, 1.0)
            target_vel_ugv[2] = 0.0
            ugv_controller.compute_and_apply_control(ugv_quat, target_vel_ugv)
            pos_hist[i, IDX_UGV, :] = ugv_pos

            env.step(actions)
    finally:
        env.close()
    return pos_hist.reshape(steps, TOTAL_AGENTS * 3)


def settling_time(norms, threshold_frac=0.05):
    if norms[0] <= 0:
        return 0.0
    threshold = threshold_frac * norms[0]
    below = np.where(norms < threshold)[0]
    if len(below) == 0:
        return STEPS * PYB_DT
    for idx in below:
        if np.all(norms[idx:] < 5 * threshold):
            return idx * PYB_DT
    return below[0] * PYB_DT


def main():
    os.makedirs(RAW, exist_ok=True)
    os.makedirs(FIG, exist_ok=True)

    M, x0 = get_dynamics_and_init()
    print(f"M eigenvalues (real parts, sorted): "
          f"{np.sort(np.real(np.linalg.eigvals(M)))}")

    # ============== Run sweep ==============
    csv_paths = []
    t0 = time.time()
    for sigma in SIGMAS:
        path = os.path.join(RAW, f"sigma_{sigma:.3f}.csv")
        csv_paths.append(path)
        if os.path.exists(path):
            print(f"[skip] sigma={sigma}: exists")
            continue
        print(f"[run]  sigma={sigma}")
        t1 = time.time()
        traj = run_hil(sigma, M, x0)
        np.savetxt(path, traj, delimiter=",", fmt="%.6f")
        print(f"        done in {time.time()-t1:.1f}s")
    print(f"Total HIL wall time: {time.time()-t0:.1f}s")

    hil_trajs = [np.loadtxt(p, delimiter=",") for p in csv_paths]
    t_csv = np.arange(STEPS) * PYB_DT
    if SIGMAS[0] != 0.0:
        raise SystemExit("expected SIGMAS[0] == 0.0 as the HIL baseline")
    baseline = hil_trajs[0]
    base_final = baseline[-1].reshape(TOTAL_AGENTS, 3)
    base_norms = np.linalg.norm(baseline @ M.T, axis=1)

    # ============== Metrics ==============
    metrics = []
    for sigma, hil in zip(SIGMAS, hil_trajs):
        Mx = hil @ M.T
        norms = np.linalg.norm(Mx, axis=1)
        norm_final = float(np.linalg.norm(M @ hil[-1]))
        ts = settling_time(norms)
        endpoint_dist = float(np.linalg.norm(hil[-1] - baseline[-1]))
        tail = norms[int(0.8 * STEPS):]
        metrics.append({
            "sigma": sigma,
            "norm_final": norm_final,
            "tail_mean": float(np.mean(tail)),
            "tail_std":  float(np.std(tail)),
            "settling_time": ts,
            "endpoint_dist_to_hil0": endpoint_dist,
        })
        print(f"  sigma={sigma:5.3f}  ||Mx||_T={norm_final:.3e}  "
              f"tail_mean={metrics[-1]['tail_mean']:.3e}  "
              f"T_s={ts:5.1f}s  d(HIL,HIL0)={endpoint_dist:.3e}")

    mcsv = os.path.join(OUT, "metrics.csv")
    with open(mcsv, "w", newline="") as f:
        w = _csv.DictWriter(f, fieldnames=list(metrics[0].keys()))
        w.writeheader(); w.writerows(metrics)
    print(f"metrics -> {mcsv}")

    # ============== Figure 1: ||Mx|| vs time, linear y ==============
    pick_styles = {
        "weak":   ((0.18, 0.62, 0.32), "-"),
        "medium": ((0.85, 0.45, 0.10), "--"),
        "strong": ((0.75, 0.10, 0.10), ":"),
    }
    fig, ax = plt.subplots(figsize=(8, 4.2))
    ax.plot(t_csv, base_norms, "-",
            color=(0.10, 0.50, 0.85), linewidth=1.6, alpha=0.9,
            label="HIL ($\\sigma$=0) baseline")
    for level, sigma in PICK.items():
        idx = SIGMAS.index(sigma)
        norms = np.linalg.norm(hil_trajs[idx] @ M.T, axis=1)
        c, ls = pick_styles[level]
        ax.plot(t_csv, norms, ls,
                color=c, linewidth=1.5, alpha=0.95,
                label=f"HIL {level} ($\\sigma$={sigma})")
    ax.set_xlabel("t (s)"); ax.set_ylabel(r"$\|Mx\|_2$")
    ax.set_title(r"$\|Mx\|$ convergence under increasing measurement noise")
    ax.grid(True, alpha=0.4)
    ax.legend(loc="upper right", fontsize=9)
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "mx_norm_vs_time.png"), dpi=140)
    plt.close(fig)

    # ============== Figure 2: 3D trajectories (weak/medium/strong) ==============
    agent_cmap = plt.get_cmap("tab10")
    fig = plt.figure(figsize=(15, 5.0))
    fig.suptitle("Formation trajectories under three noise levels "
                 "(HIL $\\sigma$=0 finals overlaid as stars)",
                 fontsize=13, fontweight="bold")
    for k, (level, sigma) in enumerate(PICK.items()):
        idx = SIGMAS.index(sigma)
        hil = hil_trajs[idx]
        ax = fig.add_subplot(1, 3, k+1, projection="3d")
        for j in range(TOTAL_AGENTS):
            c = agent_cmap(j)
            ax.plot(hil[:, 3*j], hil[:, 3*j+1], hil[:, 3*j+2],
                    "-", color=c, linewidth=0.9, alpha=0.7,
                    label=LABELS[j] if k == 0 else None)
            ax.plot(hil[0, 3*j], hil[0, 3*j+1], hil[0, 3*j+2],
                    "o", color=c, markerfacecolor="white", markersize=6)
        hil_final = hil[-1].reshape(TOTAL_AGENTS, 3)
        for j in range(TOTAL_AGENTS):
            ax.plot(hil_final[j, 0], hil_final[j, 1], hil_final[j, 2],
                    "D", color=agent_cmap(j), markersize=8,
                    markeredgecolor="black", markeredgewidth=0.6)
        for a, b in PAIRS:
            ax.plot([hil_final[a, 0], hil_final[b, 0]],
                    [hil_final[a, 1], hil_final[b, 1]],
                    [hil_final[a, 2], hil_final[b, 2]],
                    "-", color=(0.25, 0.25, 0.25), linewidth=1.6, alpha=0.85)
        for j in range(TOTAL_AGENTS):
            ax.plot(base_final[j, 0], base_final[j, 1], base_final[j, 2],
                    "*", color=agent_cmap(j), markersize=14,
                    markeredgecolor="black", markeredgewidth=0.7,
                    label="HIL $\\sigma$=0 final" if (k == 0 and j == 0) else None)
        ax.set_title(f"{level} noise ($\\sigma$={sigma})", fontsize=11)
        ax.set_xlabel("X"); ax.set_ylabel("Y"); ax.set_zlabel("Z")
        if k == 0:
            ax.legend(loc="upper left", fontsize=8)
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    fig.savefig(os.path.join(FIG, "formation_trajectories.png"), dpi=140)
    plt.close(fig)

    print(f"figures -> {FIG}/")


if __name__ == "__main__":
    main()
