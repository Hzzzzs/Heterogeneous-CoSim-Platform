"""Experiment 4 — Measurement-noise robustness WITH a per-edge Kalman filter.

WHAT IT DOES
    Identical setup to Experiment 3 (same node order, control law, common
    cruise base_vel = [0.5, 0, 0], sigma sweep, sigma=0 HIL baseline), with one
    addition: every follower runs a small 3D Kalman filter on each neighbor
    measurement before feeding it to the formation law. This pushes the
    robustness boundary far outward — noise levels that are unstable without
    the KF become stable with it.

    KF design (per (follower, neighbor) pair, independent):
        state       x ∈ R^3 : neighbor position in follower i's body frame
        process     x_{k+1} = x_k + w_k,            Q = q·I  (random walk)
        measurement z_k      = x_k + v_k,            R = sigma^2·I
        init        x_0 = first measurement,         P_0 = R + Q
        predict     P_pred = P + Q
        gain        K = P_pred / (P_pred + R)
        update      x_post = x_pred + K(z - x_pred); P_post = (1-K)P_pred
    The process-noise q is FIXED (a real physical parameter); only R tracks
    sigma. As sigma grows, K shrinks, so the filter trusts its history more and
    suppresses noise harder (effective noise grows ~ sqrt(sigma), not sigma).

HOW TO RUN
    python -m hetero_cosim.experiments.noise_kf
    Per-sigma HIL trajectories are cached in hetero_noise_moving_kf/raw/.
    Delete that folder to force a fresh sweep. Compare metrics.csv against
    hetero_noise_moving/metrics.csv (Exp 3) to quantify the KF improvement.

CONFIG (edit constants near the top)
    SIGMAS   list of noise std values to sweep (meters).
    PICK     the three sigmas drawn on the figures.
    KF_Q     process-noise variance per axis (default (5 mm)^2).
    BASE_VEL common cruise velocity (default [0.5, 0, 0]).

OUTPUTS  ->  hetero_noise_moving_kf/
    raw/sigma_*.csv               per-sigma KF-filtered HIL trajectory
    metrics.csv                   per-sigma ||Mx(T)||, tail_mean, settling time
    figures/mx_norm_vs_time.png   ||Mx|| curves for the picked sigmas
    figures/formation_trajectories.png   3D trajectories at the picked sigmas
"""

import csv as _csv
import os
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

from hetero_cosim.formations.hetero_ugv import get_dynamics_and_init
from hetero_cosim.ugv_chassis_control import UGVController
from hetero_cosim.unity_bridge import UnityBridge

OUT = "hetero_noise_moving_kf"
RAW = os.path.join(OUT, "raw")
FIG = os.path.join(OUT, "figures")

SIGMAS = [0.0, 0.30, 0.50, 1.0]
PICK   = {"weak": 0.30, "medium": 0.50, "strong": 1.0}

STEPS = 6000
LOOKAHEAD_DT = 0.001
PYB_DT       = 1.0 / 240.0
SEED_NOISE = 42

BASE_VEL = np.array([0.5, 0.0, 0.0])

# Kalman filter process noise (random-walk variance per step, per axis).
# At PYB_DT ≈ 4 ms and ~0.5 m/s motion, neighbor pos in body frame moves
# ~2 mm/step, so we expect random-walk std on the order of 5 mm.
KF_Q = (5e-3) ** 2

NUM_UAVS = 3
NUM_UGVS = 1
TOTAL_AGENTS = NUM_UAVS + NUM_UGVS

IDX_UAV_LEADER = 0
IDX_UGV        = 1
IDX_UAV_F1     = 2
IDX_UAV_F2     = 3

LABELS = ["UAV_L", "UGV", "UAV_F1", "UAV_F2"]
PAIRS  = [(0, 2), (0, 3), (1, 2), (1, 3), (2, 3)]


def Rz(angle):
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def run_hil(sigma, M, x0, steps=STEPS, seed=SEED_NOISE, q=KF_Q):
    """HIL run with noise on per-edge body-frame measurements + per-edge KF.

    The KF is only meaningful when sigma > 0; for sigma = 0 we still run it
    (with R = 0) so the data flow is identical — it then degenerates into a
    pass-through (K = 1) and adds no error.
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

    # Per-edge KF state: kf_x[i, j] = 3D estimate of neighbor j in follower i's
    # body frame; kf_P[i, j] = scalar covariance (isotropic in 3D).
    # Only entries (i, j) where i is a follower and M_ij is nonzero are used.
    kf_x = np.zeros((TOTAL_AGENTS, TOTAL_AGENTS, 3))
    kf_P = np.zeros((TOTAL_AGENTS, TOTAL_AGENTS))   # scalar, broadcast to 3D
    kf_init = np.zeros((TOTAL_AGENTS, TOTAL_AGENTS), dtype=bool)
    R = sigma ** 2  # measurement variance (per axis)

    bridge = UnityBridge()
    pos_hist = np.zeros((steps, TOTAL_AGENTS, 3))
    uav_virtual_idx = [IDX_UAV_LEADER, IDX_UAV_F1, IDX_UAV_F2]

    try:
        for i in range(steps):
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
                    # Truth measurement in follower's body frame
                    dp_world  = real_x[3*nb:3*nb+3] - xi
                    rel_local_truth = R_zi_inv @ dp_world
                    # Apply sensor noise
                    if sigma > 0:
                        z = rel_local_truth + rng.normal(0.0, sigma, size=3)
                    else:
                        z = rel_local_truth.copy()
                    # ---- Kalman filter (per (i, nb) edge) ----
                    if not kf_init[agent_idx, nb]:
                        kf_x[agent_idx, nb] = z.copy()
                        kf_P[agent_idx, nb] = R + q
                        kf_init[agent_idx, nb] = True
                        rel_local = z   # first sample: no smoothing yet
                    else:
                        # predict
                        P_pred = kf_P[agent_idx, nb] + q
                        # update
                        K_gain = P_pred / (P_pred + R) if (P_pred + R) > 0 else 0.0
                        kf_x[agent_idx, nb] = kf_x[agent_idx, nb] + K_gain * (z - kf_x[agent_idx, nb])
                        kf_P[agent_idx, nb] = (1.0 - K_gain) * P_pred
                        rel_local = kf_x[agent_idx, nb]
                    v_local += M_ij @ rel_local
                velocities_matrix[agent_idx] = R_zi @ v_local

            # Common cruise feedforward (every agent, leaders included)
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
                bridge.send_uav(j, obs_multi[j][0:3], obs_multi[j][7:10])

            target_vel_ugv = target_vels[IDX_UGV].copy()
            target_vel_ugv = np.clip(target_vel_ugv, -1.0, 1.0)
            target_vel_ugv[2] = 0.0
            ugv_controller.compute_and_apply_control(ugv_quat, target_vel_ugv)
            pos_hist[i, IDX_UGV, :] = ugv_pos
            bridge.send_ugv(0, ugv_pos, ugv_quat)

            env.step(actions)
    finally:
        env.close()
        bridge.close()
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
    print(f"M eigenvalues (real, sorted): "
          f"{np.sort(np.real(np.linalg.eigvals(M)))}")
    print(f"KF process noise q = {KF_Q:.2e}  (std ≈ {np.sqrt(KF_Q)*1000:.1f} mm/step)")

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

    # ---- Figure 1: ||Mx|| vs time ----
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
                label=f"HIL+KF {level} ($\\sigma$={sigma})")
    ax.set_xlabel("t (s)"); ax.set_ylabel(r"$\|Mx\|_2$")
    ax.set_title(r"$\|Mx\|$ convergence with per-edge Kalman filter")
    ax.grid(True, alpha=0.4); ax.legend(loc="upper right", fontsize=9)
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "mx_norm_vs_time.png"), dpi=140)
    plt.close(fig)

    # ---- Figure 2: 3D trajectories (weak / medium / strong) ----
    agent_cmap = plt.get_cmap("tab10")
    fig = plt.figure(figsize=(15, 5.0))
    fig.suptitle("Formation trajectories under three noise levels with KF "
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
