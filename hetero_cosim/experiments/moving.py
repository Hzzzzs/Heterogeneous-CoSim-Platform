"""Experiment 2 — Common cruise velocity: Sim vs HIL with a moving formation.

WHAT IT DOES
    Extends Experiment 1: instead of stationary leaders, ALL four agents
    receive a common global velocity feedforward base_vel (default
    [0.5, 0, 0] m/s), so the whole formation translates along +X while the
    followers maintain the relative geometry via the per-follower local-frame
    law. Demonstrates that the z-similar control law is invariant to common
    translation: the formation error ||Mx|| stays at the mm level even though
    the absolute positions sweep across meters.

    Control law (per-follower i):
        u_i^local = Σ_j M_ij · R_z(-yaw_i) (x_j − x_i)      (formation feedback)
        u_i^world = R_z(+yaw_i) u_i^local
        v_i^cmd   = u_i^world + base_vel                    (+ common feedforward)
    Leaders (UAV_L, UGV) receive only base_vel (no feedback term).
    Sim baseline: ODE dx/dt = M x + v_ff, v_ff = tile(base_vel, 4).

HOW TO RUN
    python -m hetero_cosim.experiments.moving                       # default 0.5 m/s
    HETERO_BASE_VEL=0.3,0,0 python -m hetero_cosim.experiments.moving
    HETERO_MOVING_OUT=my_run HETERO_BASE_VEL=0.2,0,0 python -m hetero_cosim.experiments.moving
    The HIL trajectory is cached in <out>/hil.csv; delete it to re-run HIL.

ENV VARS
    HETERO_BASE_VEL   "vx,vy,vz" cruise velocity (default "0.5,0,0").
    HETERO_MOVING_OUT output directory (default "experiment_hetero_moving").

OUTPUTS  ->  <HETERO_MOVING_OUT>/
    hil.csv                       cached HIL trajectory (6000 x 12, 25 s)
    metrics.txt                   ||Mx||, mean vx of UAV_L & UGV, per-agent gap
    figures/mx_norm_vs_time.png   ||Mx|| convergence, Sim vs HIL
    figures/formation_trajectories.png  3D overlay with HIL formation edges
"""

import os
import time

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pybullet as p
import pybullet_data
from scipy.integrate import solve_ivp

from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
from gym_pybullet_drones.utils.enums import DroneModel, Physics

from hetero_cosim.formations.hetero_ugv import get_dynamics_and_init
from hetero_cosim.ugv_chassis_control import UGVController
from hetero_cosim.unity_bridge import UnityBridge

OUT = os.environ.get("HETERO_MOVING_OUT", "experiment_hetero_moving")
FIG = os.path.join(OUT, "figures")
HIL_CSV = os.path.join(OUT, "hil.csv")

STEPS = 6000
# DT below is just the lookahead used inside the per-step formation update
# (target_pos = cur_pos + DT * v_cmd) — it does NOT determine real time.
# Real per-step physics time is set by env.PYB_TIMESTEP (default 1/240 s);
# we read it back from the env after construction and use it for time axes,
# velocity averages, and the ODE sim horizon — the previous version used
# DT here and that gave a 60-s vs 25-s mismatch with PyBullet.
LOOKAHEAD_DT = 0.001
_bv = os.environ.get("HETERO_BASE_VEL", "0.5,0,0")
BASE_VEL = np.array([float(x) for x in _bv.split(",")])
assert BASE_VEL.shape == (3,), f"HETERO_BASE_VEL must be 3 comma-separated floats, got {_bv}"

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


def run_hil(M, x0, steps=STEPS, base_vel=BASE_VEL):
    """PyBullet HIL with per-follower local-frame feedback + common feedforward
    base_vel applied to ALL agents. Returns pos_history of shape
    (steps, TOTAL_AGENTS*3) in virtual-state column order.
    """
    uav_init_xyzs = np.vstack([x0[0:3], x0[6:9], x0[9:12]])
    ugv_init_xyz  = x0[3:6]

    env = CtrlAviary(drone_model=DroneModel.CF2X, num_drones=NUM_UAVS,
                     initial_xyzs=uav_init_xyzs, physics=Physics.PYB, gui=False)
    ctrls = [DSLPIDControl(drone_model=DroneModel.CF2X) for _ in range(NUM_UAVS)]

    p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=env.CLIENT)
    ugv_id = p.loadURDF("husky/husky.urdf", basePosition=ugv_init_xyz,
                        physicsClientId=env.CLIENT)
    ugv_controller = UGVController(ugv_id, env.CLIENT)

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

            # Formation feedback for followers (leaders get zero feedback)
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
                    dp_world  = real_x[3*nb:3*nb+3] - xi
                    rel_local = R_zi_inv @ dp_world
                    v_local += M_ij @ rel_local
                velocities_matrix[agent_idx] = R_zi @ v_local

            # Common feedforward: all agents get base_vel (including leaders)
            velocities_matrix += base_vel[None, :]

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


def sim_ode(M, x0, base_vel, total_time, n_samples=2001):
    """ODE sim of the closed-loop formation feedback + common feedforward:
        dx/dt = M x + v_ff,   v_ff = tile(base_vel, TOTAL_AGENTS)
    Integrates from t=0 to t=total_time so the HIL and Sim time horizons match.
    """
    v_ff = np.tile(base_vel, TOTAL_AGENTS)
    t = np.linspace(0.0, total_time, n_samples)
    sol = solve_ivp(lambda _t, x: M @ x + v_ff,
                    (t[0], t[-1]), x0, t_eval=t,
                    method="RK45", rtol=1e-6, atol=1e-9, max_step=0.05)
    if not sol.success:
        raise RuntimeError(sol.message)
    return sol.y.T, t


def main():
    os.makedirs(OUT, exist_ok=True)
    os.makedirs(FIG, exist_ok=True)

    M, x0 = get_dynamics_and_init()

    if os.path.exists(HIL_CSV):
        print(f"[skip] HIL CSV exists: {HIL_CSV}")
    else:
        print(f"[run] HIL headless -> {HIL_CSV}")
        t0 = time.time()
        hil = run_hil(M, x0)
        np.savetxt(HIL_CSV, hil, delimiter=",", fmt="%.6f")
        print(f"       done in {time.time()-t0:.1f}s")
    hil = np.loadtxt(HIL_CSV, delimiter=",")

    # Real per-step physics time used by CtrlAviary (default 1/240 s).
    # We re-derive it here without re-creating the env: it is fixed at 1/240
    # by gym_pybullet_drones defaults (pyb_freq=ctrl_freq=240).
    PYB_DT = 1.0 / 240.0
    total_time = (hil.shape[0] - 1) * PYB_DT
    t_hil = np.arange(hil.shape[0]) * PYB_DT

    # ODE sim integrates over the same physical time as HIL.
    sim, t_sim = sim_ode(M, hil[0].copy(), base_vel=BASE_VEL,
                         total_time=total_time, n_samples=hil.shape[0])

    # Metrics — same physical time on both sides
    norm_sim_final = float(np.linalg.norm(M @ sim[-1]))
    norm_hil_final = float(np.linalg.norm(M @ hil[-1]))
    endpoint_dist  = float(np.linalg.norm(hil[-1] - sim[-1]))
    uavl_traj = hil[:, 0:3]
    ugv_traj  = hil[:, 3:6]
    uavl_mean_vx = float((uavl_traj[-1,0] - uavl_traj[0,0]) / total_time)
    ugv_mean_vx  = float((ugv_traj[-1,0]  - ugv_traj[0,0])  / total_time)

    print(f"STEPS={hil.shape[0]}, PYB_DT={PYB_DT:.6f}s, total_time={total_time:.3f}s")
    print(f"||M x_sim(T)||   = {norm_sim_final:.3e}")
    print(f"||M x_HIL(T)||  = {norm_hil_final:.3e}")
    print(f"|| x_HIL(T) - x_sim(T) || = {endpoint_dist:.3e}")
    print(f"UAV_L mean vx = {uavl_mean_vx:.4f} m/s   (target: {BASE_VEL[0]:.4f})")
    print(f"UGV   mean vx = {ugv_mean_vx:.4f} m/s   (target: {BASE_VEL[0]:.4f})")

    # ---- Figure 1: ||Mx|| convergence (linear) ----
    Mx_sim = sim @ M.T
    Mx_hil = hil @ M.T
    norm_sim = np.linalg.norm(Mx_sim, axis=1)
    norm_hil = np.linalg.norm(Mx_hil, axis=1)

    fig, ax = plt.subplots(figsize=(8, 4.2))
    ax.plot(t_sim, norm_sim, "-", color=(0.10, 0.50, 0.85),
            linewidth=1.6, label="Sim (ODE, dx/dt = Mx + v_ff)")
    ax.plot(t_hil, norm_hil, "--", color=(0.85, 0.45, 0.10),
            linewidth=1.6, label="HIL (PyBullet, no noise)")
    ax.set_xlabel("t (s)"); ax.set_ylabel(r"$\|Mx\|_2$")
    ax.set_title(r"$\|Mx\|$ convergence with common cruise velocity "
                 f"$v_{{ff}}={BASE_VEL.tolist()}$")
    ax.grid(True, alpha=0.4); ax.legend(loc="upper right", fontsize=9)
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "mx_norm_vs_time.png"), dpi=140)
    plt.close(fig)

    # ---- Figure 2: 3D trajectories overlay with HIL final graph edges ----
    cmap = plt.get_cmap("tab10")
    fig = plt.figure(figsize=(10, 7.2))
    ax = fig.add_subplot(111, projection="3d")
    for j in range(TOTAL_AGENTS):
        c = cmap(j)
        ax.plot(sim[:, 3*j], sim[:, 3*j+1], sim[:, 3*j+2],
                "-", color=c, linewidth=1.6, label=f"{LABELS[j]} Sim")
        ax.plot(hil[:, 3*j], hil[:, 3*j+1], hil[:, 3*j+2],
                "--", color=c, linewidth=1.2, alpha=0.85,
                label=f"{LABELS[j]} HIL")
        ax.plot(sim[0, 3*j], sim[0, 3*j+1], sim[0, 3*j+2],
                "o", color=c, markerfacecolor="white", markersize=6)
        ax.plot(sim[-1, 3*j], sim[-1, 3*j+1], sim[-1, 3*j+2],
                "*", color=c, markersize=13,
                markeredgecolor="black", markeredgewidth=0.5)
        ax.plot(hil[-1, 3*j], hil[-1, 3*j+1], hil[-1, 3*j+2],
                "D", color=c, markerfacecolor="white", markersize=7)
    # HIL final communication-graph edges
    hil_final = hil[-1].reshape(TOTAL_AGENTS, 3)
    for a, b in PAIRS:
        ax.plot([hil_final[a, 0], hil_final[b, 0]],
                [hil_final[a, 1], hil_final[b, 1]],
                [hil_final[a, 2], hil_final[b, 2]],
                "-", color=(0.25, 0.25, 0.25), linewidth=1.6, alpha=0.85)
    ax.set_xlabel("X"); ax.set_ylabel("Y"); ax.set_zlabel("Z")
    ax.set_title("Sim (solid, $\\star$) vs HIL (dashed, $\\diamond$) "
                 f"\N{EM DASH} common cruise $v_{{ff}}$={BASE_VEL.tolist()}")
    ax.legend(loc="upper left", bbox_to_anchor=(1.02, 1.0), fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "formation_trajectories.png"), dpi=140)
    plt.close(fig)

    # ---- Metrics file ----
    with open(os.path.join(OUT, "metrics.txt"), "w") as f:
        f.write(f"base_vel = {BASE_VEL.tolist()}\n")
        f.write(f"STEPS = {hil.shape[0]}, PYB_DT = {PYB_DT:.6f}s, total_time = {total_time:.3f}s\n\n")
        f.write(f"||M x_sim(T)||_2  = {norm_sim_final:.6e}\n")
        f.write(f"||M x_HIL(T)||_2 = {norm_hil_final:.6e}\n")
        f.write(f"|| x_HIL(T) - x_sim(T) ||_2 = {endpoint_dist:.6e}\n\n")
        f.write(f"UAV_L mean vx = {uavl_mean_vx:.4f} m/s   (target: {BASE_VEL[0]:.4f})\n")
        f.write(f"UGV   mean vx = {ugv_mean_vx:.4f}  m/s   (target: {BASE_VEL[0]:.4f})\n\n")
        for j in range(TOTAL_AGENTS):
            d = np.linalg.norm(hil[-1, 3*j:3*j+3] - sim[-1, 3*j:3*j+3])
            f.write(f"  per-agent {LABELS[j]}: {d:.6e}\n")
    print(f"figures -> {FIG}/")


if __name__ == "__main__":
    main()
