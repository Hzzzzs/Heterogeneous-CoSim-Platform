"""Headless no-noise sim-vs-HIL comparison for the heterogeneous
3-UAV + 1-UGV formation, matching run_hetero_three_uav_one_ugv_new.py.

Sim baseline: ODE integration of dx/dt = M x on the 12-dim virtual state
(node order [UAV_leader, UGV, UAV_F1, UAV_F2]). Same M and x0 as HIL.

Outputs go to experiment_hetero_baseline/.
"""

import os
import subprocess
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.integrate import solve_ivp

# Import the dynamics builder from the _new script
from run_hetero_three_uav_one_ugv_new import get_dynamics_and_init

OUT = "experiment_hetero_baseline"
FIG = os.path.join(OUT, "figures")
HIL_CSV = os.path.join(OUT, "hil_sigma0.csv")
STEPS = 10000
DT = 0.01
NUM_AGENTS = 4
LABELS = ["UAV_L", "UGV", "UAV_F1", "UAV_F2"]
# Communication edges (same as the noise sweep): 5 edges, no UAV_L↔UGV link.
PAIRS = [(0, 2), (0, 3), (1, 2), (1, 3), (2, 3)]


def run_hil_if_needed():
    """Invoke the HIL script headless to produce hil_sigma0.csv."""
    os.makedirs(OUT, exist_ok=True)
    if os.path.exists(HIL_CSV):
        print(f"[skip] HIL CSV exists: {HIL_CSV}")
        return
    env = os.environ.copy()
    env["HETERO_HEADLESS"] = "1"
    env["HETERO_UDP"]      = "0"
    env["HETERO_TREE"]     = "0"
    env["HETERO_OUT"]      = HIL_CSV
    env["HETERO_STEPS"]    = str(STEPS)
    print(f"[run] HIL headless -> {HIL_CSV}")
    subprocess.check_call([sys.executable, "run_hetero_three_uav_one_ugv_new.py"], env=env)


def sim_ode(M, x0, steps=STEPS, dt=DT):
    t = np.arange(0, steps + 1) * dt
    sol = solve_ivp(lambda _t, x: M @ x, (t[0], t[-1]), x0,
                    t_eval=t, method="RK45",
                    rtol=1e-6, atol=1e-9, max_step=0.05)
    if not sol.success:
        raise RuntimeError(sol.message)
    return sol.y.T, t


def main():
    os.makedirs(FIG, exist_ok=True)
    run_hil_if_needed()

    M, x0 = get_dynamics_and_init()
    hil = np.loadtxt(HIL_CSV, delimiter=",")
    if hil.shape != (STEPS, NUM_AGENTS * 3):
        print(f"[warn] HIL shape {hil.shape} unexpected; using what we have.")
    t_hil = np.arange(hil.shape[0]) * DT

    # Use the HIL row 0 as x0 for the ODE sim so the starting point matches
    # exactly (the HIL script initializes deterministic positions, but
    # PyBullet may report tiny offsets after init).
    x0_from_hil = hil[0].copy()
    sim, t_sim = sim_ode(M, x0_from_hil, steps=STEPS, dt=DT)
    # Also a fresh-x0 ODE (the "ideal") for reference
    sim_ideal, _ = sim_ode(M, x0, steps=STEPS, dt=DT)

    # ||Mx|| curves
    Mx_sim = sim @ M.T
    Mx_hil = hil @ M.T
    norm_sim = np.linalg.norm(Mx_sim, axis=1)
    norm_hil = np.linalg.norm(Mx_hil, axis=1)

    # Endpoint comparison
    endpoint_dist = float(np.linalg.norm(hil[-1] - sim[hil.shape[0]-1]))
    norm_sim_final = float(np.linalg.norm(M @ sim[-1]))
    norm_hil_final = float(np.linalg.norm(M @ hil[-1]))
    print(f"||M x_sim(T)||   = {norm_sim_final:.3e}")
    print(f"||M x_HIL(T)||  = {norm_hil_final:.3e}")
    print(f"|| x_HIL(T) - x_sim(T) || = {endpoint_dist:.3e}")

    # ---- Figure 1: ||Mx|| convergence (linear) ----
    fig, ax = plt.subplots(figsize=(8, 4.2))
    ax.plot(t_sim, norm_sim, "-", color=(0.10, 0.50, 0.85),
            linewidth=1.6, label="Sim (ODE dx/dt = Mx)")
    ax.plot(t_hil, norm_hil, "--", color=(0.85, 0.45, 0.10),
            linewidth=1.6, label="HIL (PyBullet, no noise)")
    ax.set_xlabel("t (s)"); ax.set_ylabel(r"$\|Mx\|_2$")
    ax.set_title("Heterogeneous formation \N{EM DASH} $\|Mx\|$ convergence (no noise)")
    ax.grid(True, alpha=0.4); ax.legend(loc="upper right", fontsize=9)
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "mx_convergence.png"), dpi=140)
    plt.close(fig)

    # ---- Figure 2: 3D trajectories overlay ----
    cmap = plt.get_cmap("tab10")
    fig = plt.figure(figsize=(9, 7))
    ax = fig.add_subplot(111, projection="3d")
    for j in range(NUM_AGENTS):
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
    # Draw the 5 communication edges over the HIL final positions
    hil_final = hil[-1].reshape(NUM_AGENTS, 3)
    for a, b in PAIRS:
        ax.plot([hil_final[a, 0], hil_final[b, 0]],
                [hil_final[a, 1], hil_final[b, 1]],
                [hil_final[a, 2], hil_final[b, 2]],
                "-", color=(0.25, 0.25, 0.25), linewidth=1.6, alpha=0.85)
    ax.set_xlabel("X"); ax.set_ylabel("Y"); ax.set_zlabel("Z")
    ax.set_title("Sim (solid, $\\star$) vs HIL (dashed, $\\diamond$) \N{EM DASH} no noise")
    ax.legend(loc="upper left", bbox_to_anchor=(1.02, 1.0), fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "trajectories_3d.png"), dpi=140)
    plt.close(fig)

    # Metrics file
    with open(os.path.join(OUT, "metrics.txt"), "w") as f:
        f.write(f"||M x_sim(T)||_2  = {norm_sim_final:.6e}\n")
        f.write(f"||M x_HIL(T)||_2 = {norm_hil_final:.6e}\n")
        f.write(f"|| x_HIL(T) - x_sim(T) ||_2 = {endpoint_dist:.6e}\n")
        for j in range(NUM_AGENTS):
            d = np.linalg.norm(hil[-1, 3*j:3*j+3] - sim[hil.shape[0]-1, 3*j:3*j+3])
            f.write(f"  per-agent {LABELS[j]}: {d:.6e}\n")
    print(f"figures -> {FIG}/")


if __name__ == "__main__":
    main()
