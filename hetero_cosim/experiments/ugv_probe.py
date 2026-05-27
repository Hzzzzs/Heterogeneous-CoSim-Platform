"""Utility — Calibrate the Husky UGV's effective wheel radius / speed tracking.

WHAT IT DOES
    Spawns a single Husky UGV (no UAVs, no formation feedback) on flat ground
    and drives it straight at a series of commanded velocities. For each
    command it reports the actual steady-state vx (measured from displacement
    in the second half of the run) and the ratio actual/commanded. Use this to
    verify the wheel_radius in tests/ugv_chassis_control.py is calibrated: a
    flat ratio near 1.0 across all speeds means the UGV tracks its velocity
    command with no systematic over/under-shoot.

    Background: the controller maps a desired chassis speed to wheel angular
    velocity via omega = v / wheel_radius. If wheel_radius is wrong, every
    commanded speed is off by the same constant ratio. The true geometric
    radius (0.17775 m) is read from the Husky URDF wheel cylinder.

HOW TO RUN
    python -m hetero_cosim.experiments.ugv_probe

CONFIG (edit constants near the top)
    TARGETS  list of commanded vx values (m/s) to sweep.
    STEPS    physics steps per command (default 2000 = ~8.3 s @ 240 Hz).

OUTPUTS  ->  experiment_ugv_probe/
    ugv_probe.csv   columns: target_vx, actual_vx, ratio
    (also printed as a table to stdout)
"""

import os
import time

import numpy as np
import pybullet as p
import pybullet_data

from hetero_cosim.ugv_chassis_control import UGVController

OUT = "experiment_ugv_probe"
STEPS = 2000        # 20 s per run
DT = 1.0 / 240.0    # PyBullet default
TARGETS = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.8, 1.0, 1.2, 1.5, 2.0]


def probe_one(target_vx):
    client = p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=client)
    p.setGravity(0, 0, -9.8, physicsClientId=client)
    p.loadURDF("plane.urdf", physicsClientId=client)
    ugv_id = p.loadURDF("husky/husky.urdf", basePosition=[0, 0, 0],
                        physicsClientId=client)
    ctrl = UGVController(ugv_id, client)

    pos_hist = np.zeros((STEPS, 3))
    target_vel_world = np.array([target_vx, 0.0, 0.0])

    for i in range(STEPS):
        pos, quat = p.getBasePositionAndOrientation(ugv_id, physicsClientId=client)
        pos_hist[i] = pos
        ctrl.compute_and_apply_control(quat, target_vel_world)
        p.stepSimulation(physicsClientId=client)

    p.disconnect(client)

    # Steady-state vx from the second half of the run
    t_half_idx = STEPS // 2
    dt_window = (STEPS - 1 - t_half_idx) * DT
    dx = pos_hist[-1, 0] - pos_hist[t_half_idx, 0]
    vx_steady = dx / dt_window
    return vx_steady


def main():
    os.makedirs(OUT, exist_ok=True)
    print(f"Probing UGV steady-state vx for {len(TARGETS)} target speeds...")
    print(f"{'target_vx':>10} | {'actual_vx':>10} | {'ratio':>6}")
    print(f"{'-'*10:>10} | {'-'*10:>10} | {'-'*6:>6}")
    rows = []
    for tv in TARGETS:
        t0 = time.time()
        actual = probe_one(tv)
        ratio = actual / tv if tv > 0 else float("nan")
        print(f"{tv:10.3f} | {actual:10.4f} | {ratio:6.3f}    ({time.time()-t0:.1f}s)")
        rows.append((tv, actual, ratio))

    out_csv = os.path.join(OUT, "ugv_probe.csv")
    with open(out_csv, "w") as f:
        f.write("target_vx,actual_vx,ratio\n")
        for tv, av, r in rows:
            f.write(f"{tv},{av},{r}\n")
    print(f"\n-> {out_csv}")


if __name__ == "__main__":
    main()
