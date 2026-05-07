"""Headless HIL sim with Gaussian position-measurement noise.

Noise is injected the same way a real sensor would be noisy: when the
controller reads the drone's position, we add N(0, sigma^2) independently
per axis per timestep. The drone's actual physical position (what we store
in the CSV) is the real PyBullet state, so the CSV still reports ground
truth; what changes is only what the controller thinks it sees.
"""

import argparse
import os
import time
import numpy as np

from run_threesimilar_headless import get_dynamics

from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
from gym_pybullet_drones.utils.enums import DroneModel, Physics


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sigma", type=float, default=0.02,
                    help="std of Gaussian position noise (metres, per axis)")
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--out", type=str, default="drone_trajectories_noisy.csv")
    ap.add_argument("--steps", type=int, default=10000)
    args = ap.parse_args()

    rng = np.random.default_rng(args.seed)
    M_sys, num_drones = get_dynamics()

    x0 = np.array([
        0.0, 0.0, 2.0,   # L1
        1.0, 0.0, 1.0,   # L2
        0.5, 1.0, 1.5,   # F3
        1.5, 0.5, 1.5,   # F4
    ])
    init_xyzs = x0.reshape((num_drones, 3))
    print(f"initial x0 (fixed geometric layout):\n{init_xyzs}")
    print(f"position noise sigma = {args.sigma} m, seed = {args.seed}")

    env = CtrlAviary(drone_model=DroneModel.CF2X,
                     num_drones=num_drones,
                     initial_xyzs=init_xyzs,
                     physics=Physics.PYB,
                     gui=False)
    ctrls = [DSLPIDControl(drone_model=DroneModel.CF2X) for _ in range(num_drones)]

    formation_dt = 0.01
    TOTAL_STEPS = args.steps
    pos_history = np.zeros((TOTAL_STEPS, num_drones, 3))
    steps_run = 0

    t0 = time.time()
    print(f"starting noisy HIL sim, {TOTAL_STEPS} steps ...")

    try:
        for i in range(TOTAL_STEPS):
            steps_run = i + 1
            obs_multi = [env._getDroneStateVector(j) for j in range(num_drones)]

            # ground-truth positions
            real_x = np.zeros(num_drones * 3)
            for j in range(num_drones):
                real_x[j*3:j*3+3] = obs_multi[j][0:3]

            # noisy position seen by the controller
            noise = rng.normal(0.0, args.sigma, size=real_x.shape)
            noisy_x = real_x + noise

            # Noise enters ONLY through the formation-law velocity, not through
            # the PID target's base position. This mirrors the Sim update
            #   x[k+1] = x[k] + dt * M * (x[k] + n)
            # so the two pipelines are structurally symmetric. In particular
            # leader rows of M are zero, hence v_leader = 0, hence
            # target_leader = real_leader -> leader does not drift under noise.
            velocities = M_sys @ noisy_x
            targets = (real_x + formation_dt * velocities).reshape((num_drones, 3))
            target_vels = velocities.reshape((num_drones, 3))

            action_matrix = np.zeros((num_drones, 4))
            for j in range(num_drones):
                # PID still gets the real state (it's the onboard IMU/EKF here
                # that is already fused). We only corrupt the position that
                # goes into the formation law, which is the "where am I" signal
                # the high-level controller consumes.
                action, _, _ = ctrls[j].computeControlFromState(
                    control_timestep=env.CTRL_TIMESTEP,
                    state=obs_multi[j],
                    target_pos=targets[j],
                    target_rpy=np.array([0, 0, 0]),
                    target_vel=target_vels[j],
                )
                action_matrix[j] = action
                pos_history[i, j, :] = obs_multi[j][0:3]  # store ground truth

            env.step(action_matrix)

            if (i + 1) % 1000 == 0:
                elapsed = time.time() - t0
                mx = np.linalg.norm(M_sys @ real_x)
                print(f"  step {i+1:5d}/{TOTAL_STEPS}  |  wall {elapsed:5.1f}s  "
                      f"|  ||M*x_real|| = {mx:.4e}")

    except KeyboardInterrupt:
        print("\ninterrupted")
    finally:
        env.close()
        csv_data = pos_history[:steps_run].reshape(steps_run, num_drones * 3)
        np.savetxt(args.out, csv_data, delimiter=",", fmt="%.6f")
        print(f"saved {steps_run} steps to {args.out} "
              f"(wall {time.time()-t0:.1f}s)")


if __name__ == "__main__":
    main()
