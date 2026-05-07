"""Headless re-run of run_threesimilar.py for HIL data collection.

Key differences vs run_threesimilar.py:
  - gui=False (no PyBullet window)
  - no UDP send to Unity
  - no per-step sleep (runs as fast as the CPU allows)
  - initial follower positions loaded from the original CSV's first row so
    the scenario matches the senior's previous run exactly
  - periodic progress print
"""

import os
import time
import numpy as np
from numpy.linalg import det, eig

from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
from gym_pybullet_drones.utils.enums import DroneModel, Physics


def get_dynamics():
    n = 4
    L = np.array([
        [0, 0, 0, 0],
        [0, 0, 0, 0],
        [1, 1, -3 - np.sqrt(2), 1 + np.sqrt(2)],
        [1, 1, 1, -3],
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
    lam = eig(np.diag([e[0]]) @ A[0:1, 0:1])[0]
    e_diag = np.array([e[0]], dtype=complex)
    epsilon = 0.1
    for i in range(1, m_dim):
        e_prod = np.prod(e[:i])
        lam_prod = np.prod(lam[:i])
        val = e_prod * det(A[: i + 1, : i + 1]) / lam_prod
        e[i] = epsilon * np.exp(-1j * np.angle(val))
        e_diag = np.append(e_diag, e[i])
        lam = eig(np.diag(e_diag) @ A[: i + 1, : i + 1])[0]
    e = -e

    def R(phi):
        return np.array([[np.cos(phi), -np.sin(phi), 0],
                         [np.sin(phi),  np.cos(phi), 0],
                         [0, 0, 1]])

    R1 = R(np.angle(e[0]))
    R2 = R(np.angle(e[1]))
    D = np.zeros((12, 12))
    D[:6, :6] = 0.1 * np.eye(6)
    D[6:9, 6:9] = np.abs(e[0]) * R1
    D[9:12, 9:12] = np.abs(e[1]) * R2

    R31, R32, R34 = R(theta31), R(theta32), R(theta34)
    R41, R42, R43 = R(theta41), R(theta42), R(theta43)
    LL = np.zeros((12, 12))
    LL[6:9, 0:3] = L[2, 0] * R31
    LL[6:9, 3:6] = L[2, 1] * R32
    LL[6:9, 6:9] = -(L[2, 0] * R31 + L[2, 1] * R32 + L[2, 3] * R34)
    LL[6:9, 9:12] = L[2, 3] * R34
    LL[9:12, 0:3] = L[3, 0] * R41
    LL[9:12, 3:6] = L[3, 1] * R42
    LL[9:12, 6:9] = L[3, 2] * R43
    LL[9:12, 9:12] = -(L[3, 0] * R41 + L[3, 1] * R42 + L[3, 2] * R43)
    return D @ LL, n


def main():
    M_sys, num_drones = get_dynamics()

    # Fixed geometric initial layout so sim and HIL share the same x0
    # (leaders spaced apart, followers in the air, no randomness).
    x0 = np.array([
        0.0, 0.0, 2.0,   # L1
        1.0, 0.0, 1.0,   # L2
        0.5, 1.0, 1.5,   # F3
        1.5, 0.5, 1.5,   # F4
    ])
    init_xyzs = x0.reshape((num_drones, 3))
    print(f"initial x0 (fixed geometric layout):\n{init_xyzs}")

    env = CtrlAviary(drone_model=DroneModel.CF2X,
                     num_drones=num_drones,
                     initial_xyzs=init_xyzs,
                     physics=Physics.PYB,
                     gui=False)
    ctrls = [DSLPIDControl(drone_model=DroneModel.CF2X) for _ in range(num_drones)]

    formation_dt = 0.01
    TOTAL_STEPS = 10000
    pos_history = np.zeros((TOTAL_STEPS, num_drones, 3))
    steps_run = 0

    t0 = time.time()
    print(f"starting headless HIL sim, {TOTAL_STEPS} steps ...")

    try:
        for i in range(TOTAL_STEPS):
            steps_run = i + 1
            obs_multi = [env._getDroneStateVector(j) for j in range(num_drones)]

            real_x = np.zeros(num_drones * 3)
            for j in range(num_drones):
                real_x[j*3:j*3+3] = obs_multi[j][0:3]

            velocities = M_sys @ real_x
            targets = (real_x + formation_dt * velocities).reshape((num_drones, 3))
            target_vels = velocities.reshape((num_drones, 3))

            action_matrix = np.zeros((num_drones, 4))
            for j in range(num_drones):
                action, _, _ = ctrls[j].computeControlFromState(
                    control_timestep=env.CTRL_TIMESTEP,
                    state=obs_multi[j],
                    target_pos=targets[j],
                    target_rpy=np.array([0, 0, 0]),
                    target_vel=target_vels[j],
                )
                action_matrix[j] = action
                pos_history[i, j, :] = obs_multi[j][0:3]

            env.step(action_matrix)

            if (i + 1) % 1000 == 0:
                elapsed = time.time() - t0
                mx = np.linalg.norm(M_sys @ real_x)
                print(f"  step {i+1:5d}/{TOTAL_STEPS}  |  wall {elapsed:6.1f}s  |  ||M*x|| = {mx:.4e}")

    except KeyboardInterrupt:
        print("\ninterrupted")
    finally:
        env.close()
        csv_data = pos_history[:steps_run].reshape(steps_run, num_drones * 3)
        np.savetxt("drone_trajectories.csv", csv_data, delimiter=",", fmt="%.6f")
        print(f"saved {steps_run} steps to drone_trajectories.csv "
              f"(wall time {time.time()-t0:.1f}s)")


if __name__ == "__main__":
    main()
