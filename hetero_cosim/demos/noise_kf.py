"""Unity Demo 4 — Measurement noise WITH per-edge Kalman filter, live to Unity.

WHAT IT DOES
    Same as demo_hetero_noise.py, but each (follower, neighbor) edge runs a 3D
    random-walk Kalman filter on the noisy measurement before it enters the
    formation law. Run this at a sigma that is unstable without the KF
    (e.g. 0.5 or 1.0) and watch the formation stay together. Poses stream to
    Unity over UDP (240 Hz); obstacles loaded with collision detection.

    KF (per edge): state = neighbor pos in follower body frame; random-walk
    process with Q = (5 mm)^2; measurement noise R = sigma^2. As sigma grows,
    the Kalman gain shrinks and the filter suppresses noise harder.

    Live-visualization counterpart of experiment_hetero_noise_kf.py. Run side
    by side with demo_hetero_noise.py at the same HETERO_SIGMA to contrast.

HOW TO RUN
    1. Start the Unity receiver on UDP port 5006 (or set UNITY_PORT).
    2. HETERO_SIGMA=1.0 python -m hetero_cosim.demos.noise_kf

ENV VARS (see demo_runtime.py for the full list)
    HETERO_SIGMA        measurement-noise std in meters, default 1.0.
    HETERO_BASE_VEL     cruise velocity, default "0.5,0,0".
    HETERO_SEED         RNG seed for the noise, default 42.
    HETERO_STEPS / HETERO_HEADLESS / UNITY_UDP / HETERO_OBSTACLE_DIR  as usual.

OUTPUT
    None on disk — live Unity stream + PyBullet GUI. Collisions stop the run.
"""

import os
import numpy as np

from hetero_cosim.runtime import (
    IDX_UAV_LEADER, IDX_UGV, TOTAL_AGENTS,
    follower_local_frame_velocity, run_demo,
)

SIGMA = float(os.environ.get("HETERO_SIGMA", "1.0"))
_bv = os.environ.get("HETERO_BASE_VEL", "0.5,0,0")
BASE_VEL = np.array([float(x) for x in _bv.split(",")])
SEED = int(os.environ.get("HETERO_SEED", "42"))
KF_Q = (5e-3) ** 2

_rng = np.random.default_rng(SEED)

# Per-(follower, neighbor) KF state, lazy-initialised on first measurement.
_kf_x = np.zeros((TOTAL_AGENTS, TOTAL_AGENTS, 3))
_kf_P = np.zeros((TOTAL_AGENTS, TOTAL_AGENTS))
_kf_init = np.zeros((TOTAL_AGENTS, TOTAL_AGENTS), dtype=bool)
_R = SIGMA ** 2


def measurement_fn(agent_idx, nb, rel_local_truth):
    # Injection
    if SIGMA > 0:
        z = rel_local_truth + _rng.normal(0.0, SIGMA, size=3)
    else:
        z = rel_local_truth.copy()
    # Kalman update (scalar P, isotropic 3D)
    if not _kf_init[agent_idx, nb]:
        _kf_x[agent_idx, nb] = z.copy()
        _kf_P[agent_idx, nb] = _R + KF_Q
        _kf_init[agent_idx, nb] = True
        return z
    P_pred = _kf_P[agent_idx, nb] + KF_Q
    denom = P_pred + _R
    K = P_pred / denom if denom > 0 else 0.0
    _kf_x[agent_idx, nb] = _kf_x[agent_idx, nb] + K * (z - _kf_x[agent_idx, nb])
    _kf_P[agent_idx, nb] = (1.0 - K) * P_pred
    return _kf_x[agent_idx, nb]


def compute_velocities(M, real_x, agent_yaws, step_idx):
    v = np.zeros((TOTAL_AGENTS, 3))
    for ai in range(TOTAL_AGENTS):
        if ai in (IDX_UAV_LEADER, IDX_UGV):
            continue
        v[ai] = follower_local_frame_velocity(
            M, real_x, ai, agent_yaws, measurement_fn=measurement_fn)
    v += BASE_VEL[None, :]
    return v


if __name__ == "__main__":
    run_demo(compute_velocities,
             name=f"noise+KF (sigma={SIGMA}, v_ff={BASE_VEL.tolist()})")
