"""Unity Demo 3 — Measurement noise (no Kalman), streamed live to Unity.

WHAT IT DOES
    Real-time GUI demo of the moving formation with Gaussian noise injected on
    every follower's per-edge relative-position measurement, NO filtering. Lets
    you watch the formation degrade as sigma rises (try 0.15 -> 0.30 -> 0.50).
    Poses stream to Unity over UDP (240 Hz); obstacles loaded with collision
    detection.

    Live-visualization counterpart of experiment_hetero_noise.py.
    Pair with demo_hetero_noise_kf.py to see the same sigma with the KF on.

HOW TO RUN
    1. Start the Unity receiver on UDP port 5006 (or set UNITY_PORT).
    2. HETERO_SIGMA=0.30 python -m hetero_cosim.demos.noise

ENV VARS (see demo_runtime.py for the full list)
    HETERO_SIGMA        measurement-noise std in meters, default 0.5.
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

SIGMA = float(os.environ.get("HETERO_SIGMA", "0.5"))
_bv = os.environ.get("HETERO_BASE_VEL", "0.5,0,0")
BASE_VEL = np.array([float(x) for x in _bv.split(",")])
SEED = int(os.environ.get("HETERO_SEED", "42"))

_rng = np.random.default_rng(SEED)


def measurement_fn(agent_idx, nb, rel_local_truth):
    if SIGMA <= 0:
        return rel_local_truth
    return rel_local_truth + _rng.normal(0.0, SIGMA, size=3)


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
             name=f"noise (sigma={SIGMA}, v_ff={BASE_VEL.tolist()})")
