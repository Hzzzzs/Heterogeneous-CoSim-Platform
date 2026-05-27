"""Unity Demo 2 — Common cruise velocity, streamed live to Unity.

WHAT IT DOES
    Real-time GUI demo of the moving formation: all agents get a common
    feedforward base_vel (default [0.5, 0, 0] m/s) and translate along +X
    while followers hold the relative geometry. Poses are streamed to Unity
    over UDP (240 Hz); obstacles in obstacles/ are loaded with collision
    detection (so the formation can be flown into the scene).

    Live-visualization counterpart of experiment_hetero_moving.py.

HOW TO RUN
    1. Start the Unity receiver on UDP port 5006 (or set UNITY_PORT).
    2. python -m hetero_cosim.demos.moving
       HETERO_BASE_VEL=0.2,0.2,0 python -m hetero_cosim.demos.moving   # diagonal cruise

ENV VARS (see demo_runtime.py for the full list)
    HETERO_BASE_VEL     "vx,vy,vz" cruise velocity, default "0.5,0,0".
    HETERO_STEPS        run length, default 6000 (~25 s).
    HETERO_HEADLESS=1   disable GUI + pacing.   UNITY_UDP=0  disable UDP.
    HETERO_OBSTACLE_DIR obstacle folder (default "obstacles", "" = none).

OUTPUT
    None on disk — live Unity stream + PyBullet GUI. Collisions stop the run.
"""

import os
import numpy as np

from hetero_cosim.runtime import (
    IDX_UAV_LEADER, IDX_UGV, TOTAL_AGENTS,
    follower_local_frame_velocity, run_demo,
)

_bv = os.environ.get("HETERO_BASE_VEL", "0.5,0,0")
BASE_VEL = np.array([float(x) for x in _bv.split(",")])
assert BASE_VEL.shape == (3,)


def compute_velocities(M, real_x, agent_yaws, step_idx):
    v = np.zeros((TOTAL_AGENTS, 3))
    for ai in range(TOTAL_AGENTS):
        if ai in (IDX_UAV_LEADER, IDX_UGV):
            continue
        v[ai] = follower_local_frame_velocity(M, real_x, ai, agent_yaws)
    v += BASE_VEL[None, :]
    return v


if __name__ == "__main__":
    run_demo(compute_velocities, name=f"moving (v_ff={BASE_VEL.tolist()})")
