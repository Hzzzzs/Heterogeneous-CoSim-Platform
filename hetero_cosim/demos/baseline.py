"""Unity Demo 1 — Stationary baseline, streamed live to Unity.

WHAT IT DOES
    Real-time GUI demo of the stationary heterogeneous formation (no cruise
    velocity, no noise). Followers converge to and hold the target geometry;
    leaders stay still. Every physics step the 4 agents' poses are pushed to
    Unity over UDP (240 Hz) for visualization, and obstacles in obstacles/
    are loaded with full collision detection.

    This is the live-visualization counterpart of experiment_hetero_baseline.py
    (which instead runs headless and produces CSV/figures). Same control law,
    different purpose: this one is for watching, not for measuring.

HOW TO RUN
    1. Start the Unity receiver listening on UDP port 5006 (or set UNITY_PORT).
    2. python -m hetero_cosim.demos.baseline

ENV VARS (see demo_runtime.py for the full list)
    HETERO_STEPS        run length, default 6000 (~25 s).
    HETERO_HEADLESS=1   disable PyBullet GUI + real-time pacing.
    UNITY_UDP=0         disable UDP send (e.g. quick local test).
    UNITY_HOST / UNITY_PORT  override Unity target (auto-resolved otherwise).
    HETERO_OBSTACLE_DIR override obstacle folder (default "obstacles", "" = none).

OUTPUT
    None on disk — this demo only streams poses to Unity and renders the
    PyBullet GUI. Mid-air / obstacle collisions stop the run with a message.
"""

import numpy as np

from hetero_cosim.runtime import (
    IDX_UAV_LEADER, IDX_UGV, TOTAL_AGENTS,
    follower_local_frame_velocity, run_demo,
)


def compute_velocities(M, real_x, agent_yaws, step_idx):
    v = np.zeros((TOTAL_AGENTS, 3))
    for ai in range(TOTAL_AGENTS):
        if ai in (IDX_UAV_LEADER, IDX_UGV):
            continue
        v[ai] = follower_local_frame_velocity(M, real_x, ai, agent_yaws)
    return v


if __name__ == "__main__":
    run_demo(compute_velocities, name="baseline (stationary)")
