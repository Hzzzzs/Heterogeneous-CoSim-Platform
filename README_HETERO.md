# Heterogeneous CoSim Platform

A PyBullet-based co-simulation platform for **heterogeneous multi-agent formation control**, combining 3 UAVs (Crazyflie CF2X) and 1 UGV (Clearpath Husky). Built on top of [gym-pybullet-drones](https://github.com/utiasDSL/gym-pybullet-drones).

## Features

- **Distributed formation control** using complex-Laplacian-based consensus law
- **Per-follower local-frame control**: each follower computes its velocity from body-frame relative measurements — no central observer required
- **Common cruise velocity feedforward**: the whole formation translates at a fixed heading while maintaining geometry
- **Per-edge Kalman filter**: 3D random-walk KF on each inter-agent measurement to reject sensor noise
- **UGV differential-drive kinematics**: Husky chassis controller with heading alignment and torque-saturated motor commands
- **Noise robustness experiments**: sigma sweep over Gaussian measurement noise, with and without KF

## Project Structure

```
.
├── run_hetero_three_uav_one_ugv_new.py   # Main runner — 3 UAV + 1 UGV formation (recommended)
├── run_hetero_three_uav_one_ugv.py       # Earlier version (centralized reference)
├── run_sim_to_unity.py                   # Stream pose data to Unity via UDP
├── run_threesimilar_new.py               # 3 homogeneous UAVs (for baseline comparison)
├── run_threesimilar_noisy.py             # Homogeneous UAVs with noise injection
├── main_hetero_sim.py                    # Headless batch runner for HIL sweep
├── main_simulation.py                    # General simulation entry point
│
├── experiment_hetero_baseline.py         # Exp 1: Sim vs HIL, stationary formation
├── experiment_hetero_moving.py           # Exp 2: Sim vs HIL, with cruise velocity
├── experiment_hetero_noise.py            # Exp 3: Noise robustness sweep (stationary)
├── experiment_hetero_noise_kf.py         # Exp 4: Noise robustness sweep + Kalman filter
├── experiment_ugv_probe.py               # UGV speed characterisation
│
└── tests/
    └── ugv_chassis_control.py            # UGVController class (differential drive)
```

## Dependencies

This project extends [gym-pybullet-drones](https://github.com/utiasDSL/gym-pybullet-drones). Install that first:

```bash
git clone https://github.com/utiasDSL/gym-pybullet-drones.git
cd gym-pybullet-drones
pip install -e .
```

Additional dependencies:

```bash
pip install numpy scipy matplotlib pybullet
```

## Quick Start

### Run the heterogeneous formation (GUI)

```bash
python run_hetero_three_uav_one_ugv_new.py
```

This spawns 3 CF2X drones and 1 Husky UGV in PyBullet GUI. The formation converges to the target geometry and (optionally) cruises along +X.

Key environment variables:

| Variable | Default | Description |
|---|---|---|
| `HETERO_HEADLESS` | `0` | Set `1` to disable GUI |
| `HETERO_STEPS` | `10000` | Number of simulation steps |
| `HETERO_OUT` | _(none)_ | Path to save trajectory CSV |
| `HETERO_UDP` | `0` | Set `1` to stream poses to Unity |

Example — headless run saving trajectory:

```bash
HETERO_HEADLESS=1 HETERO_STEPS=6000 HETERO_OUT=my_traj.csv python run_hetero_three_uav_one_ugv_new.py
```

### Run Sim vs HIL baseline comparison

```bash
python experiment_hetero_baseline.py
```

Outputs figures to `experiment_hetero_baseline/figures/`.

### Run formation with cruise velocity

```bash
python experiment_hetero_moving.py
# or with custom speed:
HETERO_BASE_VEL=0.3,0,0 HETERO_MOVING_OUT=my_output python experiment_hetero_moving.py
```

### Run noise robustness sweep (with Kalman filter)

```bash
python experiment_hetero_noise_kf.py
```

Sweeps `sigma` in `[0, 0.02, 0.05, 0.10, 0.15, 0.20, 0.30, 0.50]` m. Results saved to `hetero_noise_moving_kf/`.

To run the version **without** KF (for comparison):

```bash
python experiment_hetero_noise.py
```

### Stream to Unity

```bash
HETERO_UDP=1 python run_hetero_three_uav_one_ugv_new.py
```

Pose data is broadcast over UDP. Set the target IP/port inside `run_sim_to_unity.py`.

## Formation Setup

**Node order** (12-dim virtual state `[UAV_L, UGV, UAV_F1, UAV_F2]`):

| Index | Agent | Initial position | Role |
|---|---|---|---|
| 0 | UAV_leader | (0, 0, 2) | Leader — zero feedback, cruise only |
| 1 | UGV | (1, 0, 0) | Leader — zero feedback, cruise only |
| 2 | UAV_F1 | (0.5, 1, 1.5) | Follower |
| 3 | UAV_F2 | (1.5, 0.5, 1.5) | Follower |

**Communication graph** (5 edges): `(L1,F1), (L1,F2), (UGV,F1), (UGV,F2), (F1,F2)`

## Control Law

Each follower $i$ computes its velocity command in its own body-yaw frame:

```
u_local = Σ_j M_ij · KF(R_z(-ψ_i)(x_j − x_i) + n_ij)
v_cmd   = R_z(+ψ_i) · u_local  +  v_ff
```

where:
- `M` is the complex-Laplacian gain matrix (built in `get_dynamics_and_init()`)
- `n_ij ~ N(0, σ²I)` is Gaussian measurement noise
- `KF(·)` is the per-edge 3D Kalman filter (random-walk, `Q = (5mm)² I`, `R = σ² I`)
- `v_ff` is the common cruise feedforward (default `[0.5, 0, 0]` m/s)

Leaders receive only `v_ff`.

## Kalman Filter Design

| Parameter | Value |
|---|---|
| State | 3D relative position of neighbor in follower body frame |
| Process model | Random walk: `x_{k+1} = x_k + w`, `Q = (5mm)² I` |
| Measurement noise | `R = σ² I` (matched to injected noise) |

The KF is active per (follower, neighbor) pair independently. At `σ = 0.50 m` it reduces tail-mean formation error from **6.46 m (unstable)** to **40 mm (stable)**.

## Experiment Results Summary

| Experiment | Key finding |
|---|---|
| Baseline (stationary) | HIL vs ODE endpoint gap < 2 cm; UGV settling error fixed by spawn z=0 |
| Moving (0.5 m/s cruise) | Formation geometry preserved (‖Mx‖ ≈ 4 mm); UAV tracks at 104%, UGV at 108% |
| Noise sweep (no KF) | Robust up to σ = 0.20 m; threshold at σ = 0.30 m; unstable at σ = 0.50 m |
| Noise sweep (with KF) | σ = 0.50 m rescued from instability (160× improvement); threshold pushed beyond σ = 0.50 m |

## UGV Controller

`tests/ugv_chassis_control.py` — `UGVController` class:

- Input: target velocity vector in world frame `[vx, vy, 0]`
- Heading alignment via yaw error P-controller with cosine speed decay
- Differential-drive inverse kinematics → left/right wheel angular velocity
- PyBullet `VELOCITY_CONTROL` with torque saturation at 50 N·m

Note: control law is kinematic-level; dynamics (inertia, contact, friction) are handled by PyBullet's Featherstone solver.

## Citation / Upstream

Built on:
```
@misc{gymPybulletDrones,
  author = {Panerati, Jacopo and Zheng, Hehui and Zhou, SiQi and Xu, James and Prorok, Amanda and Schoellig, Angela P.},
  title  = {Learning to Fly -- a Gym Environment with PyBullet Physics for Reinforcement Learning of Multi-agent Quadcopter Control},
  year   = {2021},
  url    = {https://github.com/utiasDSL/gym-pybullet-drones}
}
```
