# Heterogeneous CoSim Platform

A PyBullet-based co-simulation platform for heterogeneous multi-agent formation control, validated with Hardware-in-the-Loop (HIL) experiments.

The system controls a mixed fleet of **3 UAVs (Crazyflie CF2X)** and **1 UGV (Clearpath Husky)** using a distributed complex-Laplacian consensus law. Each follower runs entirely on local, body-frame relative measurements — no central observer or global positioning required.

Built on top of [gym-pybullet-drones](https://github.com/utiasDSL/gym-pybullet-drones).

---

## Key Features

- **Distributed local-frame formation control** — each follower computes its velocity from body-yaw-frame relative measurements to neighbours
- **Common cruise velocity feedforward** — the entire formation translates at a fixed heading while maintaining geometry
- **Per-edge Kalman filter** — 3-D random-walk KF on each inter-agent measurement, raising the noise-stability threshold from σ = 0.30 m to beyond σ = 0.50 m
- **Heterogeneous actuators** — UAV DSL-PID cascade (position outer loop → attitude inner loop → motor mixing) + UGV differential-drive kinematics with torque-saturated wheel motors
- **Sim-vs-HIL validation** — ODE baseline vs full PyBullet physics compared across multiple experiments

---

## Repository Layout

```
.
├── run_hetero_three_uav_one_ugv_new.py   # Main entry point — 3 UAV + 1 UGV (recommended)
├── run_hetero_three_uav_one_ugv.py       # Earlier centralised-reference version
│
├── experiment_hetero_baseline.py         # Exp A  — Sim vs HIL, stationary formation
├── experiment_hetero_moving.py           # Exp B  — Sim vs HIL, with cruise velocity
├── experiment_hetero_noise.py            # Exp C  — Noise robustness sweep (no KF)
├── experiment_hetero_noise_kf.py         # Exp D  — Noise robustness sweep + Kalman filter
├── experiment_ugv_probe.py               # UGV speed characterisation
│
├── run_threesimilar_new.py               # 3 homogeneous UAVs (baseline comparison)
├── run_threesimilar_noisy.py             # Homogeneous UAVs with noise injection
├── run_sim_to_unity.py                   # Stream poses to Unity via UDP
│
├── tests/
│   └── ugv_chassis_control.py            # UGVController — differential-drive kinematics
│
└── gym_pybullet_drones/                  # Upstream gym (unmodified)
```

---

## Installation

**1. Clone and install**

```bash
git clone https://github.com/Hzzzzs/Heterogeneous-CoSim-Platform.git
cd Heterogeneous-CoSim-Platform
pip install -e .
```

**2. Extra dependencies**

```bash
pip install numpy scipy matplotlib pybullet
```

Tested on Python 3.10, Ubuntu 22.04 / WSL2.

---

## Usage

### Run the formation (with GUI)

```bash
python run_hetero_three_uav_one_ugv_new.py
```

Spawns 3 CF2X drones and 1 Husky in PyBullet. The formation converges and holds the target geometry.

**Environment variables:**

| Variable | Default | Effect |
|---|---|---|
| `HETERO_HEADLESS` | `0` | `1` → disable GUI, run at full speed |
| `HETERO_STEPS` | `10000` | Number of simulation steps |
| `HETERO_OUT` | _(none)_ | Path to save trajectory CSV |
| `HETERO_UDP` | `0` | `1` → stream poses to Unity over UDP |

Headless example saving a trajectory:

```bash
HETERO_HEADLESS=1 HETERO_STEPS=6000 HETERO_OUT=traj.csv \
    python run_hetero_three_uav_one_ugv_new.py
```

---

### Experiments

**Exp A — Sim vs HIL baseline (stationary)**

```bash
python experiment_hetero_baseline.py
```

Compares ODE integration (`dx/dt = Mx`) against full PyBullet physics. Outputs figures to `experiment_hetero_baseline/figures/`.

**Exp B — Moving formation (cruise velocity)**

```bash
# default: base_vel = [0.5, 0, 0] m/s
python experiment_hetero_moving.py

# custom speed and output directory
HETERO_BASE_VEL=0.3,0,0 HETERO_MOVING_OUT=out_slow \
    python experiment_hetero_moving.py
```

**Exp C — Noise robustness sweep (no KF)**

```bash
python experiment_hetero_noise.py
```

Sweeps σ ∈ {0, 0.02, 0.05, 0.10, 0.15, 0.20, 0.30, 0.50} m of Gaussian measurement noise. Results in `hetero_noise_moving/`.

**Exp D — Noise robustness with Kalman filter**

```bash
python experiment_hetero_noise_kf.py
```

Same sweep with a per-edge 3-D Kalman filter on every relative-position measurement. Results in `hetero_noise_moving_kf/`.

**UGV speed probe**

```bash
python experiment_ugv_probe.py
```

Characterises Husky's actual steady-state speed vs commanded speed.

---

### Stream to Unity

```bash
HETERO_UDP=1 python run_hetero_three_uav_one_ugv_new.py
```

Configure the target IP and port in `run_sim_to_unity.py`.

---

## Formation Configuration

**Node order** (12-dim virtual state `[UAV_L, UGV, UAV_F1, UAV_F2]`):

| Index | Agent | Initial position | Role |
|---|---|---|---|
| 0 | UAV_leader | (0, 0, 2.0) m | Leader |
| 1 | UGV | (1, 0, 0.0) m | Leader |
| 2 | UAV_F1 | (0.5, 1.0, 1.5) m | Follower |
| 3 | UAV_F2 | (1.5, 0.5, 1.5) m | Follower |

**Communication edges (5):** (L, F1), (L, F2), (UGV, F1), (UGV, F2), (F1, F2)

---

## Control Architecture

```
┌─────────────────────────────────────────┐
│         Distributed Formation Law        │
│  Leader:   v = v_ff                      │
│  Follower: v = Rz(ψ) Σ M_ij·KF(z_ij)  │
│            + v_ff                        │
└────────────┬──────────────┬─────────────┘
             │              │
    ┌────────▼──────┐ ┌─────▼──────────┐
    │ DSLPIDControl │ │ UGVController  │
    │  (240 Hz)     │ │  (240 Hz)      │
    │ pos outer PID │ │ heading align  │
    │ att inner PID │ │ diff-drive IK  │
    │ motor mixing  │ │ torque-sat motor│
    └────────┬──────┘ └──────┬─────────┘
             │               │
    ┌────────▼───────────────▼──────────┐
    │    PyBullet Physics (240 Hz)       │
    │  Featherstone + contact/friction   │
    └───────────────┬───────────────────┘
                    │ true poses
             ┌──────▼──────────────────┐
             │         Feedback         │
             │  pos/vel  → outer loop   │
             │  att/ω    → inner loop   │
             │  neighbour poses → law   │
             └─────────────────────────┘
```

**Kalman filter** (Exp D): per (follower, neighbour) pair, independent 3-D random-walk KF.
- Process noise: Q = (5 mm)² I
- Measurement noise: R = σ² I

---

## Experiment Results Summary

| Experiment | Key result |
|---|---|
| Stationary baseline | ‖Mx‖ = 3.4 mm; Sim vs HIL endpoint gap < 2 cm |
| Moving at 0.5 m/s | Formation geometry held (‖Mx‖ ≈ 4 mm) |
| Noise sweep (no KF) | Stable up to σ = 0.20 m; unstable at σ = 0.50 m |
| Noise sweep (with KF) | σ = 0.50 m: tail-mean error 6.46 m → 40 mm (160×) |

---

## Citation

If you use this work, please also cite the upstream environment:

```bibtex
@misc{gymPybulletDrones,
  author  = {Panerati, Jacopo and Zheng, Hehui and Zhou, SiQi and
             Xu, James and Prorok, Amanda and Schoellig, Angela P.},
  title   = {Learning to Fly---a Gym Environment with PyBullet Physics
             for Reinforcement Learning of Multi-agent Quadcopter Control},
  year    = {2021},
  url     = {https://github.com/utiasDSL/gym-pybullet-drones}
}
```
