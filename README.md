# Heterogeneous CoSim Platform

A PyBullet-based co-simulation platform for heterogeneous multi-agent formation
control, validated with Hardware-in-the-Loop (HIL) experiments and visualized
live in Unity.

The system controls a mixed fleet of **3 UAVs (Crazyflie CF2X)** and
**1 UGV (Clearpath Husky / Ackermann racecar)** using a distributed
complex-Laplacian (z-similar) consensus law. Each follower runs entirely on
local, body-frame relative measurements — no central observer or global
positioning required.

Built on top of [gym-pybullet-drones](https://github.com/utiasDSL/gym-pybullet-drones).

---

## Key Features

- **Distributed local-frame formation control** — each follower computes its
  velocity from body-yaw-frame relative measurements to its neighbours.
- **Common cruise feedforward** — the whole formation translates at a fixed
  heading while keeping its geometry (z-similar invariance to translation).
- **Per-edge Kalman filter** — a 3-D random-walk KF on each inter-agent
  measurement raises the noise-stability boundary from σ ≈ 0.30 m (unfiltered)
  to σ ≥ 1.0 m.
- **Heterogeneous actuators** — UAV DSL-PID cascade + UGV differential-drive
  (Husky) or Ackermann steering (racecar), all under full PyBullet physics.
- **Sim-vs-HIL validation** — ideal ODE baseline vs full PyBullet, plus live
  Unity visualization over UDP.

---

## Repository Layout

```
.
├── hetero_cosim/                 # all platform code (this project)
│   ├── ugv_chassis_control.py    # Husky differential-drive controller
│   ├── unity_bridge.py           # 240 Hz UDP pose sender to Unity
│   ├── runtime.py                # shared demo loop (env + UGV + UDP + collisions)
│   ├── formations/
│   │   ├── homogeneous_4uav.py   # baseline: 4 homogeneous UAVs
│   │   ├── hetero_ugv.py         # CORE: 3 UAV + 1 UGV, get_dynamics_and_init()
│   │   ├── hetero_ugv_gui.py     # GUI demo: Husky + tree obstacle + Unity
│   │   └── hetero_racecar.py     # Ackermann racecar variant
│   ├── experiments/              # headless, produce CSV + figures
│   │   ├── baseline.py           # Exp 1: stationary, Sim vs HIL
│   │   ├── moving.py             # Exp 2: common cruise velocity
│   │   ├── noise.py              # Exp 3: measurement-noise sweep (no KF)
│   │   ├── noise_kf.py           # Exp 4: noise sweep with per-edge KF
│   │   └── ugv_probe.py          # utility: calibrate UGV wheel radius
│   └── demos/                    # real-time, stream poses to Unity
│       ├── baseline.py  moving.py  noise.py  noise_kf.py
├── obstacles/                    # .obj scene meshes for demo collision tests
├── gym_pybullet_drones/          # upstream library (unmodified)
└── README.md
```

---

## Installation

```bash
git clone https://github.com/Hzzzzs/Heterogeneous-CoSim-Platform.git
cd Heterogeneous-CoSim-Platform
pip install -e .
pip install numpy scipy matplotlib pybullet
```

Tested on Python 3.10, Ubuntu 22.04 / WSL2.

> **Always run from the repository root** (the commands below use
> `python -m hetero_cosim...`, which relies on the repo root being the working
> directory). Output folders and `obstacles/` are resolved relative to it.

---

## Reproducing the Four Experiments

Each experiment is headless and writes a CSV trajectory + metrics + figures
into its own output folder. Results are cached — delete the output folder (or
its `hil*.csv` / `raw/`) to force a fresh run. All four cruise at
`base_vel = [0.5, 0, 0]` m/s except the stationary baseline, and run 6000 steps
≈ 25 s at 240 Hz.

### Experiment 1 — Stationary baseline (Sim vs HIL)

```bash
python -m hetero_cosim.experiments.baseline
```
Compares the ideal ODE (`dx/dt = M x`) against full PyBullet physics with
stationary leaders. **Output:** `experiment_hetero_baseline/`.
**Expected:** `‖M x_HIL(T)‖ ≈ 2.0e-3` (mm-level formation error); Sim-vs-HIL
endpoint gap ≈ 2 cm — confirms the platform reproduces the ideal dynamics.

### Experiment 2 — Common cruise velocity

```bash
python -m hetero_cosim.experiments.moving
# custom speed / output dir:
HETERO_BASE_VEL=0.3,0,0 HETERO_MOVING_OUT=my_run python -m hetero_cosim.experiments.moving
```
The whole formation translates along +X. **Output:** `experiment_hetero_moving/`.
**Expected:** `‖M x_HIL(T)‖ ≈ 2.2e-3` (still mm-level — geometry preserved under
translation); UAV mean vx ≈ 0.495 m/s, UGV ≈ 0.501 m/s (both track the command).

### Experiment 3 — Measurement-noise robustness (no KF)

```bash
python -m hetero_cosim.experiments.noise
```
Sweeps Gaussian noise σ ∈ {0, 0.02, 0.05, 0.10, 0.15, 0.20, 0.30, 0.50} m on
each per-edge relative measurement. **Output:** `hetero_noise_moving/`.
**Expected:** stable up to σ ≈ 0.20 m, engineering knee at σ = 0.30 m
(tail_mean ≈ 38 mm), **unstable at σ = 0.50 m** (tail_mean ≈ 6.1 m — formation
flies apart).

### Experiment 4 — Noise robustness with per-edge Kalman filter

```bash
python -m hetero_cosim.experiments.noise_kf
```
Same sweep with a 3-D random-walk KF on every neighbour measurement.
**Output:** `hetero_noise_moving_kf/`.
**Expected:** σ = 0.50 m is **rescued from instability** (tail_mean 6.1 m → 39 mm,
~155×); even σ = 1.0 m stays stable (tail_mean ≈ 77 mm). The KF pushes the usable
noise boundary from ≈ 0.30 m to ≥ 1.0 m.

| σ (m) | tail_mean no-KF | tail_mean +KF | improvement |
|------:|----------------:|--------------:|------------:|
| 0.30  | 3.8e-2          | 2.3e-2        | 1.7×        |
| 0.50  | 6.09 (unstable) | 3.9e-2        | ~155×       |
| 1.00  | 6.51 (unstable) | 7.7e-2        | ~84×        |

---

## UGV Wheel-Radius Calibration (utility)

```bash
python -m hetero_cosim.experiments.ugv_probe
```
Drives a lone Husky straight at several speeds and reports actual/commanded vx.
Used to verify `wheel_radius = 0.17776 m` (read from the Husky URDF wheel
cylinder) in `hetero_cosim/ugv_chassis_control.py`. A flat ratio ≈ 1.00 across
speeds means the UGV tracks its velocity command with no systematic error.

---

## Live Unity Visualization

The `demos/` scripts mirror the four experiments but render in the PyBullet GUI
and stream 240 Hz pose data to Unity over UDP (default `127.0.0.1:5006`).
Obstacles in `obstacles/` are loaded with collision detection; a mid-air or
obstacle collision stops the run with a message.

```bash
# 1. start the Unity receiver listening on UDP 5006 (or set UNITY_PORT)
# 2. run any demo from the repo root:
python -m hetero_cosim.demos.baseline                 # stationary
python -m hetero_cosim.demos.moving                   # cruise at 0.5 m/s
HETERO_SIGMA=0.30 python -m hetero_cosim.demos.noise     # noise, no filter
HETERO_SIGMA=1.0  python -m hetero_cosim.demos.noise_kf  # noise + KF, stays stable
```

Standalone GUI scenes (Husky + tree obstacle, and the Ackermann racecar):

```bash
python -m hetero_cosim.formations.hetero_ugv_gui      # Husky + tree + Unity
python -m hetero_cosim.formations.hetero_racecar      # Ackermann racecar
python -m hetero_cosim.formations.homogeneous_4uav    # 4 homogeneous UAVs (base)
```

### Common environment variables

| Variable | Default | Effect |
|---|---|---|
| `HETERO_STEPS` | 6000 | number of 240 Hz steps (~25 s) |
| `HETERO_HEADLESS` | 0 | `1` → no GUI, no real-time pacing |
| `HETERO_BASE_VEL` | `0.5,0,0` | cruise velocity (moving/noise demos & exp) |
| `HETERO_SIGMA` | demo-specific | measurement-noise std, meters |
| `UNITY_UDP` | 1 | `0` → disable UDP send |
| `UNITY_HOST` / `UNITY_PORT` | auto / 5006 | Unity target |
| `HETERO_OBSTACLE_DIR` | `obstacles` | obstacle folder (`""` → none) |

---

## Control Architecture

```
┌──────────────────────────────────────────────┐
│  Distributed formation law  (240 Hz, per node)│
│  Leader  : v = v_ff                            │
│  Follower: v = R_z(ψ) Σ M_ij·z_ij + v_ff       │
│            z_ij = R_z(-ψ)(x_j - x_i)  (+noise / +KF)
└───────────┬───────────────────┬────────────────┘
   ┌─────────▼────────┐  ┌───────▼─────────┐
   │ DSLPIDControl     │  │ UGV chassis     │
   │ pos→att→motor     │  │ diff-drive / Ackermann
   └─────────┬────────┘  └───────┬─────────┘
       ┌──────▼───────────────────▼──────┐
       │  PyBullet physics (240 Hz)       │
       └──────────────┬───────────────────┘
            true poses │→ UDP 240 Hz → Unity render
                       └→ feedback to controllers + formation law
```

Key parameters: `wheel_radius = 0.17776 m` (from Husky URDF), cruise
`base_vel = [0.5, 0, 0]`, KF process noise `Q = (5 mm)²`, measurement noise
`R = σ² I`. Node order of the 12-D virtual state: `[UAV_leader, UGV, UAV_F1, UAV_F2]`;
communication edges: (L,F1), (L,F2), (UGV,F1), (UGV,F2), (F1,F2).

---

## Citation

This platform builds on gym-pybullet-drones:

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
