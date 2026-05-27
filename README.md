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

This repository is a downstream of **gym-pybullet-drones**, so the Python
backend installs exactly the same way as the upstream project. Use an isolated
environment (conda recommended, as in the
[upstream README](https://github.com/utiasDSL/gym-pybullet-drones#installation)).

### Option A — full install (same as gym-pybullet-drones)

```bash
git clone https://github.com/Hzzzzs/Heterogeneous-CoSim-Platform.git
cd Heterogeneous-CoSim-Platform

conda create -n drones python=3.10
conda activate drones
pip install --upgrade pip
pip install -e .          # if pybullet fails to build: sudo apt install build-essential
```

`pip install -e .` reads this repo's `pyproject.toml` (which **is** the
gym-pybullet-drones spec) and pulls every dependency, including the RL extras
(`stable-baselines3`, `control`). This is the safest path and matches upstream.

### Option B — lightweight install (only what the formation code needs)

Our experiments and demos never import `torch` / `stable-baselines3` / `control`
(verified: the `CtrlAviary` → `DSLPIDControl` chain pulls none of them). If you
only want to reproduce the four experiments, you can skip the heavy RL stack:

```bash
conda create -n hetero python=3.10
conda activate hetero
pip install numpy scipy matplotlib pybullet gymnasium
pip install -e . --no-deps      # register gym_pybullet_drones without RL deps
```

Tested dependency versions (Python 3.10): numpy 1.26 / scipy 1.15 /
matplotlib 3.5 / pybullet 3.2 / gymnasium 1.2. Verified on Ubuntu 22.04 (WSL2).

> **Always run from the repository root.** The commands below use
> `python -m hetero_cosim...`, which relies on the repo root being the working
> directory; output folders and `obstacles/` are resolved relative to it.

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

## Unity Visualization Platform

The platform is a **dual-engine, decoupled** design: PyBullet (Python) is the
physics backend, Unity is the rendering frontend, running as two independent
processes coupled by two buses. Decoupling means the rendering load never
perturbs the 240 Hz physics step — it avoids the "15.5× → 1.3× single-machine
slowdown when visualization is turned on" that a single-engine setup suffers.

```
┌────────────────────┐   UDP async broadcast (ASCII, 240 Hz)   ┌──────────────────┐
│  PyBullet backend  │ ───────────────────────────────────────▶ │  Unity frontend  │
│  (physics, Python) │                                          │  (rendering)     │
│                    │ ◀─────────────────────────────────────── │                  │
└────────────────────┘   .obj terrain reverse-injection         └──────────────────┘
```

- **Forward bus (PyBullet → Unity):** every physics step, the backend packs each
  agent's `class, id, position, attitude` into a lightweight ASCII datagram and
  broadcasts it over a **non-blocking UDP** socket. UDP (not TCP) is chosen
  because the frontend is visualization-only — it never feeds back into control,
  so a dropped packet is at worst one interpolated frame ("low-latency priority"
  over "zero loss"). Message format:
  ```
  UAV,{id},{x},{y},{z},{roll},{pitch},{yaw}
  UGV,{id},{x},{y},{z},{roll},{pitch},{yaw}
  ```
- **Reverse bus (Unity → PyBullet):** scenes built in Unity (Terrain + prefab
  trees/buildings) are exported by a C# plugin as world-aligned `.obj` collision
  meshes, which PyBullet loads as mass-0 static rigid bodies. This closes the
  "visual obstacle exists in Unity but the backend only has flat ground" gap.

### The Unity project (`unity/`)

A Unity **2022.3.62f1** project lives in [`unity/`](unity/) (only
`Assets/ Packages/ ProjectSettings/` are committed; the `Library/` cache is
regenerated on first open).

1. Open **Unity Hub → Add project from disk →** select the `unity/` folder.
2. Open with Unity **2022.3.62f1** (let it rebuild the `Library/` cache).
3. Open the main scene under `Assets/Scenes/`, press **Play** — it now listens
   on UDP `5006` for pose datagrams.

### Coordinate realignment (important)

PyBullet uses a **Z-Up right-handed** world; Unity uses a **Y-Up left-handed**
world. The frontend re-aligns incoming poses with a single odd permutation
(a chirality flip, `det(P) = -1`):

- **Position:** `[X, Y, Z]_PB → [X, Z, Y]_Unity`
- **Attitude:** rebuild rotation from the euler angles, then `R_Unity = P · R_PB · Pᵀ`
- **Visual yaw compensation:** third-party ground-vehicle models often define a
  different forward axis (e.g. the Husky model faces `-X_model`, not `+Z`). A
  configurable per-model yaw offset (e.g. ±90°) aligns the model's heading with
  the physics velocity vector, removing the "car slides sideways" rendering glitch.

### Running a live demo

The `demos/` scripts mirror the four experiments but render in the PyBullet GUI
**and** stream 240 Hz poses to Unity. Obstacles in `obstacles/` are loaded with
collision detection; a mid-air or obstacle collision stops the run and prints
the offending bodies.

```bash
# 1. In Unity, press Play (scene listens on UDP 5006).
# 2. From the repo root, run any demo:
python -m hetero_cosim.demos.baseline                    # stationary formation
python -m hetero_cosim.demos.moving                      # cruise at 0.5 m/s
HETERO_SIGMA=0.30 python -m hetero_cosim.demos.noise     # noisy, no filter
HETERO_SIGMA=1.0  python -m hetero_cosim.demos.noise_kf  # noise + KF, stays stable
```

If Unity runs on a different machine (or Windows host ↔ WSL), point the sender
at it: `UNITY_HOST=192.168.1.50 python -m hetero_cosim.demos.moving`
(the script auto-resolves the WSL host IP otherwise).

Standalone GUI scenes:

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
