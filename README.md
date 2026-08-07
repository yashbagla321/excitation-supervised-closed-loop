# Excitation-Supervised Closed-Loop Self-Calibration and Target Seeking

Code and data for the paper *"Excitation-Supervised Closed-Loop
Self-Calibration and Target Seeking for an Unknown-Pose Range-Bearing
Relay"* (submitted to the IEEE Conference on Decision and Control, CDC
2027; arXiv preprint forthcoming).

This repository is a dependency-free C++17 simulation harness for the
paper's closed-loop controller: it decides online, from the same
trajectory-spread margin `S_v` used for identifiability, whether the
stored measurement window is excited enough to trust the current
calibration estimate, retriggering exploratory motion when it is not. The
estimator core here is shared with the companion ACC identifiability paper
([trajectory-induced-self-calibration](https://github.com/yashbagla321/trajectory-induced-self-calibration)),
which proves the static two-view identifiability result this controller
builds on. This repository's `main.cpp` is trimmed to the closed-loop
supervision study this paper cites; the companion repository runs the
open-loop identifiability/robustness sweeps instead.

## Controller

`u = -k(q - p_hat) + u_exp(t)`, with `u_exp(t) = A e^{-lambda(t - t0)}
[cos(omega t), sin(omega t)]^T` a decaying circular excitation whose epoch
`t0` resets whenever the stored window's trajectory spread `S_v` or local
observability `sigma_min(F)` falls below a threshold (Algorithm 1 in the
paper). The paper proves this supervision rule acquires any required
excitation within an explicit finite time and gives an accuracy-driven
rule for selecting the threshold from a desired calibration-accuracy
bound.

## Build

### Windows

```powershell
cmake -S . -B build
cmake --build build --config Release
```

### Linux

```bash
sudo apt update && sudo apt install build-essential cmake
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
```

## Run

```powershell
.\build\adaptive_localization_sim.exe
```

```bash
./build/adaptive_localization_sim
```

By default the executable loads `config/simulation.ini` (plain text,
heavily commented — the `supervised_*` keys set the paper's `S_bar`,
`sigma_bar`, and convergence thresholds) and writes into `results/`. Pass a
different config path as the first argument.

## Results-to-paper map

| File | Paper content |
|---|---|
| `results/closed_loop_local_1beacon.csv`, `results/beacon_estimates_local_1beacon.csv` | Figs. 2-3 (Algorithm 1's 60-packet flagship run: trajectory, target/beacon errors, accumulated spread `S_v`) |
| `results/supervised_lambda_sweep.csv` | Table II (decay-rate sweep, fixed circular vs. supervised, lambda in {0.02,...,2.0}) |
| `results/supervised_excitation_comparison.csv` | Section V-A nominal-scenario comparison (fixed vs. supervised beacon-position/yaw error when the fixed schedule happens to be adequate) |

All error/RMSE/reset-count columns are seed-deterministic and will
reproduce exactly.

## Figures

- `figures/closed_loop_run.dat` — the data table Figs. 2-3 are drawn from
  natively in pgfplots (matches the paper's fonts/math exactly, rather
  than a separately rendered raster image). Regenerate with
  `python scripts/export_closed_loop_data.py` after running the
  simulation binary (no extra dependencies).
- `figures/ros_gazebo_validation_compact.png` — the two-panel
  software-in-the-loop summary (Fig. 4: map view + convergence errors),
  drawn at print resolution directly from the same run's CSVs. Regenerate
  with `python scripts/render_gazebo_panel.py` (needs Pillow:
  `pip install Pillow`).
- `scripts/render_gazebo_validation_video.py` — renders the full
  ROS 2/Gazebo-style replay video and thumbnail frame-by-frame from the
  same CSVs (needs `imageio`, `numpy`, and Pillow:
  `pip install imageio numpy Pillow`); this is supplementary material, not
  itself one of the paper's print figures.

## Citation

```bibtex
@unpublished{bagla2027excitationsupervised,
  author = {Bagla, Yash},
  title  = {Excitation-Supervised Closed-Loop Self-Calibration and Target
            Seeking for an Unknown-Pose Range-Bearing Relay},
  note   = {Submitted to the IEEE Conference on Decision and Control (CDC)},
  year   = {2027}
}
```
