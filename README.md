# Excitation-Supervised Closed-Loop Self-Calibration and Target Seeking

Code and data for the paper *"Excitation-Supervised Closed-Loop
Self-Calibration and Target Seeking for an Unknown-Pose Range-Bearing
Relay"*
([arXiv:2608.12528](https://arxiv.org/abs/2608.12528)). Each release is archived on Zenodo:
[doi:10.5281/zenodo.21892671](https://doi.org/10.5281/zenodo.21892671).

This repository is a dependency-free C++17 simulation harness for the
paper's closed-loop controller: it decides online, from the same
trajectory-spread margin `S_v` used for identifiability, whether the
stored measurement window is excited enough to trust the current
calibration estimate, retriggering exploratory motion when it is not. The
estimator core here is shared with the companion identifiability paper
([arXiv:2608.09464](https://arxiv.org/abs/2608.09464), code in
[trajectory-induced-self-calibration](https://github.com/yashbagla321/trajectory-induced-self-calibration)),
which proves the static two-view identifiability result this controller
builds on. This repository's `main.cpp` is trimmed to the closed-loop
supervision study this paper cites; the companion repository runs the
open-loop identifiability/robustness sweeps instead.

## Controller

`u = u_seek + u_exp(t)`, with `u_exp(t) = A e^{-lambda(t - t0)}
[cos(omega t), sin(omega t)]^T` a decaying circular excitation whose epoch
`t0` resets whenever the stored window's trajectory spread `S_v` (computed
from the known measurement poses) falls below the threshold `S_bar`
(Algorithm 1 in the paper). While the window is underexcited, the nominal
seeking velocity `-k(q - p_hat)` is projected onto the half-space
`{v : v^T n(t) >= -A e^{-lambda T_bar} / pi}`, where
`n(t) = (-1)^floor(omega t / pi) e_y` is the current excitation
half-period's push direction: only the seeking component opposing `n(t)`
is clipped, so the paper's finite-acquisition guarantee covers the
controller as implemented, and the unprojected law is restored exactly
once `S_v >= S_bar`. Conditioning (`sigma_min` of the whitened
stacked Jacobian) is logged per packet as a diagnostic but is deliberately
not a trigger: the finite-acquisition guarantee covers exactly the spread
certificate. The default `S_bar = 0.16` comes from the paper's
accuracy-driven selection rule `S_bar = sigma^2 / eps_psi^2` at the
closed-loop range noise `sigma = 0.02 m` and the declared 0.05-rad
yaw-RMSE success criterion; the paper's threshold ablation (Table III)
sweeps this constant and quantifies the accuracy-versus-effort tradeoff.

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

### Sanitizer build (optional, for development)

```bash
cmake --preset sanitize-debug
cmake --build --preset sanitize-debug --config Debug
```

Builds Debug with AddressSanitizer (and UndefinedBehaviorSanitizer on
Clang/GCC; MSVC only supports ASan) via `CMakePresets.json`, to catch
out-of-bounds/undefined-behavior bugs in the hand-rolled pointer/array
indexing used throughout the EKF and closed-loop controller. Requires
CMake 3.21+. On MSVC, the ASan runtime DLL
(`clang_rt.asan_dynamic-x86_64.dll`, under your Visual Studio
installation's `VC/Tools/MSVC/<version>/bin/Hostx64/x64`) must be on
`PATH` to run the resulting binary.

## Run

```powershell
.\build\adaptive_localization_sim.exe
```

```bash
./build/adaptive_localization_sim
```

By default the executable loads `config/simulation.ini` (plain text,
heavily commented — the `supervised_*` keys set the paper's `S_bar`,
the convergence thresholds, and the Monte Carlo batch size) and writes
into `results/`. Pass a different config path as the first argument.

## Results-to-paper map

| File | Paper content |
|---|---|
| `results/closed_loop_local_1beacon.csv`, `results/beacon_estimates_local_1beacon.csv` | Figs. 3-4 (Algorithm 1's flagship run: trajectory, target/beacon errors, the supervisor's logged spread `S_v`) |
| `results/ros_gz/closed_loop_gz_run.csv` (+ `closed_loop_gz_seed*.csv`, `closed_loop_gz_delay*.csv`) | Fig. 2 and Section V-B (the Gazebo software-in-the-loop run, ten-seed repeatability batch, and delay-0/2 variants) |
| `results/supervised_lambda_sweep.csv` | Table II (paired 100-trial Monte Carlo decay-rate sweep: across-trial RMSE with bootstrap 95% CIs and success rates, fixed circular vs. supervised) |
| `results/supervised_threshold_ablation.csv` | Table III (spread-threshold ablation over `S_bar` in {0.05, 0.16, 1.0, 9.04}: design-rule prediction vs. measured yaw RMSE, packets to certify, path length, excitation effort) |
| `results/supervised_seeking_comparison.csv` | Section V-E (target-seeking scenario: both policies succeed in all 100 paired trials — incidental excitation from ordinary motion suffices) |
| `results/supervised_excitation_comparison.csv` | Section V-C nominal-scenario comparison (fixed vs. supervised beacon-position/yaw error when the fixed schedule happens to be adequate) |
| `results/closed_loop_showcase_understimulated_*.csv`, `results/closed_loop_showcase_seeking_ring_*.csv` | Policy-showcase figure (fixed / information-gradient / supervised on identical no-transient packets, plus the six-start seeking-ring variety panel) |

`scripts/check_acquisition_assumptions.py` (stdlib only) re-derives the
paper's assumptions-versus-implementation numbers for the finite-acquisition
proposition directly from these committed per-packet logs, verifies the
seeking-projection invariant on every underexcited packet, and asserts every
expected finding. `scripts/aggregate_gazebo_batch.py` (stdlib only)
recomputes the Gazebo table from `results/ros_gz/`: per-run final errors,
the across-seed RMS row, retrigger counts, and the packet at which the
spread certificate clears `S_bar`.

All error/RMSE/reset-count columns are seed-deterministic given a fixed
`std::mt19937` stream, but exact bit-for-bit reproduction of the committed
CSVs additionally requires the same C++ standard library implementation
used to generate them (this repo's checked-in results were built with
MinGW GCC's libstdc++ on Windows): `std::normal_distribution`'s sample
sequence for a given engine state is implementation-defined, not specified
by the C++ standard, so `sample_noise()`
(`include/adaptive_localization/Math.hpp`) can draw a
different-but-statistically-equivalent sequence on, e.g., MSVC's STL or
libc++. Rebuilding on a different standard library will
reproduce the same trends and conclusions but not necessarily identical
values to the last decimal place.

## Figures

- `figures/closed_loop_run.dat` — the data table Figs. 3-4 are drawn from
  natively in pgfplots (matches the paper's fonts/math exactly, rather
  than a separately rendered raster image). Regenerate with
  `python scripts/export_closed_loop_data.py` after running the
  simulation binary (no extra dependencies).
- `figures/ros_gazebo_validation_compact.png` — the two-panel
  software-in-the-loop summary (Fig. 2: map view + convergence errors),
  drawn at print resolution directly from the Gazebo run's logged CSV
  (`results/ros_gz/closed_loop_gz_run.csv`). Regenerate with
  `python scripts/render_gazebo_panel.py`.
- `scripts/render_gazebo_validation_video.py` — renders the replay video
  and thumbnail frame-by-frame from the same Gazebo run log; this is
  supplementary material, not itself one of the paper's print figures.
  Needs an `imageio` video backend to write the `.mp4` itself (see
  `requirements.txt`).

Python scripts depend on Pillow, numpy, and imageio; install with
`pip install -r requirements.txt`.

## ROS 2 / Gazebo software-in-the-loop experiment

`ros2_ws/src/cooperative_localization_gz` runs Algorithm 1 through the
ROS 2 / Gazebo stack rather than replaying simulator output:
`supervised_closed_loop_node` publishes planar velocity commands over
`ros_gz_bridge` to a vehicle model integrated by the Gazebo physics engine,
consumes the odometry Gazebo publishes back, emulates range--bearing
packets at those poses, and delivers them to the estimator after a
configurable sensing delay (`sensing_delay_packets`, default 1 cycle =
80 ms). Estimation, the spread certificate `S_v`, and the supervision rule
are the same `adaptive_localization_core` code the batch simulator links —
the package's CMake builds the core from this repository's root — so the
experiment changes the plant, not the algorithm.

On a machine with ROS 2 Jazzy and `ros_gz` (Linux; under WSL2 build from
the Linux filesystem, not `/mnt/c`):

```bash
cd ros2_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
cd ..
ros2 launch cooperative_localization_gz supervised_closed_loop_gz.launch.py
```

One launch is one experiment: the node paces 120 packets at 80 ms of
simulation time on the bridged `/clock`, logs
`results/ros_gz/closed_loop_gz_run.csv` (same schema as
`closed_loop_local_1beacon.csv` plus sim-time, command, and beacon-estimate
columns), and shuts the simulator down when the packet budget ends. Add
`use_gui:=true` to watch; the node moves the translucent estimate markers
in the scene each packet through the bridged `set_pose` service.
`./scripts/run_gazebo_batch.sh` reproduces the paper's full batch (ten
seeds plus delay-0/2 variants plus the flagship run).

## Citation

If you use this code, or build on the paper's results or concepts, please
cite the paper:

```bibtex
@misc{bagla2027excitationsupervised,
  author        = {Bagla, Yash},
  title         = {Excitation-Supervised Closed-Loop Self-Calibration and
                   Target Seeking for an Unknown-Pose Range-Bearing Relay},
  year          = {2026},
  eprint        = {2608.12528},
  archivePrefix = {arXiv},
  primaryClass  = {eess.SY},
  doi           = {10.48550/arXiv.2608.12528},
  url           = {https://arxiv.org/abs/2608.12528}
}
```

To cite this software artifact specifically, use the version DOI from the
Zenodo archive (see `CITATION.cff`).

## License

MIT (see `LICENSE`).
