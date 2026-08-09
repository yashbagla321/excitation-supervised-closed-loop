# cooperative_localization_gz

Modern Gazebo validation package for the hidden-target localization simulator.

This package is intentionally a validation layer, not a second estimator
implementation. The C++ nodes link against the top-level
`adaptive_localization_core` library so CLI simulations and ROS/Gazebo outputs
use the same estimator, world, measurement, and CSV writer code.

The centerpiece is `supervised_closed_loop_node`, which runs the paper's
excitation-supervised closed loop (Algorithm 1) software-in-the-loop through
Gazebo: velocity commands go to the Gazebo vehicle over `ros_gz_bridge`, the
physics engine integrates the motion, the estimator consumes the odometry
Gazebo publishes back, and emulated range--bearing packets reach the
estimator only after a configurable sensing delay. Estimation, the spread
certificate S_v, and the supervision rule are the same library code the
batch simulator runs; what differs is the plant -- real transport latency
and zero-order-hold actuation instead of idealized explicit-Euler
kinematics. The node retains the target prior until two noncoincident
vehicle-relative views support the constructive initializer; every later
packet warm-starts analytic-Jacobian Gauss--Newton. Excitation decay and
phase use simulation time in seconds (`s^-1` and `rad/s`), not packet index.

## Build

From `ros2_ws` after sourcing a ROS 2 environment with `ros_gz` installed:

```bash
colcon build --packages-select cooperative_localization_gz
```

## Run

Supervised closed-loop experiment (headless; add `use_gui:=true` to watch):

```bash
ros2 launch cooperative_localization_gz supervised_closed_loop_gz.launch.py
```

One launch is one experiment: gz-sim starts, the node paces 120 packets at
80 ms of simulation time on the bridged `/clock`, writes
`results/ros_gz/closed_loop_gz_run.csv` (same schema as
`closed_loop_local_1beacon.csv`, plus sim-time / command / beacon-estimate
columns). The shared `estimate_ready` column marks the first gauge-free
two-view estimate. The launch then shuts everything down. Batch runs pass a seed and
an output name:

```bash
ros2 launch cooperative_localization_gz supervised_closed_loop_gz.launch.py \
    seed:=8 sensing_delay_packets:=1 output_name:=closed_loop_gz_seed8.csv
```

Headless modern Gazebo (open-loop validation suite):

```bash
ros2 launch cooperative_localization_gz hidden_target_gz.launch.py use_gui:=false
```

GUI:

```bash
ros2 launch cooperative_localization_gz hidden_target_gz.launch.py use_gui:=true
```

The default launch writes:

- `results/ros_gz/ros_gz_single_trial.csv`
- `results/ros_gz/ros_gz_measurements.csv`
- `results/ros_gz_batch/*.csv`

## Nodes

- `supervised_closed_loop_node`: excitation-supervised closed-loop control
  through Gazebo (velocity commands out, odometry in, delayed emulated
  range--bearing packets, online Gauss--Newton estimation, spread-threshold
  supervision, per-packet CSV log). Moves the translucent estimate markers
  in the scene through the bridged `set_pose` service so recordings show
  the estimator converging.
- `hidden_target_experiment_node`: runs one configured estimator trial and
  publishes target/beacon markers.
- `measurement_emulator_node`: emits local-frame range-bearing packets with
  configurable noise and dropout, and publishes measurement-line markers.
- `hidden_target_batch_runner`: runs the expanded robustness/failure-mode CSV
  suite through the shared C++ core.
