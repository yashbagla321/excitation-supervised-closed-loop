#!/usr/bin/env bash
# Reproduce the paper's Gazebo software-in-the-loop results: ten seeds at the
# default one-packet sensing delay, delay-0 and delay-2 variants, and the
# flagship run (seed 7, delay 1) whose log draws Fig. 4.
#
# Run from the repository root on a machine with ROS 2 Jazzy and ros_gz
# installed (Linux; under WSL2 build from the Linux filesystem, not /mnt/c):
#
#   ./scripts/run_gazebo_batch.sh
#
# Each launch is one experiment: gz-sim starts headless, the supervised
# closed-loop node paces 120 packets at 80 ms of simulation time, writes its
# CSV under results/ros_gz/, and the launch shuts down. Runs take ~15 s each.
#
# (No `set -u`: the ROS setup scripts reference unset variables.)
set -e
source /opt/ros/jazzy/setup.bash

cd "$(dirname "$0")/.."
REPO_ROOT="$(pwd)"

cd ros2_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
cd "${REPO_ROOT}"
set +e

# One launch = one experiment, and gz-transport is brokerless: a zombie gz
# server from an interrupted earlier run publishes odometry into the next
# run's topics and corrupts it. Kill strays before every launch.
cleanup() {
    pkill -f "gz sim" 2>/dev/null
    pkill -f parameter_bridge 2>/dev/null
    pkill -f supervised_closed_loop_node 2>/dev/null
    sleep 2
}

cleanup
for seed in 7 8 9 10 11 12 13 14 15 16; do
    timeout 120 ros2 launch cooperative_localization_gz supervised_closed_loop_gz.launch.py \
        "seed:=${seed}" "output_name:=closed_loop_gz_seed${seed}.csv"
    cleanup
done

for d in 0 2; do
    timeout 120 ros2 launch cooperative_localization_gz supervised_closed_loop_gz.launch.py \
        "seed:=7" "sensing_delay_packets:=${d}" "output_name:=closed_loop_gz_delay${d}.csv"
    cleanup
done

# Flagship run (seed 7, delay 1): the primary artifact CSV behind Fig. 4.
timeout 120 ros2 launch cooperative_localization_gz supervised_closed_loop_gz.launch.py
cleanup

echo "Done. Logs in results/ros_gz/."
