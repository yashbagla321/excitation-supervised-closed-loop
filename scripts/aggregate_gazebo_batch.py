"""Aggregate the ROS 2/Gazebo batch CSVs into the paper's reported numbers.

Reads results/ros_gz/closed_loop_gz_{run,delay0,delay2,seed7..16}.csv and
prints, per run: final goal / target / beacon-position / beacon-yaw errors,
the retrigger count, and the packet at which the spread certificate first
clears S_bar = 0.16; then the across-seed RMSE row over the ten delay-1
seeds. These are the values transcribed into the Gazebo table and prose of
the closed-loop papers (previously assembled by hand). Gazebo runs are not
bit-reproducible -- message timing shifts the mm-level digits run to run --
so regenerate this table whenever the batch is rerun.

Usage: python scripts/aggregate_gazebo_batch.py [results/ros_gz]
"""
import csv
import math
import sys
from pathlib import Path

SBAR = 0.16
SEEDS = list(range(7, 17))


def load(path):
    with path.open(newline="") as handle:
        return [{k: (float(v) if v not in ("", None) else None)
                 for k, v in row.items()} for row in csv.DictReader(handle)]


def summarize(path):
    rows = load(path)
    last = rows[-1]
    retrig = sum(1 for r in rows if r["retriggered"] == 1.0)
    crossing = next((int(r["step"]) for r in rows if r["spread"] >= SBAR), None)
    return {
        "goal": last["goal_error"], "target": last["target_error"],
        "bpos": last["beacon_position_rmse"], "byaw": last["beacon_yaw_rmse"],
        "retrig": retrig, "crossing": crossing, "packets": int(last["step"]),
    }


def rms(values):
    return math.sqrt(sum(v * v for v in values) / len(values))


def main():
    base = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("results/ros_gz")
    header = f"{'run':>16s} {'goal':>8s} {'target':>8s} {'bpos':>8s} " \
             f"{'byaw':>8s} {'retrig':>6s} {'cross':>5s}"
    print(header)
    seed_rows = []
    for seed in SEEDS:
        s = summarize(base / f"closed_loop_gz_seed{seed}.csv")
        seed_rows.append(s)
        print(f"{f'seed {seed}':>16s} {s['goal']:8.4f} {s['target']:8.4f} "
              f"{s['bpos']:8.4f} {s['byaw']:8.4f} {s['retrig']:6d} "
              f"{s['crossing'] if s['crossing'] is not None else '-':>5}")
    print(f"{'across-seed RMS':>16s} "
          f"{rms([s['goal'] for s in seed_rows]):8.4f} "
          f"{rms([s['target'] for s in seed_rows]):8.4f} "
          f"{rms([s['bpos'] for s in seed_rows]):8.4f} "
          f"{rms([s['byaw'] for s in seed_rows]):8.4f}")
    retrigs = sorted({s["retrig"] for s in seed_rows})
    crossings = sorted({s["crossing"] for s in seed_rows})
    print(f"{'':>16s} retrigger counts across seeds: {retrigs}, "
          f"crossings: {crossings}")
    for name in ["run", "delay0", "delay2"]:
        s = summarize(base / f"closed_loop_gz_{name}.csv")
        print(f"{name:>16s} {s['goal']:8.4f} {s['target']:8.4f} "
              f"{s['bpos']:8.4f} {s['byaw']:8.4f} {s['retrig']:6d} "
              f"{s['crossing'] if s['crossing'] is not None else '-':>5}")


if __name__ == "__main__":
    main()
