"""Export the 60-step flagship closed-loop run as a pgfplots-readable table.

The paper draws Figs. 2-3 (closed-loop trajectory and error traces) natively
in pgfplots from the .dat file this writes, so fonts and math notation match
the paper text rather than a separately rendered raster image.

Module responsibility: this script does no simulation of its own. It is a
thin, one-way transform from the C++ binary's flagship-run CSV
(`results/closed_loop_local_1beacon.csv`, written by src/main.cpp using the
excitation-supervised controller) into a whitespace-delimited `.dat` file
that LaTeX/pgfplots can \\addplot directly."""

from __future__ import annotations

import csv
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
RESULTS = ROOT / "results"


def load_numeric_rows(path: Path) -> list[dict[str, float]]:
    """Read a CSV file written by the C++ binary into a list of row dicts.

    Every column is parsed as a float (the closed-loop CSVs are all-numeric:
    step index, positions, errors, RMSEs), which lets the caller index into
    a row by column name (e.g. row["robot_x"]) instead of by position.

    Parameters:
        path: CSV file to read, with a header row naming each column.
    Returns: one dict per data row, mapping column name to its float value.
    """
    with path.open(newline="") as handle:
        return [
            {key: float(value) for key, value in row.items()}
            for row in csv.DictReader(handle)
        ]


def export_closed_loop_pgfplots_data() -> None:
    """Convert the flagship closed-loop run into `figures/closed_loop_run.dat`.

    Takes no parameters and returns nothing; it reads
    `results/closed_loop_local_1beacon.csv` (already restricted to the first
    60 measurement steps, matching the window plotted in the paper) and
    writes a matching `.dat` file with one extra derived column:

    - The first 7 columns (step, robot_x/y, goal_error, target_error,
      beacon_position_rmse, beacon_yaw_rmse) are copied straight through from
      the CSV, just renamed to the short pgfplots-friendly headers
      `x y goal target bpos byaw`.
    - `spread` is computed here rather than read from the CSV: for each
      prefix of the trajectory (steps 0..end), it is the sum of squared
      distances of every visited robot position from the running mean
      position so far — a growing-window measure of how spatially spread out
      the vehicle's path has been. This is a plotting-side visualization of
      the same idea as the trajectory-spread margin (S_v) that the
      excitation-supervised controller in Simulation.cpp checks against
      `supervised_spread_threshold` to decide whether to retrigger its
      excitation epoch; it is recomputed independently here rather than
      re-exported from the C++ run, so it is not guaranteed to use the exact
      same formula as the controller's internal `trajectory_spread()`.
    """
    rows = load_numeric_rows(RESULTS / "closed_loop_local_1beacon.csv")[:60]
    spread: list[float] = []
    for end in range(1, len(rows) + 1):
        xs = [row["robot_x"] for row in rows[:end]]
        ys = [row["robot_y"] for row in rows[:end]]
        mean_x = sum(xs) / end
        mean_y = sum(ys) / end
        spread.append(sum((x - mean_x) ** 2 + (y - mean_y) ** 2 for x, y in zip(xs, ys)))

    out = ROOT / "figures" / "closed_loop_run.dat"
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("w", newline="\n") as handle:
        handle.write("step x y goal target bpos byaw spread\n")
        for index, row in enumerate(rows):
            handle.write(
                f"{index} {row['robot_x']:.6f} {row['robot_y']:.6f} "
                f"{row['goal_error']:.8f} {row['target_error']:.8f} "
                f"{row['beacon_position_rmse']:.8f} {row['beacon_yaw_rmse']:.8f} "
                f"{spread[index]:.6f}\n"
            )
    print(out)


if __name__ == "__main__":
    export_closed_loop_pgfplots_data()
