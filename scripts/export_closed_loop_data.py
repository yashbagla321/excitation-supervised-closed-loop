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

    Every non-empty cell is parsed as a float (the closed-loop CSVs are
    all-numeric except for optional metrics, which are written as empty
    cells when not applicable and skipped here), which lets the caller
    index into a row by column name (e.g. row["robot_x"]) instead of by
    position.

    Parameters:
        path: CSV file to read, with a header row naming each column.
    Returns: one dict per data row, mapping column name to its float value.
    """
    with path.open(newline="") as handle:
        return [
            {key: float(value) for key, value in row.items() if value != ""}
            for row in csv.DictReader(handle)
        ]


def export_closed_loop_pgfplots_data() -> None:
    """Convert the flagship closed-loop run into `figures/closed_loop_run.dat`.

    Takes no parameters and returns nothing; it reads
    `results/closed_loop_local_1beacon.csv` and writes the first 61 rows
    (measurement steps 0..60 inclusive, the packet window plotted in the
    paper) as a matching `.dat` file:

    - step, robot_x/y, goal_error, target_error, beacon_position_rmse, and
      beacon_yaw_rmse are copied straight through from the CSV, renamed to
      the short pgfplots-friendly headers `x y goal target bpos byaw`.
    - `spread` is copied directly from the CSV's `spread` column, which the
      C++ run logs per step as the stored window's trajectory-spread margin
      S_v computed from the known measurement poses -- exactly the quantity
      the excitation supervisor compares against
      `supervised_spread_threshold`, so the plotted trace is the
      supervisor's own certificate rather than a plotting-side
      recomputation.
    """
    rows = load_numeric_rows(RESULTS / "closed_loop_local_1beacon.csv")[:61]

    out = ROOT / "figures" / "closed_loop_run.dat"
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("w", newline="\n") as handle:
        handle.write("step x y goal target bpos byaw spread\n")
        for row in rows:
            handle.write(
                f"{int(row['step'])} {row['robot_x']:.6f} {row['robot_y']:.6f} "
                f"{row['goal_error']:.8f} {row['target_error']:.8f} "
                f"{row['beacon_position_rmse']:.8f} {row['beacon_yaw_rmse']:.8f} "
                f"{row['spread']:.6f}\n"
            )
    print(out)


if __name__ == "__main__":
    export_closed_loop_pgfplots_data()
