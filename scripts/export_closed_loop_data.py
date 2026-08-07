"""Export the 60-step flagship closed-loop run as a pgfplots-readable table.

The paper draws Figs. 2-3 (closed-loop trajectory and error traces) natively
in pgfplots from the .dat file this writes, so fonts and math notation match
the paper text rather than a separately rendered raster image."""

from __future__ import annotations

import csv
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
RESULTS = ROOT / "results"


def load_numeric_rows(path: Path) -> list[dict[str, float]]:
    with path.open(newline="") as handle:
        return [
            {key: float(value) for key, value in row.items()}
            for row in csv.DictReader(handle)
        ]


def export_closed_loop_pgfplots_data() -> None:
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
