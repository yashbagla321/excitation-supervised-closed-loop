#!/usr/bin/env python3
"""Render a validation video from the Gazebo software-in-the-loop run's log.

Module responsibility: this script does no simulation of its own and talks
to no live ROS/Gazebo process. It replays the CSV logged by the ROS 2
supervised closed-loop node while it steered the Gazebo vehicle
(ros2_ws/src/cooperative_localization_gz, results/ros_gz/
closed_loop_gz_run.csv: Gazebo-odometry poses plus the node's online
estimates), frame-by-frame into an MP4 (map + live error plot + status
text), plus a single still thumbnail. It is the animated counterpart of
scripts/render_gazebo_panel.py, which renders one frozen frame of the same
data as a static print figure; the on-screen text is accurate to what this
script actually does — it paints PIL graphics from the logged run's
numbers, not screen footage of the Gazebo GUI. Pass --trajectory to replay
a batch-simulator CSV instead (the per-step beacon-estimate columns are
then read from --beacons)."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

import imageio.v2 as imageio
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from plot_fonts import load_font

WIDTH = 1280
HEIGHT = 720
TARGET_TRUE = (1.2, -0.75)
WORLD_X = (-4.0, 4.0)
WORLD_Y = (-3.2, 3.4)


# Module-level font cache: every frame reuses the same handful of font sizes
# for headers/labels/legends, so they are loaded once here rather than
# re-loaded (and re-probed for font-file existence) on every render_frame()
# call across a video that may run to hundreds of frames.
FONT_XL = load_font(34, True)
FONT_L = load_font(24, True)
FONT_M = load_font(20)
FONT_MB = load_font(20, True)
FONT_S = load_font(16)
FONT_SB = load_font(16, True)
FONT_XS = load_font(13)


def read_rows(path: Path) -> list[dict[str, float]]:
    """Read the per-step closed-loop trajectory CSV into a list of row dicts.

    Each row of the CSV (step index, robot position, target estimate,
    error metrics, etc., as written by adaptive::write_closed_loop_csv) is
    converted to a dict of column name -> float. An empty string value is
    mapped to NaN rather than raising, so an optional/missing field in a row
    (if any) does not abort the whole read.

    Parameters:
        path: path to the trajectory CSV (default: results/closed_loop_local_1beacon.csv).
    Returns: one dict per row, in file order, keyed by CSV column name.
    """
    with path.open(newline="") as f:
        reader = csv.DictReader(f)
        rows: list[dict[str, float]] = []
        for row in reader:
            parsed = {}
            for key, value in row.items():
                parsed[key] = float(value) if value else float("nan")
            rows.append(parsed)
        return rows


def read_beacons(path: Path) -> list[dict[str, float]]:
    """Read the beacon-estimate CSV (true vs. estimated beacon pose) into row dicts.

    Same float-parsing/NaN-on-empty behavior as read_rows(), kept as a
    separate function because it reads a semantically different file
    (one row per beacon, with true_x/true_y/estimate_x/estimate_y/... columns
    from adaptive::write_beacon_estimate_csv) rather than one row per
    simulation step.

    Parameters:
        path: path to the beacon-estimate CSV (default:
            results/beacon_estimates_local_1beacon.csv).
    Returns: one dict per beacon row, keyed by CSV column name.
    """
    with path.open(newline="") as f:
        reader = csv.DictReader(f)
        out = []
        for row in reader:
            out.append({k: float(v) if v else float("nan") for k, v in row.items()})
        return out


def world_to_px(x: float, y: float, box: tuple[int, int, int, int]) -> tuple[int, int]:
    """Map a world-frame (x, y) coordinate to an integer pixel inside `box`.

    Linearly rescales x from the module-level WORLD_X range and y from
    WORLD_Y into the pixel rectangle `box = (left, top, right, bottom)`,
    flipping the y axis (image rows increase downward, world y increases
    upward) and rounding to the nearest pixel for crisp PIL drawing calls.

    Parameters:
        x, y: world-frame coordinates (meters) to project.
        box: destination pixel rectangle as (left, top, right, bottom).
    Returns: the (px, py) pixel coordinates, rounded to the nearest int.
    """
    left, top, right, bottom = box
    px = left + (x - WORLD_X[0]) / (WORLD_X[1] - WORLD_X[0]) * (right - left)
    py = bottom - (y - WORLD_Y[0]) / (WORLD_Y[1] - WORLD_Y[0]) * (bottom - top)
    return int(round(px)), int(round(py))


def draw_grid(draw: ImageDraw.ImageDraw, box: tuple[int, int, int, int]) -> None:
    """Draw the map panel's background card, axis gridlines, and axis labels.

    Fills `box` with a light rounded-rectangle background, then draws one
    vertical gridline per integer world-x coordinate and one horizontal
    gridline per integer world-y coordinate (each covering the whole span
    of WORLD_X/WORLD_Y that is visible in `box`), darkening the line that
    passes through the corresponding world axis (x=0 or y=0) so the origin
    is easy to spot. Meant to be called once per frame before any
    trajectory/marker drawing, so the grid sits underneath the data.

    Parameters:
        draw: the ImageDraw context to draw into (mutated).
        box: the map panel's pixel rectangle as (left, top, right, bottom).
    Returns: nothing; `draw`'s target image is modified in place.
    """
    left, top, right, bottom = box
    draw.rounded_rectangle(box, radius=10, fill=(250, 250, 247), outline=(211, 216, 222), width=2)
    for x in range(math.ceil(WORLD_X[0]), math.floor(WORLD_X[1]) + 1):
        px, _ = world_to_px(x, 0, box)
        color = (178, 184, 193) if x == 0 else (230, 232, 235)
        width = 2 if x == 0 else 1
        draw.line([(px, top + 8), (px, bottom - 8)], fill=color, width=width)
        draw.text((px - 4, bottom + 8), str(x), fill=(85, 91, 100), font=FONT_XS)
    for y in range(math.ceil(WORLD_Y[0]), math.floor(WORLD_Y[1]) + 1):
        _, py = world_to_px(0, y, box)
        color = (178, 184, 193) if y == 0 else (230, 232, 235)
        width = 2 if y == 0 else 1
        draw.line([(left + 8, py), (right - 8, py)], fill=color, width=width)
        draw.text((left - 22, py - 8), str(y), fill=(85, 91, 100), font=FONT_XS)
    draw.text(((left + right) // 2 - 38, bottom + 30), "x position (m)", fill=(50, 56, 64), font=FONT_S)
    draw.text((left - 56, (top + bottom) // 2 - 44), "y position (m)", fill=(50, 56, 64), font=FONT_S)


def draw_polyline(
    draw: ImageDraw.ImageDraw,
    points: list[tuple[float, float]],
    box: tuple[int, int, int, int],
    color: tuple[int, int, int],
    width: int,
) -> None:
    """Draw a world-frame polyline (e.g. the traveled path) into the map panel.

    Projects every (x, y) point through world_to_px() and connects them with
    a single curved-join line. A no-op when fewer than 2 points are given
    (e.g. rendering frame 0, before any path has been traveled), since PIL's
    draw.line() needs at least two points.

    Parameters:
        draw: the ImageDraw context to draw into (mutated).
        points: world-frame (x, y) coordinates to connect, in order.
        box: the map panel's pixel rectangle, passed through to world_to_px().
        color: line color as an (r, g, b) tuple.
        width: line width in pixels.
    Returns: nothing; `draw`'s target image is modified in place.
    """
    if len(points) < 2:
        return
    draw.line([world_to_px(x, y, box) for x, y in points], fill=color, width=width, joint="curve")


def draw_marker(
    draw: ImageDraw.ImageDraw,
    xy: tuple[int, int],
    color: tuple[int, int, int],
    shape: str,
    label: str,
    label_offset: tuple[int, int] = (9, -19),
) -> None:
    """Draw one labeled map marker (vehicle, target, estimate, or beacon).

    `shape` selects which glyph is drawn at pixel position `xy`: "circle"
    for the vehicle, "star" for the true target, "diamond" for the target
    estimate, and "square" for beacon markers (true and estimated, in
    different colors — see render_frame()). Every shape is followed by a
    text label offset from the marker center by `label_offset`, which
    callers tune per-marker to keep labels from overlapping the glyph or
    each other.

    Parameters:
        draw: the ImageDraw context to draw into (mutated).
        xy: pixel coordinates of the marker center.
        color: fill color for both the glyph and its label text.
        shape: one of "circle", "square", "diamond", "star"; any other value
            draws only the label with no glyph (falls through the if/elif
            chain silently).
        label: text drawn next to the marker.
        label_offset: (dx, dy) pixel offset from `xy` at which to draw `label`.
    Returns: nothing; `draw`'s target image is modified in place.
    """
    x, y = xy
    if shape == "circle":
        draw.ellipse((x - 7, y - 7, x + 7, y + 7), fill=color, outline=(20, 25, 30), width=2)
    elif shape == "square":
        draw.rectangle((x - 7, y - 7, x + 7, y + 7), fill=color, outline=(20, 25, 30), width=2)
    elif shape == "diamond":
        draw.polygon([(x, y - 9), (x + 9, y), (x, y + 9), (x - 9, y)], fill=color, outline=(20, 25, 30))
    elif shape == "star":
        pts = []
        for i in range(10):
            r = 11 if i % 2 == 0 else 5
            a = -math.pi / 2 + i * math.pi / 5
            pts.append((x + r * math.cos(a), y + r * math.sin(a)))
        draw.polygon(pts, fill=color, outline=(20, 25, 30))
    draw.text((x + label_offset[0], y + label_offset[1]), label, fill=color, font=FONT_SB)


def draw_error_plot(
    draw: ImageDraw.ImageDraw,
    rows: list[dict[str, float]],
    idx: int,
    box: tuple[int, int, int, int],
) -> None:
    """Draw the log-scale convergence-error subplot for the current frame.

    Plots the same four series as scripts/render_gazebo_panel.py (robot-target
    distance, target-estimate error, beacon position RMSE, beacon yaw RMSE)
    on a shared log10 y-axis, over a fixed window of the first `n = min(60,
    len(rows))` steps (the video does not rescale the x-axis as more frames
    play beyond step 60; the plotted curves always stop at step n-1). Values
    are floored to 1e-4 before taking the log so a converged-to-zero error
    still has a finite y position. A vertical cursor line tracks the current
    frame index `idx` while `idx < n`, then stays pinned at the right edge
    (x position for step n-1) for any later frame, since `marker_x` is
    computed from `min(idx, n - 1)`. A small legend below the plot names
    each series/color.

    Parameters:
        draw: the ImageDraw context to draw into (mutated).
        rows: the full per-step trajectory rows (as returned by read_rows()).
        idx: the current frame's step index, used to place the cursor line.
        box: this subplot's pixel rectangle as (left, top, right, bottom).
    Returns: nothing; `draw`'s target image is modified in place.
    """
    left, top, right, bottom = box
    draw.rounded_rectangle(box, radius=8, fill=(255, 255, 255), outline=(211, 216, 222), width=2)
    draw.text((left + 18, top + 12), "Convergence errors, first 60 measurement steps", fill=(32, 39, 48), font=FONT_MB)
    plot = (left + 58, top + 54, right - 24, bottom - 76)
    pleft, ptop, pright, pbottom = plot
    draw.rectangle(plot, outline=(225, 228, 232), width=1)

    series = [
        ("goal_error", "robot-target", (43, 111, 246)),
        ("target_error", "target est.", (16, 142, 78)),
        ("beacon_position_rmse", "beacon pos.", (125, 90, 233)),
        ("beacon_yaw_rmse", "beacon yaw", (230, 64, 107)),
    ]
    n = min(60, len(rows))
    min_log, max_log = -3.05, 0.82

    for grid in [-3, -2, -1, 0]:
        y = pbottom - (grid - min_log) / (max_log - min_log) * (pbottom - ptop)
        draw.line([(pleft, y), (pright, y)], fill=(235, 237, 240), width=1)
        draw.text((left + 12, int(y) - 8), f"1e{grid}", fill=(95, 101, 110), font=FONT_XS)

    for key, label, color in series:
        vals = [max(rows[j][key], 1e-4) for j in range(n)]
        pts = []
        for j, value in enumerate(vals):
            x = pleft + j / (n - 1) * (pright - pleft)
            y = pbottom - (math.log10(value) - min_log) / (max_log - min_log) * (pbottom - ptop)
            pts.append((x, y))
        draw.line(pts, fill=color, width=3)

    marker_x = pleft + min(idx, n - 1) / (n - 1) * (pright - pleft)
    draw.line([(marker_x, ptop), (marker_x, pbottom)], fill=(30, 37, 46), width=2)
    draw.text((pleft, bottom - 56), "measurement step k", fill=(68, 75, 84), font=FONT_XS)
    legend_positions = [
        (left + 18, bottom - 36),
        (left + 190, bottom - 36),
        (left + 18, bottom - 16),
        (left + 190, bottom - 16),
    ]
    for (_, label, color), (legend_x, legend_y) in zip(series, legend_positions):
        draw.line([(legend_x, legend_y), (legend_x + 22, legend_y)], fill=color, width=4)
        draw.text((legend_x + 28, legend_y - 8), label, fill=(54, 60, 70), font=FONT_XS)


def draw_text_panel(
    draw: ImageDraw.ImageDraw,
    row: dict[str, float],
    idx: int,
    total: int,
    box: tuple[int, int, int, int],
) -> None:
    """Draw the status/caption card: step counter, live metrics, and disclaimers.

    Renders a fixed block of caption lines describing what the video is
    (same estimator core as the batch Monte Carlo/robustness suite; a
    simulated ROS 2/Gazebo validation track, not a physical-hardware
    experiment) interleaved with the current frame's live numeric readout
    (step index out of `total - 1`, and the same four error metrics plotted
    in draw_error_plot: goal error, target error, beacon position RMSE,
    beacon yaw RMSE). Metric lines are drawn bold (FONT_SB); the rest of the
    caption text is regular weight (FONT_S). `idx` is accepted for a
    consistent call signature with the other per-frame draw_*() helpers but
    is not read directly here — `row` (already indexed by the caller) supplies
    the numbers actually drawn.

    Parameters:
        draw: the ImageDraw context to draw into (mutated).
        row: the current frame's trajectory row (one entry of `rows`).
        idx: current frame index (unused in this function's own logic).
        total: total number of trajectory rows, used for the "n / total-1" caption.
        box: this panel's pixel rectangle as (left, top, right, bottom).
    Returns: nothing; `draw`'s target image is modified in place.
    """
    left, top, right, bottom = box
    draw.rounded_rectangle(box, radius=8, fill=(247, 249, 252), outline=(211, 216, 222), width=2)
    draw.text((left + 18, top + 14), "ROS 2 / Gazebo validation track", fill=(32, 39, 48), font=FONT_MB)
    lines = [
        "Modern Gazebo package uses the same C++ estimator core",
        "as the standalone Monte Carlo and robustness suite.",
        "",
        f"Step: {int(row['step']):03d} / {total - 1}",
        f"Robot-target distance: {row['goal_error']:.4f} m",
        f"Target estimate error: {row['target_error']:.4f} m",
        f"Beacon position RMSE: {row['beacon_position_rmse']:.4f} m",
        f"Beacon yaw RMSE: {row['beacon_yaw_rmse']:.5f} rad",
        "",
        "Supplementary video: simulated Gazebo/ROS validation,",
        "not a physical hardware experiment.",
    ]
    y = top + 52
    for line in lines:
        font = FONT_SB if line.startswith(("Step", "Robot", "Target", "Beacon")) else FONT_S
        draw.text((left + 18, y), line, fill=(55, 63, 74), font=font)
        y += 25 if line else 13


def render_frame(
    rows: list[dict[str, float]],
    beacons: list[dict[str, float]],
    idx: int,
) -> Image.Image:
    """Compose one full 1280x720 video frame at step `idx`.

    Lays out the whole frame: a dark title bar, the map panel (left), the
    error-convergence plot and status/caption panel (right column, stacked),
    and a one-line disclaimer along the bottom. This is the single function
    called once per output frame by main()'s encoding loop, and is also
    reused directly to render the still thumbnail image.

    Map panel contents, in draw order:
      1. draw_grid() for the background grid.
      2. The full trajectory in light grey (`full_path`, all rows) so the
         whole run's extent is always visible, with the traveled prefix up
         to and including `idx` (`current_path`) redrawn in bold blue on top
         — the "played so far" portion of the path.
      3. A connecting line from the current robot position `q(k)` to the
         current target estimate `p_hat`, then markers for the robot
         (circle), the fixed true target (star, at module-level TARGET_TRUE
         = (1.2, -0.75), matching World::make_world in src/World.cpp), and
         the target estimate (diamond).
      4. The first beacon's true and estimated position (square markers, two
         colors) — `beacons[:1]` because this flagship scenario has exactly
         one beacon; the slice is defensive rather than a real multi-beacon
         selection.

    The right column then draws draw_error_plot() (fixed 60-step error
    traces with a cursor at `idx`) above draw_text_panel() (live numeric
    readout for `rows[idx]` plus the caption text), and a closing disclaimer
    line is drawn along the bottom of the frame.

    Parameters:
        rows: full per-step trajectory rows (from read_rows()).
        beacons: beacon true/estimate rows (from read_beacons()).
        idx: index into `rows` for the step this frame represents.
    Returns: a new PIL Image (RGB, 1280x720) for this frame.
    """
    img = Image.new("RGB", (WIDTH, HEIGHT), (238, 241, 245))
    draw = ImageDraw.Draw(img)
    draw.rectangle((0, 0, WIDTH, 72), fill=(24, 32, 43))
    draw.text((32, 18), "Hidden-target localization with one unknown-pose range-bearing beacon", fill=(255, 255, 255), font=FONT_XL)
    draw.text((34, 52), "Top-down validation view generated from the shared C++ estimator outputs used by ROS/Gazebo nodes", fill=(187, 198, 211), font=FONT_S)

    map_box = (78, 112, 708, 612)
    draw_grid(draw, map_box)
    full_path = [(r["robot_x"], r["robot_y"]) for r in rows]
    current_path = full_path[: idx + 1]
    draw_polyline(draw, full_path, map_box, (198, 205, 214), 3)
    draw_polyline(draw, current_path, map_box, (43, 111, 246), 5)

    row = rows[idx]
    robot = (row["robot_x"], row["robot_y"])
    target_est = (row["target_estimate_x"], row["target_estimate_y"])
    draw.line([world_to_px(*robot, map_box), world_to_px(*target_est, map_box)], fill=(80, 88, 99), width=2)
    draw_marker(draw, world_to_px(*robot, map_box), (43, 111, 246), "circle", "q(k)", (-42, -27))
    draw_marker(draw, world_to_px(*TARGET_TRUE, map_box), (245, 158, 11), "star", "target p", (12, -36))
    draw_marker(draw, world_to_px(*target_est, map_box), (16, 142, 78), "diamond", "estimate p_hat", (13, 7))

    for beacon in beacons[:1]:
        true_xy = (beacon["true_x"], beacon["true_y"])
        # Prefer the per-step beacon estimate logged by the Gazebo node (so
        # the marker animates with the run); fall back to the static final
        # estimate from the separate beacon-estimates CSV.
        if "beacon_estimate_x" in row:
            est_xy = (row["beacon_estimate_x"], row["beacon_estimate_y"])
        else:
            est_xy = (beacon["estimate_x"], beacon["estimate_y"])
        draw_marker(draw, world_to_px(*true_xy, map_box), (234, 88, 12), "square", "true beacon")
        draw_marker(draw, world_to_px(*est_xy, map_box), (125, 90, 233), "square", "estimated beacon", (9, 8))

    draw_error_plot(draw, rows, idx, (748, 112, 1216, 388))
    draw_text_panel(draw, row, idx, len(rows), (748, 414, 1216, 660))
    draw.text((84, 676), "Software-in-the-loop validation (ROS 2 node + Gazebo physics). It does not claim physical robot sensing.", fill=(70, 78, 88), font=FONT_S)
    return img


def main() -> None:
    """CLI entry point: read trajectory/beacon CSVs and encode the replay video.

    Takes no parameters (arguments come from sys.argv via argparse) and
    returns nothing. Recognized flags, all optional with defaults matching
    the flagship one-beacon run's output paths:
      --trajectory: closed-loop trajectory CSV (default
        results/closed_loop_local_1beacon.csv).
      --beacons: beacon true/estimate CSV (default
        results/beacon_estimates_local_1beacon.csv).
      --output: destination MP4 path (default
        figures/ros_gazebo_validation_demo.mp4).
      --thumbnail: destination still-frame PNG path (default
        figures/ros_gazebo_validation_thumbnail.png).
      --fps: output video frame rate (default 20).

    Sequence:
      1. Load both CSVs via read_rows()/read_beacons(); ensure the output
         and thumbnail directories exist.
      2. Build the frame-index sequence to encode: one frame per row of the
         trajectory (`frame_indices`), then `fps * 2` extra frames all
         repeating the final index (`hold`) so the video pauses for two
         seconds on the converged end state instead of cutting immediately.
      3. Open an H.264 MP4 writer (imageio, libx264, quality=8) and render +
         append one frame per index in `frame_indices + hold`.
      4. Separately render a single still frame at step `min(45, len(rows) -
         1)` (a fixed illustrative step, clamped so it never indexes past
         the end of a shorter-than-46-step run) and save it as the
         thumbnail.
      5. Print the output video and thumbnail paths.
    """
    parser = argparse.ArgumentParser()
    parser.add_argument("--trajectory", type=Path, default=Path("results/ros_gz/closed_loop_gz_run.csv"))
    parser.add_argument("--beacons", type=Path, default=Path("results/beacon_estimates_local_1beacon.csv"))
    parser.add_argument("--output", type=Path, default=Path("figures/ros_gazebo_validation_demo.mp4"))
    parser.add_argument("--thumbnail", type=Path, default=Path("figures/ros_gazebo_validation_thumbnail.png"))
    parser.add_argument("--fps", type=int, default=20)
    args = parser.parse_args()

    rows = read_rows(args.trajectory)
    if rows and "beacon_estimate_x" in rows[0]:
        # Gazebo-node log: the beacon estimate is logged per step, so the
        # separate beacon-estimates CSV is not needed. The true beacon pose
        # comes from the world definition (make_world in src/World.cpp).
        beacons = [{
            "true_x": -2.2,
            "true_y": -1.4,
            "estimate_x": rows[-1]["beacon_estimate_x"],
            "estimate_y": rows[-1]["beacon_estimate_y"],
        }]
    else:
        beacons = read_beacons(args.beacons)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.thumbnail.parent.mkdir(parents=True, exist_ok=True)

    # Play every recorded step once, then hold on the final (most-converged)
    # frame for two seconds so the video does not end abruptly mid-motion.
    frame_indices = list(range(len(rows)))
    hold = [len(rows) - 1] * (args.fps * 2)
    with imageio.get_writer(args.output, fps=args.fps, codec="libx264", quality=8, macro_block_size=16) as writer:
        for idx in frame_indices + hold:
            frame = render_frame(rows, beacons, idx)
            writer.append_data(np.asarray(frame))

    render_frame(rows, beacons, min(45, len(rows) - 1)).save(args.thumbnail)
    print(args.output)
    print(args.thumbnail)


if __name__ == "__main__":
    main()
