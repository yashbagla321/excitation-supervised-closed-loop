#!/usr/bin/env python3
"""Render a ROS/Gazebo-style validation video from simulator CSV outputs."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

import imageio.v2 as imageio
import numpy as np
from PIL import Image, ImageDraw, ImageFont


WIDTH = 1280
HEIGHT = 720
TARGET_TRUE = (1.2, -0.75)
WORLD_X = (-4.0, 4.0)
WORLD_Y = (-3.2, 3.4)


def load_font(size: int, bold: bool = False) -> ImageFont.FreeTypeFont:
    candidates = [
        Path("C:/Windows/Fonts/arialbd.ttf" if bold else "C:/Windows/Fonts/arial.ttf"),
        Path("C:/Windows/Fonts/segoeuib.ttf" if bold else "C:/Windows/Fonts/segoeui.ttf"),
    ]
    for candidate in candidates:
        if candidate.exists():
            return ImageFont.truetype(str(candidate), size)
    return ImageFont.load_default()


FONT_XL = load_font(34, True)
FONT_L = load_font(24, True)
FONT_M = load_font(20)
FONT_MB = load_font(20, True)
FONT_S = load_font(16)
FONT_SB = load_font(16, True)
FONT_XS = load_font(13)


def read_rows(path: Path) -> list[dict[str, float]]:
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
    with path.open(newline="") as f:
        reader = csv.DictReader(f)
        out = []
        for row in reader:
            out.append({k: float(v) if v else float("nan") for k, v in row.items()})
        return out


def world_to_px(x: float, y: float, box: tuple[int, int, int, int]) -> tuple[int, int]:
    left, top, right, bottom = box
    px = left + (x - WORLD_X[0]) / (WORLD_X[1] - WORLD_X[0]) * (right - left)
    py = bottom - (y - WORLD_Y[0]) / (WORLD_Y[1] - WORLD_Y[0]) * (bottom - top)
    return int(round(px)), int(round(py))


def draw_grid(draw: ImageDraw.ImageDraw, box: tuple[int, int, int, int]) -> None:
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
        est_xy = (beacon["estimate_x"], beacon["estimate_y"])
        draw_marker(draw, world_to_px(*true_xy, map_box), (234, 88, 12), "square", "true beacon")
        draw_marker(draw, world_to_px(*est_xy, map_box), (125, 90, 233), "square", "estimated beacon", (9, 8))

    draw_error_plot(draw, rows, idx, (748, 112, 1216, 388))
    draw_text_panel(draw, row, idx, len(rows), (748, 414, 1216, 660))
    draw.text((84, 676), "Video evidence is middleware/simulation validation. It does not claim physical robot sensing.", fill=(70, 78, 88), font=FONT_S)
    return img


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--trajectory", type=Path, default=Path("results/closed_loop_local_1beacon.csv"))
    parser.add_argument("--beacons", type=Path, default=Path("results/beacon_estimates_local_1beacon.csv"))
    parser.add_argument("--output", type=Path, default=Path("figures/ros_gazebo_validation_demo.mp4"))
    parser.add_argument("--thumbnail", type=Path, default=Path("figures/ros_gazebo_validation_thumbnail.png"))
    parser.add_argument("--fps", type=int, default=20)
    args = parser.parse_args()

    rows = read_rows(args.trajectory)
    beacons = read_beacons(args.beacons)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.thumbnail.parent.mkdir(parents=True, exist_ok=True)

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
