"""Render the two-panel software-in-the-loop summary (map view + convergence
errors) used in the paper's Fig. 4, drawn at print resolution directly from
the replay CSVs rather than cropped from the demo video's thumbnail."""

from __future__ import annotations

import csv
import math
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[1]
RESULTS = ROOT / "results"


def load_font(size: int, bold: bool = False) -> ImageFont.ImageFont:
    candidates = [
        Path("C:/Windows/Fonts/arialbd.ttf" if bold else "C:/Windows/Fonts/arial.ttf"),
        Path("C:/Windows/Fonts/calibrib.ttf" if bold else "C:/Windows/Fonts/calibri.ttf"),
    ]
    for candidate in candidates:
        if candidate.exists():
            return ImageFont.truetype(str(candidate), size)
    return ImageFont.load_default()


def load_numeric_rows(path: Path) -> list[dict[str, float]]:
    with path.open(newline="") as handle:
        return [
            {key: float(value) for key, value in row.items()}
            for row in csv.DictReader(handle)
        ]


def draw_vertical_text(
    image: Image.Image,
    xy: tuple[int, int],
    text: str,
    font: ImageFont.ImageFont,
    fill: str,
) -> None:
    bounds = font.getbbox(text)
    text_layer = Image.new(
        "RGBA", (bounds[2] - bounds[0] + 8, bounds[3] - bounds[1] + 8), (0, 0, 0, 0)
    )
    ImageDraw.Draw(text_layer).text(
        (4 - bounds[0], 4 - bounds[1]), text, font=font, fill=fill
    )
    text_layer = text_layer.rotate(90, expand=True)
    image.paste(text_layer, xy, text_layer)


def render_gazebo_panel() -> None:
    rows = load_numeric_rows(RESULTS / "closed_loop_local_1beacon.csv")[:60]
    with (RESULTS / "beacon_estimates_local_1beacon.csv").open(newline="") as handle:
        beacon = {key: float(value) for key, value in next(csv.DictReader(handle)).items()}
    replay_step = 25  # mid-replay cursor: converging but still clearly separated

    image = Image.new("RGB", (2010, 860), "#ffffff")
    draw = ImageDraw.Draw(image)
    tick_font = load_font(28)
    label_font = load_font(33)
    tag_font = load_font(30, True)

    world_x = (-3.7, 2.7)
    world_y = (-1.35, 3.35)
    map_plot = (110, 40, 950, 738)

    def to_map(x: float, y: float) -> tuple[float, float]:
        left, top, right, bottom = map_plot
        px = left + (x - world_x[0]) / (world_x[1] - world_x[0]) * (right - left)
        py = bottom - (y - world_y[0]) / (world_y[1] - world_y[0]) * (bottom - top)
        return px, py

    for grid_x in range(-3, 3):
        px, _ = to_map(grid_x, 0)
        axis = grid_x == 0
        draw.line((px, map_plot[1], px, map_plot[3]), fill="#94a3b8" if axis else "#e2e8f0", width=2 if axis else 1)
        label = str(grid_x)
        width = draw.textlength(label, font=tick_font)
        draw.text((px - width / 2, map_plot[3] + 12), label, fill="#64748b", font=tick_font)
    for grid_y in range(-2, 4):
        _, py = to_map(0, grid_y)
        axis = grid_y == 0
        draw.line((map_plot[0], py, map_plot[2], py), fill="#94a3b8" if axis else "#e2e8f0", width=2 if axis else 1)
        label = str(grid_y)
        width = draw.textlength(label, font=tick_font)
        draw.text((map_plot[0] - width - 16, py - 17), label, fill="#64748b", font=tick_font)
    draw.rectangle(map_plot, outline="#64748b", width=2)
    x_label = "x position (m)"
    draw.text(
        ((map_plot[0] + map_plot[2]) / 2 - draw.textlength(x_label, font=label_font) / 2, map_plot[3] + 56),
        x_label, fill="#334155", font=label_font,
    )
    draw_vertical_text(image, (10, (map_plot[1] + map_plot[3]) // 2 - 130), "y position (m)", label_font, "#334155")

    path = [to_map(row["robot_x"], row["robot_y"]) for row in rows]
    draw.line(path, fill="#cbd5e1", width=4, joint="curve")
    draw.line(path[: replay_step + 1], fill="#2563eb", width=7, joint="curve")

    robot = to_map(rows[replay_step]["robot_x"], rows[replay_step]["robot_y"])
    estimate = to_map(rows[replay_step]["target_estimate_x"], rows[replay_step]["target_estimate_y"])
    target = to_map(1.2, -0.75)
    draw.line((robot, estimate), fill="#475569", width=3)

    def star(center: tuple[float, float], radius: float, color: str) -> None:
        points = []
        for i in range(10):
            r = radius if i % 2 == 0 else radius * 0.45
            angle = -math.pi / 2 + i * math.pi / 5
            points.append((center[0] + r * math.cos(angle), center[1] + r * math.sin(angle)))
        draw.polygon(points, fill=color, outline="#1f2937")

    star(target, 20, "#f59e0b")
    draw.polygon(
        [(estimate[0], estimate[1] - 16), (estimate[0] + 16, estimate[1]),
         (estimate[0], estimate[1] + 16), (estimate[0] - 16, estimate[1])],
        fill="#059669", outline="#1f2937",
    )
    draw.ellipse((robot[0] - 13, robot[1] - 13, robot[0] + 13, robot[1] + 13), fill="#2563eb", outline="#1f2937", width=3)
    true_beacon = to_map(beacon["true_x"], beacon["true_y"])
    est_beacon = to_map(beacon["estimate_x"], beacon["estimate_y"])
    draw.rectangle((true_beacon[0] - 13, true_beacon[1] - 13, true_beacon[0] + 13, true_beacon[1] + 13), fill="#ea580c", outline="#1f2937", width=3)
    draw.rectangle((est_beacon[0] - 13, est_beacon[1] - 13, est_beacon[0] + 13, est_beacon[1] + 13), fill="#7c3aed", outline="#1f2937", width=3)

    draw.text((robot[0] - draw.textlength("vehicle", font=tag_font) - 22, robot[1] - 44), "vehicle", fill="#2563eb", font=tag_font)
    draw.text((target[0] + 24, target[1] - 44), "true target", fill="#b45309", font=tag_font)
    draw.text((estimate[0] + 24, estimate[1] + 8), "estimate", fill="#059669", font=tag_font)
    draw.text((true_beacon[0] + 24, true_beacon[1] - 40), "true beacon", fill="#ea580c", font=tag_font)
    draw.text((est_beacon[0] + 24, est_beacon[1] + 6), "estimated beacon", fill="#7c3aed", font=tag_font)

    err_plot = (1180, 140, 1960, 738)
    log_min, log_max = -4.0, 1.0

    def to_err(step: int, value: float) -> tuple[float, float]:
        left, top, right, bottom = err_plot
        px = left + step / (len(rows) - 1) * (right - left)
        clipped = min(max(math.log10(max(value, 10.0**log_min)), log_min), log_max)
        py = bottom - (clipped - log_min) / (log_max - log_min) * (bottom - top)
        return px, py

    exponent_font = load_font(20)
    for exponent in range(-4, 2):
        _, py = to_err(0, 10.0**exponent)
        draw.line((err_plot[0], py, err_plot[2], py), fill="#e2e8f0", width=1)
        base_width = draw.textlength("10", font=tick_font)
        exp_text = str(exponent)
        exp_width = draw.textlength(exp_text, font=exponent_font)
        anchor_x = err_plot[0] - base_width - exp_width - 16
        draw.text((anchor_x, py - 15), "10", fill="#64748b", font=tick_font)
        draw.text((anchor_x + base_width + 1, py - 26), exp_text, fill="#64748b", font=exponent_font)
    for step in (0, 15, 30, 45, 59):
        px, _ = to_err(step, 10.0**log_min)
        draw.line((px, err_plot[1], px, err_plot[3]), fill="#e2e8f0", width=1)
        label = str(step)
        width = draw.textlength(label, font=tick_font)
        draw.text((px - width / 2, err_plot[3] + 12), label, fill="#64748b", font=tick_font)
    draw.rectangle(err_plot, outline="#64748b", width=2)

    series = [
        ("goal_error", "robot-target", "#2563eb"),
        ("target_error", "target est.", "#059669"),
        ("beacon_position_rmse", "beacon pos.", "#7c3aed"),
        ("beacon_yaw_rmse", "beacon yaw", "#e11d48"),
    ]
    legend_positions = [(1180, 52), (1590, 52), (1180, 98), (1590, 98)]
    for (key, label, color), (legend_x, legend_y) in zip(series, legend_positions):
        draw.line((legend_x, legend_y, legend_x + 42, legend_y), fill=color, width=7)
        draw.text((legend_x + 54, legend_y - 17), label, fill="#334155", font=tick_font)
        points = [to_err(index, row[key]) for index, row in enumerate(rows)]
        draw.line(points, fill=color, width=5, joint="curve")

    marker_x, _ = to_err(replay_step, 1.0)
    draw.line((marker_x, err_plot[1], marker_x, err_plot[3]), fill="#1f2937", width=3)

    x_label = "measurement step"
    draw.text(
        ((err_plot[0] + err_plot[2]) / 2 - draw.textlength(x_label, font=label_font) / 2, err_plot[3] + 56),
        x_label, fill="#334155", font=label_font,
    )
    draw_vertical_text(image, (1078, (err_plot[1] + err_plot[3]) // 2 - 50), "error", label_font, "#334155")

    out = ROOT / "figures" / "ros_gazebo_validation_compact.png"
    out.parent.mkdir(parents=True, exist_ok=True)
    image.save(out)
    print(out)


if __name__ == "__main__":
    render_gazebo_panel()
