"""Render the two-panel software-in-the-loop summary (map view + convergence
errors) used in the paper's Fig. 4, drawn at print resolution directly from
the Gazebo run's logged CSV.

Module responsibility: this script does no simulation. It is a static,
single-image renderer: it reads the CSV logged by the ROS 2 supervised
closed-loop node while it steered the Gazebo vehicle
(results/ros_gz/closed_loop_gz_run.csv, written by
ros2_ws/src/cooperative_localization_gz's supervised_closed_loop_node: the
vehicle poses are Gazebo odometry and the estimates are what the node
computed online from the delayed packets) and paints a fixed-layout,
two-panel PNG (world-frame map on the left, log-scale convergence-error
traces on the right) frozen at one replay step. It shares its pixel-mapping
and drawing approach with scripts/render_gazebo_validation_video.py, which
renders the same data as an animated sequence instead of one static frame;
the two are not required to produce visually identical output."""

from __future__ import annotations

import csv
import math
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

from plot_fonts import load_font

ROOT = Path(__file__).resolve().parents[1]
RESULTS = ROOT / "results"


def load_numeric_rows(path: Path) -> list[dict[str, float]]:
    """Read a CSV file written by the C++ binary into a list of row dicts.

    Every column is parsed as a float, matching the all-numeric layout of
    the closed-loop and beacon-estimate CSVs this script consumes.

    Parameters:
        path: CSV file to read, with a header row naming each column.
    Returns: one dict per data row, mapping column name to its float value.
    """
    with path.open(newline="") as handle:
        return [
            {key: float(value) for key, value in row.items() if value != ""}
            for row in csv.DictReader(handle)
        ]


def draw_vertical_text(
    image: Image.Image,
    xy: tuple[int, int],
    text: str,
    font: ImageFont.ImageFont,
    fill: str,
) -> None:
    """Draw text rotated 90 degrees, for the panels' vertical axis labels.

    Pillow has no built-in rotated-text draw call, so this renders the text
    normally onto a small transparent scratch layer sized to the text's
    bounding box, rotates that layer 90 degrees, and pastes the result onto
    the target image using itself as the paste mask (so the transparent
    background does not overwrite what is already on the image).

    Parameters:
        image: destination image to paste the rotated text onto (mutated).
        xy: top-left pixel coordinate at which to paste the rotated layer.
        text: the string to render.
        font: font to render the text with.
        fill: text color (any Pillow-accepted color spec).
    Returns: nothing; the image is modified in place.
    """
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
    """Render `figures/ros_gazebo_validation_compact.png`, the two-panel figure.

    Takes no parameters and returns nothing. Reads packets 0..60 of
    `results/ros_gz/closed_loop_gz_run.csv` (the Gazebo software-in-the-loop
    run logged by the ROS 2 supervised closed-loop node), then draws:

    - Left panel ("map"): the full 60-packet vehicle path in light grey,
      with the prefix up to `replay_step` (25, chosen as a point where the
      estimate has visibly started converging but is still clearly separated
      from the truth — a deliberate mid-run snapshot, not the final step)
      highlighted in blue. Markers show the vehicle position, the true
      target (fixed at (1.2, -0.75), matching World::make_world's hardcoded
      target — see src/World.cpp), the current target estimate, and the true
      vs. estimated beacon position (true pose from the world definition,
      estimate from the run's final logged beacon estimate), connected by a
      line from vehicle to target estimate.
    - Right panel ("errors"): log10-scale traces (clipped to [1e-4, 1e1])
      of all 60 packets' goal error (robot-to-target distance), target
      estimation error, beacon position RMSE, and beacon yaw RMSE, each on
      its own color, with a vertical marker line at `replay_step` tying the
      two panels to the same moment in the run.

    Both panels are built by hand with PIL primitives (lines, polygons,
    text) rather than a plotting library, so the output matches the paper's
    print typography/line weights exactly instead of a library's defaults.
    """
    rows = load_numeric_rows(RESULTS / "ros_gz" / "closed_loop_gz_run.csv")[:61]
    # True beacon pose from the world definition; estimated beacon from the
    # run's final logged estimate.
    beacon = {
        "true_x": -2.2,
        "true_y": -1.4,
        "estimate_x": rows[-1]["beacon_estimate_x"],
        "estimate_y": rows[-1]["beacon_estimate_y"],
    }
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
        """Map a world-frame (x, y) coordinate to a pixel inside `map_plot`.

        Linearly rescales x from `world_x` and y from `world_y` into the
        pixel rectangle `map_plot`, flipping y (image rows grow downward,
        world y grows upward) so the map reads top-is-north like the paper's
        figure.
        """
        left, top, right, bottom = map_plot
        px = left + (x - world_x[0]) / (world_x[1] - world_x[0]) * (right - left)
        py = bottom - (y - world_y[0]) / (world_y[1] - world_y[0]) * (bottom - top)
        return px, py

    # Draw the left panel's background grid, axis ticks, and axis labels
    # before any trajectory/marker overlay, so grid lines sit behind the data.
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

    # Full 60-step path in light grey (context for where the run goes overall),
    # with the prefix through replay_step redrawn in bold blue on top of it
    # (what has actually happened "so far" at this frozen replay moment).
    path = [to_map(row["robot_x"], row["robot_y"]) for row in rows]
    draw.line(path, fill="#cbd5e1", width=4, joint="curve")
    draw.line(path[: replay_step + 1], fill="#2563eb", width=7, joint="curve")

    robot = to_map(rows[replay_step]["robot_x"], rows[replay_step]["robot_y"])
    estimate = to_map(rows[replay_step]["target_estimate_x"], rows[replay_step]["target_estimate_y"])
    target = to_map(1.2, -0.75)
    draw.line((robot, estimate), fill="#475569", width=3)

    def star(center: tuple[float, float], radius: float, color: str) -> None:
        """Fill a 5-pointed star centered at `center` into the drawing.

        Alternates an outer radius `radius` and an inner radius `0.45 *
        radius` across 10 evenly-spaced angles (36 degrees apart, starting
        pointing straight up) to produce the star's 10 vertices, then fills
        the resulting polygon. Used to mark the true target location.
        """
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
        """Map (measurement step, error value) to a pixel in `err_plot`.

        The x-axis is linear in step index across the full 60-step window.
        The y-axis is log10-scaled: `value` is floored to 10**log_min before
        taking the log (so a zero or near-zero error still maps to a finite
        pixel instead of -inf) and the resulting exponent is clamped to
        [log_min, log_max] before being rescaled into the plot's pixel rows,
        so any convergence error that undershoots 1e-4 is still drawn
        pinned to the bottom of the axis rather than off-plot.
        """
        left, top, right, bottom = err_plot
        px = left + step / (len(rows) - 1) * (right - left)
        clipped = min(max(math.log10(max(value, 10.0**log_min)), log_min), log_max)
        py = bottom - (clipped - log_min) / (log_max - log_min) * (bottom - top)
        return px, py

    # Draw horizontal log-scale gridlines/labels ("10" + superscript-style
    # exponent, since PIL text has no true superscript) and vertical
    # step-index gridlines for the right panel before plotting the series.
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

    # The four convergence-error series this figure compares: robot-to-target
    # distance, target-estimate error, and the beacon position/yaw RMSEs that
    # the excitation-supervised controller is ultimately trying to drive down.
    # Each is drawn as its own colored polyline with a matching legend swatch.
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

    # Vertical cursor tying this panel to the replay_step highlighted in the
    # map panel, so a reader can see exactly how far each error had converged
    # at the moment shown on the map.
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
