#!/usr/bin/env python3

"""Plot SITL tracking, reference dynamics, XY path, and position error."""

import argparse
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.lines import Line2D


AXES = ("x", "y", "z")
COLORS = ("#0072B2", "#D55E00", "#009E73")


def quaternion_yaw(quaternion):
    norms = np.linalg.norm(quaternion, axis=1, keepdims=True)
    quaternion = quaternion / np.maximum(norms, np.finfo(float).eps)
    qw, qx, qy, qz = quaternion.T
    return np.arctan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz),
    )


def load_trial_data(csv_path, warmup):
    samples = np.genfromtxt(str(csv_path), delimiter=",", names=True)
    if samples.size == 0:
        raise RuntimeError("CSV file contains no samples: {}".format(csv_path))
    samples = np.atleast_1d(samples)
    samples = samples[
        np.concatenate(([True], np.diff(samples["stamp"]) > 0.0))
    ]
    if samples.size < 2:
        raise RuntimeError("CSV file contains too few increasing timestamps")
    time_all = samples["stamp"] - samples["stamp"][0]
    reference_velocity_all = np.column_stack(
        [samples["ref_v{}".format(axis)] for axis in AXES]
    )
    reference_acceleration_all = np.gradient(
        reference_velocity_all, time_all, axis=0
    )
    reference_quaternion_all = np.column_stack(
        [samples["ref_q{}".format(axis)] for axis in ("w", "x", "y", "z")]
    )
    actual_quaternion_all = np.column_stack(
        [samples["actual_q{}".format(axis)] for axis in ("w", "x", "y", "z")]
    )
    selected = time_all >= warmup
    if np.count_nonzero(selected) < 2:
        raise RuntimeError("too few samples remain after the warm-up interval")

    time = time_all[selected]
    time -= time[0]
    reference = np.column_stack(
        [samples["ref_{}".format(axis)][selected] for axis in AXES]
    )
    actual = np.column_stack(
        [samples["actual_{}".format(axis)][selected] for axis in AXES]
    )
    reference_velocity = reference_velocity_all[selected]
    reference_acceleration = reference_acceleration_all[selected]
    reference_yaw = quaternion_yaw(reference_quaternion_all[selected])
    actual_yaw = quaternion_yaw(actual_quaternion_all[selected])
    return (
        time,
        reference,
        actual,
        reference_velocity,
        reference_acceleration,
        reference_yaw,
        actual_yaw,
    )


def plot_result(csv_path, output_path, title, warmup):
    (
        time,
        reference,
        actual,
        reference_velocity,
        reference_acceleration,
        reference_yaw,
        actual_yaw,
    ) = load_trial_data(csv_path, warmup)
    error = actual - reference
    error_norm = np.linalg.norm(error, axis=1)
    position_rmse = np.sqrt(np.mean(error_norm**2))
    maximum_error = error_norm.max()

    reference_speed = np.linalg.norm(reference_velocity, axis=1)
    reference_acceleration_norm = np.linalg.norm(reference_acceleration, axis=1)
    max_acceleration_index = int(np.argmax(reference_acceleration_norm))
    max_acceleration_time = time[max_acceleration_index]

    figure, axes = plt.subplots(5, 2, figsize=(14.0, 15.5))
    for index, (axis_name, color) in enumerate(zip(AXES, COLORS)):
        tracking_axis = axes[index, 0]
        tracking_axis.plot(
            time,
            reference[:, index],
            color="black",
            linewidth=1.35,
            linestyle="--",
            label="Reference",
        )
        tracking_axis.plot(
            time,
            actual[:, index],
            color=color,
            linewidth=1.05,
            label="Actual",
        )
        tracking_axis.set_ylabel("{} position [m]".format(axis_name.upper()))
        tracking_axis.grid(True, alpha=0.28)
        tracking_axis.axvline(
            max_acceleration_time, color="#9467BD", linewidth=0.8, alpha=0.55
        )

        error_axis = axes[index, 1]
        error_axis.axhline(0.0, color="black", linewidth=0.75, alpha=0.55)
        error_axis.plot(time, error[:, index], color=color, linewidth=1.0)
        axis_rmse = np.sqrt(np.mean(error[:, index] ** 2))
        axis_maximum = np.abs(error[:, index]).max()
        error_axis.set_ylabel("{} error [m]".format(axis_name.upper()))
        error_axis.set_title(
            "RMSE {:.3f} m, max |error| {:.3f} m".format(axis_rmse, axis_maximum),
            fontsize=10,
        )
        error_axis.grid(True, alpha=0.28)
        error_axis.axvline(
            max_acceleration_time, color="#9467BD", linewidth=0.8, alpha=0.55
        )

    velocity_axis = axes[3, 0]
    for index, (axis_name, color) in enumerate(zip(AXES, COLORS)):
        velocity_axis.plot(
            time,
            reference_velocity[:, index],
            color=color,
            linewidth=0.95,
            label=r"$v_{}$".format(axis_name),
        )
    velocity_axis.plot(
        time, reference_speed, color="black", linewidth=1.25, label=r"$\|v\|$"
    )
    velocity_axis.axvline(
        max_acceleration_time, color="#9467BD", linewidth=0.8, alpha=0.55
    )
    velocity_axis.set_title("Reference velocity")
    velocity_axis.set_ylabel("Velocity [m/s]")
    velocity_axis.set_xlabel("Time [s]")
    velocity_axis.grid(True, alpha=0.28)
    velocity_axis.legend(ncol=4, fontsize=9, loc="best")

    acceleration_axis = axes[3, 1]
    for index, (axis_name, color) in enumerate(zip(AXES, COLORS)):
        acceleration_axis.plot(
            time,
            reference_acceleration[:, index],
            color=color,
            linewidth=0.95,
            label=r"$a_{}$".format(axis_name),
        )
    acceleration_axis.plot(
        time,
        reference_acceleration_norm,
        color="black",
        linewidth=1.25,
        label=r"$\|a\|$",
    )
    acceleration_axis.axvline(
        max_acceleration_time,
        color="#9467BD",
        linewidth=0.9,
        linestyle="--",
        label="max acceleration",
    )
    acceleration_axis.scatter(
        [max_acceleration_time],
        [reference_acceleration_norm[max_acceleration_index]],
        color="#9467BD",
        s=20,
        zorder=4,
    )
    acceleration_axis.set_title(
        "Reference acceleration: max {:.3f} m/s² at {:.3f} m/s".format(
            reference_acceleration_norm[max_acceleration_index],
            reference_speed[max_acceleration_index],
        )
    )
    acceleration_axis.set_ylabel("Acceleration [m/s²]")
    acceleration_axis.set_xlabel("Time [s]")
    acceleration_axis.grid(True, alpha=0.28)
    acceleration_axis.legend(ncol=3, fontsize=9, loc="best")

    xy_axis = axes[4, 0]
    xy_axis.plot(
        reference[:, 0],
        reference[:, 1],
        color="black",
        linewidth=1.35,
        linestyle="--",
        label="Reference",
    )
    xy_axis.plot(
        actual[:, 0], actual[:, 1], color=COLORS[0], linewidth=1.0, label="Actual"
    )
    arrow_indices = np.unique(
        np.linspace(0, time.size - 1, min(12, time.size), dtype=int)
    )
    xy_extent = np.ptp(
        np.vstack((reference[:, :2], actual[:, :2])), axis=0
    ).max()
    arrow_length = 0.045 * max(xy_extent, 1.0)
    xy_axis.quiver(
        reference[arrow_indices, 0],
        reference[arrow_indices, 1],
        arrow_length * np.cos(reference_yaw[arrow_indices]),
        arrow_length * np.sin(reference_yaw[arrow_indices]),
        angles="xy",
        scale_units="xy",
        scale=1.0,
        pivot="mid",
        width=0.004,
        color="black",
        alpha=0.75,
    )
    xy_axis.quiver(
        actual[arrow_indices, 0],
        actual[arrow_indices, 1],
        arrow_length * np.cos(actual_yaw[arrow_indices]),
        arrow_length * np.sin(actual_yaw[arrow_indices]),
        angles="xy",
        scale_units="xy",
        scale=1.0,
        pivot="mid",
        width=0.004,
        color=COLORS[0],
        alpha=0.75,
    )
    xy_axis.scatter(reference[0, 0], reference[0, 1], color="#009E73", s=28, label="Start")
    xy_axis.scatter(reference[-1, 0], reference[-1, 1], color="#D55E00", s=28, label="End")
    xy_axis.set_aspect("equal", adjustable="datalim")
    xy_axis.set_title("XY path contour")
    xy_axis.set_xlabel("X [m]")
    xy_axis.set_ylabel("Y [m]")
    xy_axis.grid(True, alpha=0.28)
    legend_handles, legend_labels = xy_axis.get_legend_handles_labels()
    legend_handles[2:2] = [
        Line2D(
            [],
            [],
            color="black",
            marker=r"$\rightarrow$",
            linestyle="None",
            markersize=12,
        ),
        Line2D(
            [],
            [],
            color=COLORS[0],
            marker=r"$\rightarrow$",
            linestyle="None",
            markersize=12,
        ),
    ]
    legend_labels[2:2] = ["Reference yaw", "Actual yaw"]
    xy_axis.legend(legend_handles, legend_labels, fontsize=9, loc="best")

    norm_axis = axes[4, 1]
    norm_axis.plot(time, error_norm, color="#D55E00", linewidth=1.0, label="3D position error")
    norm_axis.axvline(
        max_acceleration_time, color="#9467BD", linewidth=0.8, alpha=0.55
    )
    norm_axis.set_title("3D position error")
    norm_axis.set_xlabel("Time [s]")
    norm_axis.set_ylabel("3D position error [m]", color="#D55E00")
    norm_axis.tick_params(axis="y", labelcolor="#D55E00")
    norm_axis.grid(True, alpha=0.28)
    axes[0, 0].legend(loc="best")
    axes[0, 0].set_title("Reference and actual position")
    for row in range(3):
        axes[row, 0].set_xlim(time[0], time[-1])
        axes[row, 1].set_xlim(time[0], time[-1])
    figure.suptitle(
        "{}\n3D position RMSE {:.3f} m, maximum error {:.3f} m".format(
            title, position_rmse, maximum_error
        ),
        fontsize=14,
    )
    figure.tight_layout(rect=(0.0, 0.0, 1.0, 0.955))
    output_path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(str(output_path), dpi=180, bbox_inches="tight")
    plt.close(figure)
    return position_rmse, maximum_error


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--title", default="PX4 Iris SITL tracking")
    parser.add_argument("--warmup", type=float, default=0.5)
    args = parser.parse_args()

    rmse, maximum = plot_result(
        args.csv.resolve(), args.output.resolve(), args.title, args.warmup
    )
    print(
        "wrote {} (position RMSE {:.3f} m, maximum error {:.3f} m)".format(
            args.output, rmse, maximum
        )
    )


if __name__ == "__main__":
    main()
