#!/usr/bin/env python3
"""Plot a simulated dead-reckoning versus EKF run.

Reads the benchmark trace and landmark table written by ekf_loc_benchmark
and writes a trajectory figure, an error-over-time figure, and a short GIF.
"""

import argparse
import csv
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.patches import Polygon


TRUTH_COLOR = "#1c1917"
DR_COLOR = "#c2410c"
EKF_COLOR = "#0f766e"
LANDMARK_COLOR = "#1e3a5f"
GUIDE_COLOR = "#64748b"


def read_rows(path):
    with path.open(newline="") as handle:
        return list(csv.DictReader(handle))


def as_float(rows, key):
    return np.array([float(row[key]) for row in rows], dtype=float)


def load_trace(path):
    rows = read_rows(path)
    if not rows:
        raise SystemExit(f"trace is empty: {path}")
    return {
        "t": as_float(rows, "t_s"),
        "x_true": as_float(rows, "x_true_m"),
        "y_true": as_float(rows, "y_true_m"),
        "theta_true": as_float(rows, "theta_true_rad"),
        "x_dr": as_float(rows, "x_dr_m"),
        "y_dr": as_float(rows, "y_dr_m"),
        "x_ekf": as_float(rows, "x_ekf_m"),
        "y_ekf": as_float(rows, "y_ekf_m"),
        "theta_ekf": as_float(rows, "theta_ekf_rad"),
        "pos_dr": as_float(rows, "pos_err_dr_m"),
        "pos_ekf": as_float(rows, "pos_err_ekf_m"),
        "head_dr": as_float(rows, "heading_err_dr_deg"),
        "head_ekf": as_float(rows, "heading_err_ekf_deg"),
        "nees_dr": as_float(rows, "nees_dr"),
        "nees_ekf": as_float(rows, "nees_ekf"),
    }


def load_landmarks(path):
    rows = read_rows(path)
    if not rows:
        raise SystemExit(f"landmark table is empty: {path}")
    return np.array([[float(row["x_m"]), float(row["y_m"])] for row in rows], dtype=float)


def apply_style():
    plt.rcParams.update(
        {
            "font.family": "DejaVu Sans",
            "axes.spines.top": False,
            "axes.spines.right": False,
            "axes.grid": True,
            "grid.alpha": 0.28,
            "grid.linewidth": 0.6,
            "axes.labelsize": 11,
            "axes.titlesize": 13,
            "legend.fontsize": 9,
            "legend.frameon": False,
            "figure.facecolor": "white",
            "axes.facecolor": "white",
            "savefig.facecolor": "white",
        }
    )


def robot_triangle(x, y, theta, length):
    nose = np.array([np.cos(theta), np.sin(theta)]) * length
    left = np.array([np.cos(theta + 2.4), np.sin(theta + 2.4)]) * length * 0.55
    right = np.array([np.cos(theta - 2.4), np.sin(theta - 2.4)]) * length * 0.55
    return np.array(
        [
            [x + nose[0], y + nose[1]],
            [x + left[0], y + left[1]],
            [x + right[0], y + right[1]],
        ]
    )


def mean_position_rmse(benchmark_path):
    rows = read_rows(benchmark_path)
    found = {}
    for row in rows:
        if row["seed"] == "mean":
            found[row["method"]] = row["rmse_position_m"]
    if "dead_reckoning" not in found or "ekf" not in found:
        raise SystemExit(f"benchmark CSV has no mean rows: {benchmark_path}")
    return found["dead_reckoning"], found["ekf"]


def plot_trajectory(trace, landmarks, path, dead_reckoning_rmse, ekf_rmse):
    fig, ax = plt.subplots(figsize=(7.2, 6.8))
    ax.plot(trace["x_true"], trace["y_true"], color=TRUTH_COLOR, lw=2.0, label="Ground truth", zorder=3)
    ax.plot(
        trace["x_dr"],
        trace["y_dr"],
        color=DR_COLOR,
        lw=1.6,
        ls="--",
        label="Dead reckoning",
        zorder=2,
    )
    ax.plot(trace["x_ekf"], trace["y_ekf"], color=EKF_COLOR, lw=2.0, label="EKF", zorder=4)
    ax.scatter(
        landmarks[:, 0],
        landmarks[:, 1],
        marker="^",
        s=46,
        color=LANDMARK_COLOR,
        label="Landmark",
        zorder=5,
    )
    ax.scatter(
        [trace["x_true"][0]],
        [trace["y_true"][0]],
        s=36,
        facecolors="white",
        edgecolors=TRUTH_COLOR,
        linewidths=1.4,
        zorder=6,
        label="Start",
    )
    ax.set_aspect("equal", adjustable="datalim")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title(
        "Simulated trajectory\n"
        f"mean position RMSE  dead reckoning {dead_reckoning_rmse} m,  EKF {ekf_rmse} m",
        fontsize=12,
    )
    ax.legend(loc="upper left")
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


def plot_errors(trace, path):
    fig, axes = plt.subplots(3, 1, figsize=(7.4, 7.6), sharex=True)
    time = trace["t"]
    axes[0].plot(time, trace["pos_dr"], color=DR_COLOR, lw=1.4, label="Dead reckoning")
    axes[0].plot(time, trace["pos_ekf"], color=EKF_COLOR, lw=1.6, label="EKF")
    axes[0].set_ylabel("Position error (m)")
    axes[0].set_title("Simulated error over time")
    axes[0].legend(loc="upper left")

    axes[1].plot(time, trace["head_dr"], color=DR_COLOR, lw=1.4, label="Dead reckoning")
    axes[1].plot(time, trace["head_ekf"], color=EKF_COLOR, lw=1.6, label="EKF")
    axes[1].set_ylabel("Heading error (deg)")

    axes[2].plot(time, trace["nees_dr"], color=DR_COLOR, lw=1.0, alpha=0.9, label="Dead reckoning")
    axes[2].plot(time, trace["nees_ekf"], color=EKF_COLOR, lw=1.4, label="EKF")
    axes[2].axhline(3.0, color=GUIDE_COLOR, lw=1.0, ls=":", label="State dimension (3)")
    axes[2].set_ylabel("NEES")
    axes[2].set_xlabel("Time (s)")
    axes[2].set_ylim(bottom=0.0)
    axes[2].legend(loc="upper right")
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


def plot_animation(trace, landmarks, path, frames):
    count = len(trace["t"])
    indexes = np.linspace(0, count - 1, frames, dtype=int)
    fig, ax = plt.subplots(figsize=(6.2, 5.8))
    ax.set_aspect("equal", adjustable="box")
    pad = 0.8
    xs = np.concatenate([trace["x_true"], trace["x_dr"], trace["x_ekf"], landmarks[:, 0]])
    ys = np.concatenate([trace["y_true"], trace["y_dr"], trace["y_ekf"], landmarks[:, 1]])
    ax.set_xlim(xs.min() - pad, xs.max() + pad)
    ax.set_ylim(ys.min() - pad, ys.max() + pad)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.scatter(landmarks[:, 0], landmarks[:, 1], marker="^", s=36, color=LANDMARK_COLOR, zorder=5)
    (line_true,) = ax.plot([], [], color=TRUTH_COLOR, lw=1.8, label="Ground truth")
    (line_dr,) = ax.plot([], [], color=DR_COLOR, lw=1.3, ls="--", label="Dead reckoning")
    (line_ekf,) = ax.plot([], [], color=EKF_COLOR, lw=1.8, label="EKF")
    body = Polygon(robot_triangle(0, 0, 0, 0.28), closed=True, facecolor=EKF_COLOR, edgecolor="none", zorder=6)
    ax.add_patch(body)
    title = ax.set_title("Simulated trajectory")
    ax.legend(loc="upper left")

    def update(frame_index):
        end = indexes[frame_index] + 1
        line_true.set_data(trace["x_true"][:end], trace["y_true"][:end])
        line_dr.set_data(trace["x_dr"][:end], trace["y_dr"][:end])
        line_ekf.set_data(trace["x_ekf"][:end], trace["y_ekf"][:end])
        body.set_xy(
            robot_triangle(
                trace["x_ekf"][end - 1],
                trace["y_ekf"][end - 1],
                trace["theta_ekf"][end - 1],
                0.28,
            )
        )
        title.set_text(f"Simulated trajectory  t = {trace['t'][end - 1]:.1f} s")
        return line_true, line_dr, line_ekf, body, title

    animation = FuncAnimation(fig, update, frames=len(indexes), blit=False)
    animation.save(path, writer=PillowWriter(fps=12))
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description="Plot simulated EKF localization results.")
    parser.add_argument("--benchmark", type=Path, default=Path("docs/benchmark.csv"))
    parser.add_argument("--trace", type=Path, default=Path("docs/sample_run.csv"))
    parser.add_argument("--landmarks", type=Path, default=Path("docs/landmarks.csv"))
    parser.add_argument("--trajectory", type=Path, default=Path("docs/trajectory.png"))
    parser.add_argument("--errors", type=Path, default=Path("docs/error_over_time.png"))
    parser.add_argument("--animation", type=Path, default=Path("docs/trajectory.gif"))
    parser.add_argument("--frames", type=int, default=80)
    args = parser.parse_args()

    apply_style()
    trace = load_trace(args.trace)
    landmarks = load_landmarks(args.landmarks)
    dead_reckoning_rmse, ekf_rmse = mean_position_rmse(args.benchmark)
    args.trajectory.parent.mkdir(parents=True, exist_ok=True)
    plot_trajectory(trace, landmarks, args.trajectory, dead_reckoning_rmse, ekf_rmse)
    plot_errors(trace, args.errors)
    plot_animation(trace, landmarks, args.animation, args.frames)
    print(f"wrote {args.trajectory}")
    print(f"wrote {args.errors}")
    print(f"wrote {args.animation}")


if __name__ == "__main__":
    main()
