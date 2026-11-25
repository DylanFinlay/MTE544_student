import argparse
import csv
import json
import os
import re
import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_LOGS_DIR = os.path.join(SCRIPT_DIR, "logs")


def read_pgm_and_yaml(map_name="room", base_dir=SCRIPT_DIR):
    """Read a PGM/YAML map pair without requiring ROS 2."""
    if map_name.endswith(".yaml"):
        yaml_path = map_name
        pgm_path = map_name[:-5] + ".pgm"
    elif map_name.endswith(".pgm"):
        pgm_path = map_name
        yaml_path = map_name[:-4] + ".yaml"
    else:
        yaml_path = map_name + ".yaml"
        pgm_path = map_name + ".pgm"

    if not os.path.isabs(yaml_path):
        yaml_path = os.path.join(base_dir, yaml_path)
    if not os.path.isabs(pgm_path):
        pgm_path = os.path.join(base_dir, pgm_path)

    if not (os.path.exists(pgm_path) and os.path.exists(yaml_path)):
        return None

    # Parse PGM (P5 binary)
    with open(pgm_path, "rb") as f:
        header = f.readline().decode().strip()
        if header != "P5":
            raise ValueError(f"Unsupported PGM format in {pgm_path}: {header}")
        line = f.readline().decode().strip()
        while line.startswith("#"):
            line = f.readline().decode().strip()
        width, height = map(int, line.split())
        _max_value = int(f.readline().decode().strip())
        image_data = f.read()

    image_array = np.frombuffer(image_data, dtype=np.uint8).reshape((height, width))

    # Parse YAML metadata
    origin_x, origin_y, resolution = 0.0, 0.0, 0.05
    with open(yaml_path, "r") as f:
        for line in f:
            if "origin" in line:
                vals = re.findall(r"-?\d+(?:\.\d+)?", line)
                if len(vals) >= 2:
                    origin_x = float(vals[0])
                    origin_y = float(vals[1])
            elif "resolution" in line:
                vals = re.findall(r"\d+(?:\.\d+)?", line)
                if vals:
                    resolution = float(vals[0])

    extent = [origin_x, origin_x + width * resolution, origin_y, origin_y + height * resolution]
    return image_array, extent


def load_xy_csv(filepath):
    """Load x, y columns from a CSV file (handles trailing commas from utilities.Logger)."""
    if not filepath or not os.path.exists(filepath):
        return np.empty((0, 2))
    rows = []
    with open(filepath, "r", newline="") as f:
        reader = csv.reader(f)
        header = next(reader, None)
        if header is None:
            return np.empty((0, 2))
        for row in reader:
            vals = [v.strip() for v in row if v.strip() != ""]
            if len(vals) >= 2:
                try:
                    rows.append([float(vals[0]), float(vals[1])])
                except ValueError:
                    continue
    if not rows:
        return np.empty((0, 2))
    return np.array(rows)


def load_pid_csv(filepath):
    """Load [e, e_dot, e_int, stamp] rows from a Logger CSV."""
    if not filepath or not os.path.exists(filepath):
        return np.empty((0, 4))
    rows = []
    with open(filepath, "r", newline="") as f:
        reader = csv.reader(f)
        header = next(reader, None)
        if header is None:
            return np.empty((0, 4))
        for row in reader:
            vals = [v.strip() for v in row if v.strip() != ""]
            if len(vals) >= 4:
                try:
                    rows.append([float(vals[0]), float(vals[1]), float(vals[2]), float(vals[3])])
                except ValueError:
                    continue
    if not rows:
        return np.empty((0, 4))
    return np.array(rows)


def _draw_map_background(ax, map_data):
    if map_data is not None:
        image_array, extent = map_data
        ax.imshow(image_array, cmap="gray", origin="upper", extent=extent)
        ax.set_xlim(extent[0], extent[1])
        ax.set_ylim(extent[2], extent[3])
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.grid(True, linestyle="--", alpha=0.5)


def _load_run_metadata(run_dir):
    info_path = os.path.join(run_dir, "run_info.json")
    if os.path.exists(info_path):
        with open(info_path, "r") as f:
            return json.load(f)

    # Fallback if run_info.json does not exist
    folder_name = os.path.basename(os.path.normpath(run_dir))
    heuristic_name = folder_name.split("_")[0].capitalize() if "_" in folder_name else "Unknown"
    goals = []
    for fname in sorted(os.listdir(run_dir)):
        m = re.match(r"planned_path_goal(\d+)\.csv$", fname)
        if m:
            idx = int(m.group(1))
            goals.append({
                "goal_index": idx,
                "planned_path_file": fname,
                "actual_trajectory_file": f"actual_trajectory_goal{idx}.csv",
                "reached": False,
            })
    return {
        "heuristic_name": heuristic_name,
        "map_name": "room",
        "goals": goals,
    }


def generate_run_plots(run_dir, map_name=None, force=False):
    """Generate trajectory and PID plots for a single run directory. Returns list of created plot paths."""
    if not os.path.isdir(run_dir):
        return []

    info = _load_run_metadata(run_dir)
    heuristic_name = info.get("heuristic_name", "Unknown")
    map_name = map_name or info.get("map_name", "room")
    goals = info.get("goals", [])

    map_data = read_pgm_and_yaml(map_name)
    created_files = []

    # 1. Per-goal trajectory plots
    valid_goals = []
    for g in goals:
        idx = g.get("goal_index", 1)
        planned_path = load_xy_csv(os.path.join(run_dir, g.get("planned_path_file", "")))
        actual_traj = load_xy_csv(os.path.join(run_dir, g.get("actual_trajectory_file", "")))

        if len(planned_path) == 0 and len(actual_traj) == 0:
            continue
        valid_goals.append((idx, g, planned_path, actual_traj))

        out_path = os.path.join(run_dir, f"trajectory_goal{idx}.png")
        if not force and os.path.exists(out_path):
            continue

        fig, ax = plt.subplots(figsize=(8, 6), dpi=150)
        _draw_map_background(ax, map_data)

        if len(planned_path) > 0:
            ax.plot(
                planned_path[:, 0],
                planned_path[:, 1],
                linestyle="--",
                linewidth=2.0,
                color="tab:blue",
                label="Planned Path (A*)",
            )

        if len(actual_traj) > 0:
            ax.plot(
                actual_traj[:, 0],
                actual_traj[:, 1],
                linestyle="-",
                linewidth=2.0,
                color="tab:orange",
                label="Actual Robot Trajectory",
            )

        # Mark Start and Goal poses clearly (required by rubric)
        start_pt = g.get("start") or (planned_path[0].tolist() if len(planned_path) > 0 else actual_traj[0].tolist())
        goal_pt = g.get("end") or (planned_path[-1].tolist() if len(planned_path) > 0 else actual_traj[-1].tolist())

        ax.scatter(
            start_pt[0],
            start_pt[1],
            color="limegreen",
            edgecolors="black",
            s=90,
            zorder=5,
            label=f"Start ({start_pt[0]:.2f}, {start_pt[1]:.2f})",
        )
        ax.scatter(
            goal_pt[0],
            goal_pt[1],
            color="red",
            edgecolors="black",
            marker="*",
            s=160,
            zorder=6,
            label=f"Goal ({goal_pt[0]:.2f}, {goal_pt[1]:.2f})",
        )

        status = "Reached" if g.get("reached", False) else "In Progress / Stopped"
        ax.set_title(f"{heuristic_name} Heuristic - Goal {idx} ({status})")
        ax.legend(loc="best", fontsize=8)
        fig.tight_layout()
        fig.savefig(out_path)
        plt.close(fig)
        created_files.append(out_path)

    # 2. Combined overview plot for all goals in this run (or robotPose.csv fallback)
    overview_path = os.path.join(run_dir, "trajectory_overview.png")
    if force or not os.path.exists(overview_path):
        if valid_goals:
            fig, ax = plt.subplots(figsize=(8, 6), dpi=150)
            _draw_map_background(ax, map_data)
            colors = ["tab:blue", "tab:orange", "tab:purple", "tab:cyan", "tab:brown"]

            for i, (idx, g, planned_path, actual_traj) in enumerate(valid_goals):
                c = colors[i % len(colors)]
                if len(planned_path) > 0:
                    ax.plot(
                        planned_path[:, 0],
                        planned_path[:, 1],
                        linestyle="--",
                        linewidth=1.8,
                        color=c,
                        label=f"Goal {idx} Planned",
                    )
                if len(actual_traj) > 0:
                    ax.plot(
                        actual_traj[:, 0],
                        actual_traj[:, 1],
                        linestyle="-",
                        linewidth=2.0,
                        color=c,
                        alpha=0.85,
                        label=f"Goal {idx} Actual",
                    )
                start_pt = g.get("start") or (planned_path[0].tolist() if len(planned_path) > 0 else actual_traj[0].tolist())
                goal_pt = g.get("end") or (planned_path[-1].tolist() if len(planned_path) > 0 else actual_traj[-1].tolist())
                ax.scatter(start_pt[0], start_pt[1], color="limegreen", edgecolors="black", s=80, zorder=5,
                           label="Start" if i == 0 else None)
                ax.scatter(goal_pt[0], goal_pt[1], color="red", edgecolors="black", marker="*", s=150, zorder=6,
                           label="Goal" if i == 0 else None)

            ax.set_title(f"{heuristic_name} Heuristic - All Planned & Actual Trajectories")
            ax.legend(loc="best", fontsize=8)
            fig.tight_layout()
            fig.savefig(overview_path)
            plt.close(fig)
            created_files.append(overview_path)
        else:
            # Fallback: plot robotPose.csv if any poses were recorded
            pose_xy = load_xy_csv(os.path.join(run_dir, "robotPose.csv"))
            if len(pose_xy) > 0:
                fig, ax = plt.subplots(figsize=(8, 6), dpi=150)
                _draw_map_background(ax, map_data)
                ax.plot(pose_xy[:, 0], pose_xy[:, 1], color="tab:orange", linewidth=2.0, label="Robot Trajectory")
                ax.scatter(pose_xy[0, 0], pose_xy[0, 1], color="limegreen", edgecolors="black", s=80, zorder=5, label="Start")
                ax.scatter(pose_xy[-1, 0], pose_xy[-1, 1], color="red", edgecolors="black", marker="*", s=150, zorder=6, label="End")
                ax.set_title(f"{heuristic_name} Run - Recorded Robot Pose")
                ax.legend(loc="best", fontsize=8)
                fig.tight_layout()
                fig.savefig(overview_path)
                plt.close(fig)
                created_files.append(overview_path)

    # 3. PID error plot (linear & angular)
    pid_path = os.path.join(run_dir, "pid_errors.png")
    if force or not os.path.exists(pid_path):
        lin_data = load_pid_csv(os.path.join(run_dir, "linear.csv"))
        ang_data = load_pid_csv(os.path.join(run_dir, "angular.csv"))
        if len(lin_data) > 0 or len(ang_data) > 0:
            fig, axes = plt.subplots(2, 1, figsize=(8, 6), dpi=150, sharex=False)

            if len(lin_data) > 0:
                t_lin = (lin_data[:, 3] - lin_data[0, 3]) / 1e9
                axes[0].plot(t_lin, lin_data[:, 0], label="Linear Error (m)", color="tab:blue")
            axes[0].set_title(f"{heuristic_name} Heuristic - PID Tracking Errors")
            axes[0].set_ylabel("Linear Error (m)")
            axes[0].grid(True, linestyle="--", alpha=0.5)
            axes[0].legend(loc="best", fontsize=8)

            if len(ang_data) > 0:
                t_ang = (ang_data[:, 3] - ang_data[0, 3]) / 1e9
                axes[1].plot(t_ang, ang_data[:, 0], label="Angular Error (rad)", color="tab:red")
            axes[1].set_xlabel("Time (s)")
            axes[1].set_ylabel("Angular Error (rad)")
            axes[1].grid(True, linestyle="--", alpha=0.5)
            axes[1].legend(loc="best", fontsize=8)

            fig.tight_layout()
            fig.savefig(pid_path)
            plt.close(fig)
            created_files.append(pid_path)

    return created_files


def compare_runs(run_dirs, out_path, map_name="room"):
    """Overlay two or more runs (e.g., Euclidean vs Manhattan) on the same map in different colors."""
    map_data = read_pgm_and_yaml(map_name)
    fig, ax = plt.subplots(figsize=(9, 7), dpi=150)
    _draw_map_background(ax, map_data)

    palette = ["tab:blue", "tab:orange", "tab:purple", "tab:cyan"]
    start_labeled = False
    goal_labeled = False

    for i, rdir in enumerate(run_dirs):
        info = _load_run_metadata(rdir)
        label_prefix = info.get("heuristic_name", os.path.basename(os.path.normpath(rdir)))
        color = palette[i % len(palette)]

        for g in info.get("goals", []):
            idx = g.get("goal_index", 1)
            planned = load_xy_csv(os.path.join(rdir, g.get("planned_path_file", "")))
            actual = load_xy_csv(os.path.join(rdir, g.get("actual_trajectory_file", "")))

            if len(planned) > 0:
                ax.plot(
                    planned[:, 0],
                    planned[:, 1],
                    linestyle="--",
                    linewidth=1.8,
                    color=color,
                    label=f"{label_prefix} Goal {idx} (Planned)",
                )
            if len(actual) > 0:
                ax.plot(
                    actual[:, 0],
                    actual[:, 1],
                    linestyle="-",
                    linewidth=2.0,
                    color=color,
                    alpha=0.85,
                    label=f"{label_prefix} Goal {idx} (Actual)",
                )

            start_pt = g.get("start") or (planned[0].tolist() if len(planned) > 0 else None)
            goal_pt = g.get("end") or (planned[-1].tolist() if len(planned) > 0 else None)
            if start_pt is not None:
                ax.scatter(
                    start_pt[0],
                    start_pt[1],
                    color="limegreen",
                    edgecolors="black",
                    s=80,
                    zorder=5,
                    label="Start" if not start_labeled else None,
                )
                start_labeled = True
            if goal_pt is not None:
                ax.scatter(
                    goal_pt[0],
                    goal_pt[1],
                    color="red",
                    edgecolors="black",
                    marker="*",
                    s=150,
                    zorder=6,
                    label="Goal" if not goal_labeled else None,
                )
                goal_labeled = True

    ax.set_title("Heuristic Comparison: Planned & Actual Trajectories on Map")
    ax.legend(loc="best", fontsize=8)
    fig.tight_layout()
    fig.savefig(out_path)
    plt.close(fig)
    print(f"Saved comparison plot to: {out_path}")


def plot_all_missing(logs_dir=DEFAULT_LOGS_DIR, map_name=None, force=False):
    """Scan all subfolders inside logs/ and generate plots for any subfolder missing them."""
    if not os.path.isdir(logs_dir):
        print(f"Logs directory does not exist yet: {logs_dir}")
        return

    subfolders = sorted(
        os.path.join(logs_dir, d)
        for d in os.listdir(logs_dir)
        if os.path.isdir(os.path.join(logs_dir, d))
    )
    if not subfolders:
        print(f"No run subfolders found in {logs_dir}")
        return

    total_created = 0
    for folder in subfolders:
        created = generate_run_plots(folder, map_name=map_name, force=force)
        if created:
            total_created += len(created)
            print(f"[{os.path.basename(folder)}] Generated {len(created)} plot(s):")
            for p in created:
                print(f"  - {p}")
        else:
            print(f"[{os.path.basename(folder)}] Plots up to date (skipped).")

    print(f"Done. Generated {total_created} plot(s) across {len(subfolders)} run(s).")


def main():
    parser = argparse.ArgumentParser(description="Generate trajectory & PID plots for Lab 4 runs in logs/.")
    parser.add_argument("--logs-dir", default=DEFAULT_LOGS_DIR, help="Path to logs directory (default: ./logs)")
    parser.add_argument("--map", default=None, help="Map base name or path (default: from run_info.json or 'room')")
    parser.add_argument("--force", action="store_true", help="Regenerate plots even if they already exist")
    parser.add_argument(
        "--compare",
        nargs="+",
        metavar="RUN_DIR",
        help="Compare two or more run folders on a single map plot (e.g. Euclidean vs Manhattan)",
    )
    parser.add_argument("--out", default=os.path.join(DEFAULT_LOGS_DIR, "comparison.png"), help="Output path for --compare")
    args = parser.parse_args()

    if args.compare:
        compare_runs(args.compare, args.out, map_name=args.map or "room")
    else:
        plot_all_missing(logs_dir=args.logs_dir, map_name=args.map, force=args.force)


if __name__ == "__main__":
    main()
