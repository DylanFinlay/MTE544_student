import os
import argparse
import matplotlib.pyplot as plt
from utilities import FileReader


def save_and_show(filename):
    """Save plot to the root directory (and subfolder if applicable) before showing."""
    base = os.path.splitext(os.path.basename(filename))[0]
    subfolder = os.path.dirname(filename)
    parent = os.path.basename(subfolder) if subfolder else ""

    if parent and parent not in (".", "", "logs"):
        out_name = f"{parent}_{base}.png"
    else:
        out_name = f"{base}.png"

    plt.savefig(out_name, dpi=300, bbox_inches="tight")
    print(f"Saved plot to {out_name}")

    if subfolder and subfolder not in (".", ""):
        folder_out = os.path.join(subfolder, f"{base}.png")
        plt.savefig(folder_out, dpi=300, bbox_inches="tight")
        print(f"Saved copy to {folder_out}")

    plt.show()
    plt.close()


def plot_robot_pose(headers, values, filename):
    """Plot comprehensive 4-panel EKF state and sensor comparison dashboard."""
    t0 = values[0][-1]
    # Timestamps are in nanoseconds, convert to elapsed seconds
    time_s = [(val[-1] - t0) / 1e9 for val in values]

    col = {name: i for i, name in enumerate(headers)}

    fig, axes = plt.subplots(2, 2, figsize=(14, 9))
    fig.suptitle(f"EKF Localization Analysis - {filename}", fontsize=14)

    # 1. State Space Trajectory (x vs y)
    axes[0, 0].plot([v[col["kf_x"]] for v in values], [v[col["kf_y"]] for v in values], "b-", linewidth=2, label="EKF Path")
    if "odom_x" in col and "odom_y" in col:
        axes[0, 0].plot([v[col["odom_x"]] for v in values], [v[col["odom_y"]] for v in values], "k--", alpha=0.75, linewidth=1.5, label="Raw Odometry")
        axes[0, 0].set_title("Robot Trajectory: EKF vs Odometry")
    else:
        axes[0, 0].set_title("Robot Trajectory (State Space)")

    axes[0, 0].plot(values[0][col["kf_x"]], values[0][col["kf_y"]], "go", label="Start")
    axes[0, 0].plot(values[-1][col["kf_x"]], values[-1][col["kf_y"]], "rx", label="End")
    axes[0, 0].set_xlabel("x [m]")
    axes[0, 0].set_ylabel("y [m]")
    axes[0, 0].legend()
    axes[0, 0].grid(True)
    axes[0, 0].axis("equal")

    # 2. Longitudinal Acceleration: imu_ax vs kf_ax
    axes[0, 1].plot(time_s, [v[col["imu_ax"]] for v in values], "gray", alpha=0.6, label="Measured (imu_ax)")
    axes[0, 1].plot(time_s, [v[col["kf_ax"]] for v in values], "r-", label="Estimated (kf_ax)")
    axes[0, 1].set_title("Longitudinal Acceleration: Measured vs Estimated")
    axes[0, 1].set_xlabel("Time [s]")
    axes[0, 1].set_ylabel("Acceleration [m/s$^2$]")
    axes[0, 1].legend()
    axes[0, 1].grid(True)

    # 3. Lateral Acceleration: imu_ay vs kf_ay
    axes[1, 0].plot(time_s, [v[col["imu_ay"]] for v in values], "gray", alpha=0.6, label="Measured (imu_ay)")
    axes[1, 0].plot(time_s, [v[col["kf_ay"]] for v in values], "g-", label="Estimated (kf_ay)")
    axes[1, 0].set_title("Lateral Acceleration: Measured vs Estimated")
    axes[1, 0].set_xlabel("Time [s]")
    axes[1, 0].set_ylabel("Acceleration [m/s$^2$]")
    axes[1, 0].legend()
    axes[1, 0].grid(True)

    # 4. Velocities: kf_vx and kf_w
    axes[1, 1].plot(time_s, [v[col["kf_vx"]] for v in values], "b-", label="Linear Vel kf_vx [m/s]")
    axes[1, 1].plot(time_s, [v[col["kf_w"]] for v in values], "m--", label="Angular Vel kf_w [rad/s]")
    axes[1, 1].set_title("Estimated Velocities")
    axes[1, 1].set_xlabel("Time [s]")
    axes[1, 1].set_ylabel("Velocity")
    axes[1, 1].legend()
    axes[1, 1].grid(True)

    plt.tight_layout()
    save_and_show(filename)


def plot_controller_errors(headers, values, filename):
    """Plot controller tracking errors (linear.csv or angular.csv)."""
    t0 = values[0][-1]
    time_s = [(val[-1] - t0) / 1e9 for val in values]

    fig, axes = plt.subplots(2, 1, figsize=(12, 6))
    fig.suptitle(f"Controller Errors - {filename}", fontsize=14)

    # Error
    axes[0].plot(time_s, [v[0] for v in values], "b-", label=headers[0])
    axes[0].set_title(f"Tracking Error ({headers[0]})")
    axes[0].set_xlabel("Time [s]")
    axes[0].set_ylabel("Error")
    axes[0].legend()
    axes[0].grid(True)

    # Derivative and Integral
    for i in range(1, len(headers) - 1):
        axes[1].plot(time_s, [v[i] for v in values], label=headers[i])
    axes[1].set_title("Error Derivative & Integral")
    axes[1].set_xlabel("Time [s]")
    axes[1].set_ylabel("Value")
    axes[1].legend()
    axes[1].grid(True)

    plt.tight_layout()
    save_and_show(filename)


def plot_errors(filename):
    headers, values = FileReader(filename).read_file()
    if not values:
        print(f"No data found in {filename}")
        return

    # Check if file is robot pose/EKF log or controller error log
    if "kf_x" in headers and "imu_ax" in headers:
        plot_robot_pose(headers, values, filename)
    else:
        plot_controller_errors(headers, values, filename)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Plot robot pose or controller error data.")
    parser.add_argument("--files", nargs="+", required=True, help="List of CSV files to process")

    args = parser.parse_args()
    print("Plotting files:", args.files)

    for filename in args.files:
        plot_errors(filename)


