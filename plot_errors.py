import os
import math
import argparse
import matplotlib.pyplot as plt
from utilities import FileReader


def plot_pose(filename, headers, values):
    if len(values) < 2:
        print(f"Not enough data in {filename}")
        return

    first_stamp = values[0][-1]
    time_list = [(val[-1] - first_stamp) / 1e9 for val in values]

    x = [val[0] for val in values]
    y = [val[1] for val in values]
    theta = [val[2] for val in values]

    # Compute linear and angular velocities from pose differences
    vel_times = []
    linear_vels = []
    angular_vels = []
    for i in range(1, len(values)):
        dt = time_list[i] - time_list[i - 1]
        if dt > 0:
            vel_times.append(time_list[i])
            v = math.sqrt((x[i] - x[i - 1]) ** 2 + (y[i] - y[i - 1]) ** 2) / dt
            linear_vels.append(v)

            dth = (theta[i] - theta[i - 1] + math.pi) % (2 * math.pi) - math.pi
            w = dth / dt
            angular_vels.append(w)

    fig, axes = plt.subplots(2, 2, figsize=(13, 9))
    fig.suptitle(f"Robot Odometry & Velocities ({filename})", fontsize=14)

    # 1. Trajectory (x - y)
    axes[0, 0].plot(x, y, 'b-', label='Trajectory')
    axes[0, 0].plot(x[0], y[0], 'go', markersize=8, label='Start')
    axes[0, 0].plot(x[-1], y[-1], 'ro', markersize=8, label='End')
    axes[0, 0].set_title("Robot Trajectory (x - y)")
    axes[0, 0].set_xlabel("x [m]")
    axes[0, 0].set_ylabel("y [m]")
    axes[0, 0].axis('equal')
    axes[0, 0].grid(True)
    axes[0, 0].legend()

    # 2. Pose vs Time
    axes[0, 1].plot(time_list, x, 'b-', label='x [m]')
    axes[0, 1].plot(time_list, y, 'g-', label='y [m]')
    axes[0, 1].plot(time_list, theta, 'r-', label='theta [rad]')
    axes[0, 1].set_title("Pose vs Time")
    axes[0, 1].set_xlabel("Time [s]")
    axes[0, 1].set_ylabel("Pose")
    axes[0, 1].grid(True)
    axes[0, 1].legend()

    # 3. Linear Velocity vs Time with saturation limits
    axes[1, 0].plot(vel_times, linear_vels, 'b-', label='Linear Velocity v')
    axes[1, 0].axhline(y=0.31, color='r', linestyle='--', label='TB4 Saturation (0.31 m/s)')
    axes[1, 0].axhline(y=0.22, color='orange', linestyle='--', label='TB3 Saturation (0.22 m/s)')
    axes[1, 0].set_title("Linear Velocity vs Time")
    axes[1, 0].set_xlabel("Time [s]")
    axes[1, 0].set_ylabel("v [m/s]")
    axes[1, 0].grid(True)
    axes[1, 0].legend()

    # 4. Angular Velocity vs Time with saturation limits
    axes[1, 1].plot(vel_times, angular_vels, 'b-', label='Angular Velocity w')
    axes[1, 1].axhline(y=1.9, color='r', linestyle='--', label='TB4 Saturation (±1.9 rad/s)')
    axes[1, 1].axhline(y=-1.9, color='r', linestyle='--')
    axes[1, 1].axhline(y=2.84, color='orange', linestyle='--', label='TB3 Saturation (±2.84 rad/s)')
    axes[1, 1].axhline(y=-2.84, color='orange', linestyle='--')
    axes[1, 1].set_title("Angular Velocity vs Time")
    axes[1, 1].set_xlabel("Time [s]")
    axes[1, 1].set_ylabel("w [rad/s]")
    axes[1, 1].grid(True)
    axes[1, 1].legend()

    plt.tight_layout()


def plot_error_file(filename, headers, values):
    if len(values) < 2:
        print(f"Not enough data in {filename}")
        return

    first_stamp = values[0][-1]
    time_list = [(val[-1] - first_stamp) / 1e9 for val in values]

    name = "Linear" if "linear" in filename.lower() else ("Angular" if "angular" in filename.lower() else "Error")

    fig, axes = plt.subplots(1, 2, figsize=(14, 5))
    fig.suptitle(f"{name} Error Plots ({filename})", fontsize=14)

    # 1. State space phase portrait: e vs e_dot
    e = [val[0] for val in values]
    e_dot = [val[1] for val in values]
    axes[0].plot(e, e_dot, 'b-', marker='.', markersize=3, label='e vs e_dot')
    axes[0].plot(e[0], e_dot[0], 'go', label='Start')
    axes[0].plot(e[-1], e_dot[-1], 'ro', label='End')
    axes[0].set_title(f"State Space: {name} (e vs e_dot)")
    axes[0].set_xlabel("Error e")
    axes[0].set_ylabel("Error Derivative e_dot")
    axes[0].grid(True)
    axes[0].legend()

    # 2. Individual states over time
    colors = ['b', 'g', 'm', 'c']
    for i in range(len(headers) - 1):
        color = colors[i % len(colors)]
        axes[1].plot(time_list, [val[i] for val in values], color=color, label=f"{headers[i]}")

    axes[1].set_title(f"{name} Error States vs Time")
    axes[1].set_xlabel("Time [s]")
    axes[1].set_ylabel("Values")
    axes[1].grid(True)
    axes[1].legend()

    plt.tight_layout()


def plot_errors(filename):
    headers, values = FileReader(filename).read_file()
    if not values:
        print(f"No data found in {filename}")
        return

    if "pose" in filename.lower() or "theta" in [h.lower() for h in headers]:
        plot_pose(filename, headers, values)
    else:
        plot_error_file(filename, headers, values)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Plot robot errors and trajectories.')
    parser.add_argument('--files', nargs='+', default=None, help='List of files to process')
    args = parser.parse_args()

    if args.files:
        filenames = args.files
    else:
        candidates = ["robot_pose.csv", "linear.csv", "angular.csv"]
        filenames = [f for f in candidates if os.path.exists(f)]
        if not filenames:
            filenames = candidates

    print("Plotting files:", filenames)
    for filename in filenames:
        if os.path.exists(filename):
            plot_errors(filename)
        else:
            print(f"File not found: {filename}")

    plt.show()
