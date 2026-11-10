import os
import math
import argparse
import matplotlib.pyplot as plt
from utilities import FileReader


import os
import math
import argparse
import matplotlib.pyplot as plt
from utilities import FileReader


# ==========================================
# Styling configuration for report plots
# ==========================================
plt.rcParams['font.size'] = 10
plt.rcParams['axes.titlesize'] = 11
plt.rcParams['axes.labelsize'] = 10
plt.rcParams['xtick.labelsize'] = 9
plt.rcParams['ytick.labelsize'] = 9
plt.rcParams['legend.fontsize'] = 9
plt.rcParams['lines.linewidth'] = 1.8
plt.rcParams['grid.linestyle'] = '--'
plt.rcParams['grid.alpha'] = 0.6


def load_run_data(folder):
    """Loads robot_pose.csv, linear.csv, and angular.csv from a given folder."""
    pose_file = os.path.join(folder, 'robot_pose.csv')
    lin_file = os.path.join(folder, 'linear.csv')
    ang_file = os.path.join(folder, 'angular.csv')

    data = {}
    if os.path.exists(pose_file):
        headers, values = FileReader(pose_file).read_file()
        if values and len(values) > 1:
            t0 = values[0][-1]
            data['pose'] = {
                't': [(v[-1] - t0) / 1e9 for v in values],
                'x': [v[0] for v in values],
                'y': [v[1] for v in values],
                'theta': [v[2] for v in values],
            }

    if os.path.exists(lin_file):
        headers, values = FileReader(lin_file).read_file()
        if values and len(values) > 1:
            t0 = values[0][-1]
            data['linear'] = {
                't': [(v[-1] - t0) / 1e9 for v in values],
                'e': [v[0] for v in values],
                'e_dot': [v[1] for v in values],
                'e_int': [v[2] for v in values] if len(values[0]) > 3 else [],
            }

    if os.path.exists(ang_file):
        headers, values = FileReader(ang_file).read_file()
        if values and len(values) > 1:
            t0 = values[0][-1]
            data['angular'] = {
                't': [(v[-1] - t0) / 1e9 for v in values],
                'e': [v[0] for v in values],
                'e_dot': [v[1] for v in values],
                'e_int': [v[2] for v in values] if len(values[0]) > 3 else [],
            }

    return data


# =========================================================================
# Section 5: Point Controller Unified Figure (2x2 Grid)
# Meets Rubric: {e-t, edot-t}, {x-t, y-t, th-t}, {x-y}, {e-edot} in 1 figure
# =========================================================================
def plot_point_controller_summary(folder, data, goal=(1.0, 1.0), save_path=None):
    if 'pose' not in data:
        print(f"Missing pose data in {folder}")
        return

    p = data['pose']
    run_name = os.path.basename(folder)

    fig, axes = plt.subplots(2, 2, figsize=(11.5, 8.5))
    fig.suptitle(f"Point Controller Performance: {run_name}", fontsize=13, fontweight='bold', y=0.98)

    # (a) {x - y} Trajectory Plot
    ax_xy = axes[0, 0]
    ax_xy.plot(p['x'], p['y'], color='#1f77b4', label='Robot Path', zorder=2)
    ax_xy.plot(p['x'][0], p['y'][0], marker='o', markersize=8, color='#2ca02c', linestyle='None',
               label=f"Start (0, 0)", zorder=3)
    ax_xy.plot(p['x'][-1], p['y'][-1], marker='s', markersize=8, color='#d62728', linestyle='None',
               label=f"End ({p['x'][-1]:.2f}, {p['y'][-1]:.2f})", zorder=3)
    ax_xy.plot(goal[0], goal[1], marker='*', markersize=11, color='#e377c2', linestyle='None',
               label=f"Goal ({goal[0]:.1f}, {goal[1]:.1f})", zorder=3)
    ax_xy.set_title("(a) Trajectory: {x - y}")
    ax_xy.set_xlabel("x [m]")
    ax_xy.set_ylabel("y [m]")
    ax_xy.axis('equal')
    ax_xy.grid(True)
    ax_xy.legend(loc='lower right', framealpha=0.85)

    # (b) {x-t, y-t, th-t} Pose States vs Time
    ax_pose = axes[0, 1]
    ax_pose.plot(p['t'], p['x'], color='#1f77b4', linestyle='-', label='x [m]')
    ax_pose.plot(p['t'], p['y'], color='#ff7f0e', linestyle='--', label='y [m]')
    ax_pose_th = ax_pose.twinx()
    ax_pose_th.plot(p['t'], p['theta'], color='#2ca02c', linestyle=':', label=r'$\theta$ [rad]')
    ax_pose.set_title(r"(b) Pose vs Time: {x-t, y-t, $\theta$-t}")
    ax_pose.set_xlabel("Time [s]")
    ax_pose.set_ylabel("Position [m]")
    ax_pose_th.set_ylabel(r"Orientation $\theta$ [rad]")
    ax_pose.grid(True)
    # Combine legends from both axes
    lines1, labels1 = ax_pose.get_legend_handles_labels()
    lines2, labels2 = ax_pose_th.get_legend_handles_labels()
    ax_pose.legend(lines1 + lines2, labels1 + labels2, loc='upper left', framealpha=0.85)

    # (c) {e-t, edot-t} Errors and Derivatives vs Time
    ax_err = axes[1, 0]
    ax_err_ang = ax_err.twinx()
    has_lin = 'linear' in data
    has_ang = 'angular' in data

    if has_lin:
        l = data['linear']
        ax_err.plot(l['t'], l['e'], color='#1f77b4', linestyle='-', label=r'$e_{lin}$ [m]')
        ax_err.plot(l['t'], l['e_dot'], color='#1f77b4', linestyle='--', alpha=0.75, label=r'$\dot{e}_{lin}$ [m/s]')
    if has_ang:
        a = data['angular']
        ax_err_ang.plot(a['t'], a['e'], color='#d62728', linestyle='-', label=r'$e_{ang}$ [rad]')
        ax_err_ang.plot(a['t'], a['e_dot'], color='#d62728', linestyle='--', alpha=0.75, label=r'$\dot{e}_{ang}$ [rad/s]')

    ax_err.axhline(0, color='gray', linestyle=':', alpha=0.5)
    ax_err.set_title(r"(c) Errors vs Time: {e-t, $\dot{e}$-t}")
    ax_err.set_xlabel("Time [s]")
    ax_err.set_ylabel(r"Linear Error $e_{lin}$ [m], $\dot{e}_{lin}$ [m/s]")
    ax_err_ang.set_ylabel(r"Angular Error $e_{ang}$ [rad], $\dot{e}_{ang}$ [rad/s]")
    ax_err.grid(True)
    lines1, labels1 = ax_err.get_legend_handles_labels()
    lines2, labels2 = ax_err_ang.get_legend_handles_labels()
    ax_err.legend(lines1 + lines2, labels1 + labels2, loc='upper right', framealpha=0.85)

    # (d) {e-edot} State Space Phase Portrait
    ax_phase = axes[1, 1]
    if has_lin:
        l = data['linear']
        ax_phase.plot(l['e'], l['e_dot'], color='#1f77b4', linestyle='-', label='Linear Phase')
        ax_phase.plot(l['e'][0], l['e_dot'][0], marker='o', color='#2ca02c', linestyle='None',
                      label='Lin Start', markersize=6)
        ax_phase.plot(l['e'][-1], l['e_dot'][-1], marker='s', color='#1f77b4', linestyle='None',
                      label='Lin End', markersize=6)
    if has_ang:
        a = data['angular']
        ax_phase.plot(a['e'], a['e_dot'], color='#d62728', linestyle='-', label='Angular Phase')
        ax_phase.plot(a['e'][0], a['e_dot'][0], marker='^', color='#ff7f0e', linestyle='None',
                      label='Ang Start', markersize=6)
        ax_phase.plot(a['e'][-1], a['e_dot'][-1], marker='d', color='#d62728', linestyle='None',
                      label='Ang End', markersize=6)

    ax_phase.axhline(0, color='gray', linestyle=':', alpha=0.5)
    ax_phase.axvline(0, color='gray', linestyle=':', alpha=0.5)
    ax_phase.set_title(r"(d) State Space: {e - $\dot{e}$}")
    ax_phase.set_xlabel("Error e")
    ax_phase.set_ylabel(r"Error Derivative $\dot{e}$")
    ax_phase.grid(True)
    ax_phase.legend(loc='lower left', framealpha=0.85)

    plt.tight_layout(rect=[0, 0, 1, 0.96])
    if save_path:
        plt.savefig(save_path, dpi=300)
        plt.close(fig)
    return fig


# =========================================================================
# Section 6: Trajectory Tracking Plots (Parabola & Sigmoid)
# Meets Rubric: Part 6 {x-y} for parabola and sigmoid (2 plots)
# =========================================================================
def get_reference_trajectory(traj_type, num_points=200):
    """Generates the theoretical reference trajectory according to the lab specs."""
    if traj_type == 'parabola':
        xs = [1.5 * i / (num_points - 1) for i in range(num_points)]
        ys = [x ** 2 for x in xs]
        name = r"Parabola ($y = x^2, x \in [0, 1.5]$)"
    elif traj_type == 'sigmoid':
        xs = [2.5 * i / (num_points - 1) for i in range(num_points)]
        ys = [2.0 / (1.0 + math.exp(-2.0 * x)) - 1.0 for x in xs]
        name = r"Sigmoid ($\sigma(x) = \frac{2}{1 + e^{-2x}} - 1, x \in [0, 2.5]$)"
    else:
        xs, ys, name = [], [], "Unknown"
    return xs, ys, name


def detect_trajectory_type(folder, data):
    name = folder.lower()
    if 'sigmoid' in name:
        return 'sigmoid'
    if 'pose' in data:
        max_x = max(data['pose']['x'])
        max_y = max(data['pose']['y'])
        if max_x > 2.0 and max_y < 1.3:
            return 'sigmoid'
    return 'parabola'


def plot_trajectory_xy_standalone(folder, data, traj_type, save_path=None):
    """Generates a standalone {x-y} plot of the actual trajectory vs theoretical reference."""
    if 'pose' not in data:
        return

    p = data['pose']
    ref_x, ref_y, ref_name = get_reference_trajectory(traj_type)
    run_name = os.path.basename(folder)

    fig, ax = plt.subplots(figsize=(6.5, 5.0))
    ax.plot(ref_x, ref_y, 'k--', linewidth=2.0, label='Desired Reference')
    ax.plot(p['x'], p['y'], color='#1f77b4', linewidth=2.0, label='Actual Robot Path')
    ax.plot(p['x'][0], p['y'][0], marker='o', markersize=8, color='#2ca02c', linestyle='None',
            label=f"Start ({p['x'][0]:.2f}, {p['y'][0]:.2f})")
    ax.plot(p['x'][-1], p['y'][-1], marker='s', markersize=8, color='#d62728', linestyle='None',
            label=f"End ({p['x'][-1]:.2f}, {p['y'][-1]:.2f})")

    ax.set_title(f"Trajectory Tracking {{x-y}}: {run_name}\n{ref_name}", fontsize=11, fontweight='bold')
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.axis('equal')
    ax.grid(True)
    ax.legend(loc='lower right', framealpha=0.85)

    plt.tight_layout()
    if save_path:
        plt.savefig(save_path, dpi=300)
        plt.close(fig)
    return fig


def plot_trajectory_summary(folder, data, traj_type, save_path=None):
    """Generates a 3-panel figure: {x-y} tracking, tracking errors vs time, and pose vs time."""
    if 'pose' not in data:
        return

    p = data['pose']
    ref_x, ref_y, ref_name = get_reference_trajectory(traj_type)
    run_name = os.path.basename(folder)

    fig, axes = plt.subplots(1, 3, figsize=(16, 4.6))
    fig.suptitle(f"Trajectory Performance: {run_name} ({ref_name})", fontsize=12, fontweight='bold', y=0.98)

    # 1. Trajectory {x - y}
    ax_xy = axes[0]
    ax_xy.plot(ref_x, ref_y, 'k--', linewidth=2.0, label='Reference')
    ax_xy.plot(p['x'], p['y'], color='#1f77b4', label='Actual Path')
    ax_xy.plot(p['x'][0], p['y'][0], marker='o', markersize=7, color='#2ca02c', linestyle='None', label='Start')
    ax_xy.plot(p['x'][-1], p['y'][-1], marker='s', markersize=7, color='#d62728', linestyle='None', label='End')
    ax_xy.set_title("(a) Trajectory Tracking {x-y}")
    ax_xy.set_xlabel("x [m]")
    ax_xy.set_ylabel("y [m]")
    ax_xy.axis('equal')
    ax_xy.grid(True)
    ax_xy.legend(loc='lower right', framealpha=0.85)

    # 2. Tracking Errors vs Time
    ax_err = axes[1]
    has_lin = 'linear' in data
    has_ang = 'angular' in data
    ax_err_ang = ax_err.twinx()
    if has_lin:
        l = data['linear']
        ax_err.plot(l['t'], l['e'], color='#1f77b4', label=r'$e_{lin}$ [m]')
    if has_ang:
        a = data['angular']
        ax_err_ang.plot(a['t'], a['e'], color='#d62728', linestyle='-', label=r'$e_{ang}$ [rad]')
    ax_err.axhline(0, color='gray', linestyle=':', alpha=0.5)
    ax_err.set_title("(b) Tracking Errors vs Time")
    ax_err.set_xlabel("Time [s]")
    ax_err.set_ylabel(r"Linear Error $e_{lin}$ [m]")
    ax_err_ang.set_ylabel(r"Angular Error $e_{ang}$ [rad]")
    ax_err.grid(True)
    lines1, labels1 = ax_err.get_legend_handles_labels()
    lines2, labels2 = ax_err_ang.get_legend_handles_labels()
    ax_err.legend(lines1 + lines2, labels1 + labels2, loc='upper right', framealpha=0.85)

    # 3. Pose States vs Time
    ax_pose = axes[2]
    ax_pose.plot(p['t'], p['x'], color='#1f77b4', linestyle='-', label='x [m]')
    ax_pose.plot(p['t'], p['y'], color='#ff7f0e', linestyle='--', label='y [m]')
    ax_pose_th = ax_pose.twinx()
    ax_pose_th.plot(p['t'], p['theta'], color='#2ca02c', linestyle=':', label=r'$\theta$ [rad]')
    ax_pose.set_title(r"(c) Pose States vs Time")
    ax_pose.set_xlabel("Time [s]")
    ax_pose.set_ylabel("Position [m]")
    ax_pose_th.set_ylabel(r"Orientation $\theta$ [rad]")
    ax_pose.grid(True)
    lines1, labels1 = ax_pose.get_legend_handles_labels()
    lines2, labels2 = ax_pose_th.get_legend_handles_labels()
    ax_pose.legend(lines1 + lines2, labels1 + labels2, loc='upper left', framealpha=0.85)

    plt.tight_layout(rect=[0, 0, 1, 0.94])
    if save_path:
        plt.savefig(save_path, dpi=300)
        plt.close(fig)
    return fig


def plot_tracking_errors_single_plot(folder, data, traj_type, save_path=None):
    """
    Plots both Linear Error (e_lin) and Angular Error (e_ang) vs Time on a single plot
    using dual y-axes (twinx) for clean report inclusion.
    """
    has_lin = 'linear' in data
    has_ang = 'angular' in data
    if not has_lin and not has_ang:
        return None

    run_name = os.path.basename(folder)
    display_name = "Parabola (PID #9)" if "pid_9" in run_name.lower() else (
        "Sigmoid (Improved PID)" if "sigmoid" in run_name.lower() else f"{traj_type.capitalize()} ({run_name})"
    )

    fig, ax_lin = plt.subplots(figsize=(6.5, 4.5))
    ax_ang = ax_lin.twinx()

    # Horizontal zero reference
    ax_lin.axhline(0, color='gray', linestyle=':', alpha=0.5, linewidth=1.2)

    if has_lin:
        l = data['linear']
        ax_lin.plot(l['t'], l['e'], color='#1f77b4', linestyle='-', linewidth=2.0, label=r'Linear Error $e_{lin}$ [m]')
        ax_lin.set_ylabel(r'Linear Error $e_{lin}$ [m]', color='#1f77b4', fontsize=10)
        ax_lin.tick_params(axis='y', labelcolor='#1f77b4')

    if has_ang:
        a = data['angular']
        ax_ang.plot(a['t'], a['e'], color='#d62728', linestyle='--', linewidth=2.0, label=r'Angular Error $e_{ang}$ [rad]')
        ax_ang.set_ylabel(r'Angular Error $e_{ang}$ [rad]', color='#d62728', fontsize=10)
        ax_ang.tick_params(axis='y', labelcolor='#d62728')

    ax_lin.set_xlabel('Time [s]', fontsize=10)
    ax_lin.set_title(f'Tracking Errors vs Time: {display_name}', fontsize=11, fontweight='bold')
    ax_lin.grid(True)

    # Combine legends from both axes
    lines1, labels1 = ax_lin.get_legend_handles_labels()
    lines2, labels2 = ax_ang.get_legend_handles_labels()
    ax_lin.legend(lines1 + lines2, labels1 + labels2, loc='upper right', framealpha=0.88)

    plt.tight_layout()
    if save_path:
        plt.savefig(save_path, dpi=300)
        plt.close(fig)
    return fig


def plot_bonus_errors_combined(pid9_folder, sigmoid_folder, save_path=None):
    """
    Generates a 1x2 figure showing the linear and angular tracking errors
    for Parabola (pid_9) and Sigmoid (pid_sigmoid_improved) side-by-side.
    """
    data_p = load_run_data(pid9_folder)
    data_s = load_run_data(sigmoid_folder)

    fig, (ax1_lin, ax2_lin) = plt.subplots(1, 2, figsize=(13.0, 4.8))
    fig.suptitle('Trajectory Tracking Errors: Linear and Angular Errors vs Time', fontsize=12, fontweight='bold', y=0.98)

    # 1. Parabola (pid_9)
    ax1_ang = ax1_lin.twinx()
    ax1_lin.axhline(0, color='gray', linestyle=':', alpha=0.5, linewidth=1.2)
    if 'linear' in data_p:
        l = data_p['linear']
        ax1_lin.plot(l['t'], l['e'], color='#1f77b4', linestyle='-', linewidth=2.0, label=r'Linear Error $e_{lin}$ [m]')
        ax1_lin.set_ylabel(r'Linear Error $e_{lin}$ [m]', color='#1f77b4')
        ax1_lin.tick_params(axis='y', labelcolor='#1f77b4')
    if 'angular' in data_p:
        a = data_p['angular']
        ax1_ang.plot(a['t'], a['e'], color='#d62728', linestyle='--', linewidth=2.0, label=r'Angular Error $e_{ang}$ [rad]')
        ax1_ang.set_ylabel(r'Angular Error $e_{ang}$ [rad]', color='#d62728')
        ax1_ang.tick_params(axis='y', labelcolor='#d62728')
    ax1_lin.set_xlabel('Time [s]')
    ax1_lin.set_title('(a) Parabola: $y = x^2$ (pid_9)', fontsize=11, fontweight='bold')
    ax1_lin.grid(True)
    lines1, labels1 = ax1_lin.get_legend_handles_labels()
    lines2, labels2 = ax1_ang.get_legend_handles_labels()
    ax1_lin.legend(lines1 + lines2, labels1 + labels2, loc='upper right', framealpha=0.88)

    # 2. Sigmoid (pid_sigmoid_improved)
    ax2_ang = ax2_lin.twinx()
    ax2_lin.axhline(0, color='gray', linestyle=':', alpha=0.5, linewidth=1.2)
    if 'linear' in data_s:
        l = data_s['linear']
        ax2_lin.plot(l['t'], l['e'], color='#1f77b4', linestyle='-', linewidth=2.0, label=r'Linear Error $e_{lin}$ [m]')
        ax2_lin.set_ylabel(r'Linear Error $e_{lin}$ [m]', color='#1f77b4')
        ax2_lin.tick_params(axis='y', labelcolor='#1f77b4')
    if 'angular' in data_s:
        a = data_s['angular']
        ax2_ang.plot(a['t'], a['e'], color='#d62728', linestyle='--', linewidth=2.0, label=r'Angular Error $e_{ang}$ [rad]')
        ax2_ang.set_ylabel(r'Angular Error $e_{ang}$ [rad]', color='#d62728')
        ax2_ang.tick_params(axis='y', labelcolor='#d62728')
    ax2_lin.set_xlabel('Time [s]')
    ax2_lin.set_title(r'(b) Sigmoid: $\sigma(x)$ (pid_sigmoid_improved)', fontsize=11, fontweight='bold')
    ax2_lin.grid(True)
    lines1, labels1 = ax2_lin.get_legend_handles_labels()
    lines2, labels2 = ax2_ang.get_legend_handles_labels()
    ax2_lin.legend(lines1 + lines2, labels1 + labels2, loc='upper right', framealpha=0.88)

    plt.tight_layout(rect=[0, 0, 1, 0.95])
    if save_path:
        plt.savefig(save_path, dpi=300)
        plt.close(fig)
    return fig


# =========================================================================
# Section 2 Deliverable: Stability Analysis Numerical Metrics Table
# Agility (Rise Time), Accuracy (Steady-state error), Overshoot, Oscillations
# =========================================================================
def calculate_stability_metrics(data, goal=(1.0, 1.0)):
    if 'pose' not in data:
        return None

    p = data['pose']
    t = p['t']
    x = p['x']
    y = p['y']
    theta = p['theta']

    # Euclidean distance to goal point
    dists = [math.sqrt((xi - goal[0]) ** 2 + (yi - goal[1]) ** 2) for xi, yi in zip(x, y)]
    init_dist = dists[0]
    final_dist = dists[-1]

    # Rise time (tr): time to reach 90% reduction of initial distance
    rise_time = None
    for ti, di in zip(t, dists):
        if di <= 0.1 * init_dist:
            rise_time = ti
            break
    if rise_time is None:
        rise_time = t[-1]

    # Settling time (ts): last time distance exceeds 0.05m tolerance
    settling_time = None
    for ti, di in reversed(list(zip(t, dists))):
        if di > 0.05:
            settling_time = ti
            break
    if settling_time is None:
        settling_time = t[0]

    # Overshoot along line of motion
    # Projection coordinate along goal direction: s = (x + y) / sqrt(2)
    s = [(xi + yi) / math.sqrt(2) for xi, yi in zip(x, y)]
    goal_s = math.sqrt(goal[0]**2 + goal[1]**2)
    max_s = max(s)
    overshoot_dist = max(0.0, max_s - goal_s)
    overshoot_pct = (overshoot_dist / goal_s) * 100.0

    # Number of oscillations in linear error derivative (zero crossings)
    oscillations = 0
    if 'linear' in data:
        e_dots = data['linear']['e_dot']
        for i in range(1, len(e_dots)):
            if (e_dots[i] >= 0 and e_dots[i - 1] < 0) or (e_dots[i] < 0 and e_dots[i - 1] >= 0):
                oscillations += 1

    return {
        'total_time_s': round(t[-1], 2),
        'rise_time_s': round(rise_time, 2),
        'settling_time_s': round(settling_time, 2),
        'overshoot_pct': round(overshoot_pct, 1),
        'final_pos_err_m': round(final_dist, 3),
        'final_theta_rad': round(theta[-1], 3),
        'oscillations': oscillations
    }


def generate_stability_analysis_report(section5_dir, output_csv, output_md):
    """Generates the numerical comparison table required for Section 2."""
    rows = []
    for item in sorted(os.listdir(section5_dir)):
        sub = os.path.join(section5_dir, item)
        if not os.path.isdir(sub):
            continue
        data = load_run_data(sub)
        metrics = calculate_stability_metrics(data)
        if metrics:
            metrics['run'] = item
            rows.append(metrics)

    if not rows:
        return

    # Write CSV
    with open(output_csv, 'w') as f:
        f.write("Controller Run,Rise Time tr [s],Settling Time ts [s],Overshoot [%],Steady-State Error ess [m],Final Theta [rad],Oscillations,Total Time [s]\n")
        for r in rows:
            f.write(f"{r['run']},{r['rise_time_s']},{r['settling_time_s']},{r['overshoot_pct']},{r['final_pos_err_m']},{r['final_theta_rad']},{r['oscillations']},{r['total_time_s']}\n")

    # Write Markdown table
    with open(output_md, 'w') as f:
        f.write("# Section 2: Stability and Performance Analysis Table (Point Controller)\n\n")
        f.write("| Controller Configuration | Agility: Rise Time $t_r$ [s] | Agility: Settling Time $t_s$ [s] | Overshoot [% OS] | Accuracy: $e_{ss}$ [m] | Final $\\theta$ [rad] | Oscillations | Total Time [s] |\n")
        f.write("| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: |\n")
        for r in rows:
            f.write(f"| **{r['run']}** | {r['rise_time_s']} | {r['settling_time_s']} | {r['overshoot_pct']}% | {r['final_pos_err_m']} | {r['final_theta_rad']} | {r['oscillations']} | {r['total_time_s']} |\n")

    print(f"\nGenerated Stability Analysis Table:\n - {output_csv}\n - {output_md}")


# =========================================================================
# Tuning Comparison Plots (Directly earns the 10 marks for Tuning Process)
# =========================================================================
def generate_tuning_comparison_plots(section5_dir, section6_dir):
    """Generates overlaid comparison figures to visually showcase controller tuning."""
    # 1. P-Controller tuning comparison
    p_runs = [
        ('pcontroller_baseline_point(1 1)', 'P Baseline', '#1f77b4', '-'),
        ('pcontroller_klp025kap09_point(1 1)GOOD', 'P Tuned (Good)', '#2ca02c', '-'),
        ('pcontroller_klp06kap04_point(1 1)BAD', 'P Aggressive (Bad/Overshoot)', '#d62728', '--')
    ]

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))
    fig.suptitle("P-Controller Gain Tuning Progression (Section 5)", fontsize=13, fontweight='bold')

    for folder_name, label, color, style in p_runs:
        path = os.path.join(section5_dir, folder_name)
        data = load_run_data(path)
        if 'pose' in data:
            p = data['pose']
            ax1.plot(p['x'], p['y'], color=color, linestyle=style, label=label)
        if 'linear' in data:
            l = data['linear']
            ax2.plot(l['t'], l['e'], color=color, linestyle=style, label=label)

    ax1.plot(1.0, 1.0, marker='*', markersize=12, color='#e377c2', label='Goal (1, 1)', linestyle='None')
    ax1.set_title("(a) Trajectory Tracking {x-y}")
    ax1.set_xlabel("x [m]")
    ax1.set_ylabel("y [m]")
    ax1.axis('equal')
    ax1.grid(True)
    ax1.legend(loc='lower right')

    ax2.set_title("(b) Linear Error Decay {e-t}")
    ax2.set_xlabel("Time [s]")
    ax2.set_ylabel("Error e [m]")
    ax2.grid(True)
    ax2.legend(loc='upper right')

    plt.tight_layout()
    save_p_tuning = os.path.join(section5_dir, "tuning_p_controller_comparison.png")
    plt.savefig(save_p_tuning, dpi=300)
    plt.close(fig)
    print(f"Saved: {save_p_tuning}")

    # 2. P vs PID comparison
    p_vs_pid = [
        ('pcontroller_klp025kap09_point(1 1)GOOD', 'Best P-Controller', '#1f77b4', '-'),
        ('pidcontroller_klp02klv05kli04kap08kav06kai02_point(1 1)', 'Best PID-Controller', '#2ca02c', '-'),
        ('pidcontroller_baseline_point(1 1)', 'PID Baseline', '#ff7f0e', '--')
    ]

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))
    fig.suptitle("P vs. PID Controller Performance Comparison (Point Goal)", fontsize=13, fontweight='bold')

    for folder_name, label, color, style in p_vs_pid:
        path = os.path.join(section5_dir, folder_name)
        data = load_run_data(path)
        if 'pose' in data:
            p = data['pose']
            ax1.plot(p['x'], p['y'], color=color, linestyle=style, label=label)
        if 'linear' in data:
            l = data['linear']
            ax2.plot(l['t'], l['e'], color=color, linestyle=style, label=label)

    ax1.plot(1.0, 1.0, marker='*', markersize=12, color='#e377c2', label='Goal (1, 1)', linestyle='None')
    ax1.set_title("(a) Trajectory Tracking {x-y}")
    ax1.set_xlabel("x [m]")
    ax1.set_ylabel("y [m]")
    ax1.axis('equal')
    ax1.grid(True)
    ax1.legend(loc='lower right')

    ax2.set_title("(b) Linear Error Decay {e-t}")
    ax2.set_xlabel("Time [s]")
    ax2.set_ylabel("Error e [m]")
    ax2.grid(True)
    ax2.legend(loc='upper right')

    plt.tight_layout()
    save_comparison = os.path.join(section5_dir, "comparison_p_vs_pid.png")
    plt.savefig(save_comparison, dpi=300)
    plt.close(fig)
    print(f"Saved: {save_comparison}")

    # 3. Sigmoid tuning comparison in Section 6
    sig_runs = [
        ('pid_sigmoid_baseline', 'Sigmoid Baseline', '#ff7f0e', '--'),
        ('pid_sigmoid_improved', 'Sigmoid Improved', '#1f77b4', '-')
    ]
    ref_x, ref_y, _ = get_reference_trajectory('sigmoid')

    fig, ax = plt.subplots(figsize=(7, 5))
    ax.plot(ref_x, ref_y, 'k--', linewidth=2.0, label='Desired Reference')
    for folder_name, label, color, style in sig_runs:
        path = os.path.join(section6_dir, folder_name)
        data = load_run_data(path)
        if 'pose' in data:
            p = data['pose']
            ax.plot(p['x'], p['y'], color=color, linestyle=style, label=label)
    ax.set_title("Sigmoid Trajectory Tracking Comparison (Baseline vs Improved)", fontsize=11, fontweight='bold')
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.axis('equal')
    ax.grid(True)
    ax.legend(loc='lower right')
    plt.tight_layout()
    save_sig = os.path.join(section6_dir, "comparison_sigmoid_tuning.png")
    plt.savefig(save_sig, dpi=300)
    plt.close(fig)
    print(f"Saved: {save_sig}")

    # 4. Parabola tuning comparison in Section 6
    para_runs = [
        ('pid_baseline', 'PID Baseline', '#ff7f0e', '--'),
        ('pid_4', 'PID #4 (High Hist/Slow)', '#2ca02c', ':'),
        ('Pid_7', 'PID #7 (Tuned/Fast)', '#1f77b4', '-')
    ]
    p_ref_x, p_ref_y, _ = get_reference_trajectory('parabola')

    fig, ax = plt.subplots(figsize=(7, 5))
    ax.plot(p_ref_x, p_ref_y, 'k--', linewidth=2.0, label='Desired Reference ($y = x^2$)')
    for folder_name, label, color, style in para_runs:
        path = os.path.join(section6_dir, folder_name)
        data = load_run_data(path)
        if 'pose' in data:
            p = data['pose']
            ax.plot(p['x'], p['y'], color=color, linestyle=style, label=label)
    ax.set_title(r"Parabola Trajectory Tracking Comparison ($y = x^2$)", fontsize=11, fontweight='bold')
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.axis('equal')
    ax.grid(True)
    ax.legend(loc='lower right')
    plt.tight_layout()
    save_para = os.path.join(section6_dir, "comparison_parabola_tuning.png")
    plt.savefig(save_para, dpi=300)
    plt.close(fig)
    print(f"Saved: {save_para}")


# =========================================================================
# Main batch processing routine
# =========================================================================
def process_all_subfolders(base_dirs):
    for base_dir in base_dirs:
        if not os.path.exists(base_dir):
            print(f"Directory not found: {base_dir}")
            continue

        is_section5 = "section 5" in base_dir.lower()
        is_section6 = "section 6" in base_dir.lower()

        print(f"\n==========================================")
        print(f"Processing Base Directory: {base_dir}")
        print(f"==========================================")

        for item in sorted(os.listdir(base_dir)):
            subfolder = os.path.join(base_dir, item)
            if not os.path.isdir(subfolder):
                continue

            data = load_run_data(subfolder)
            if 'pose' not in data:
                print(f"Skipping {item} (no robot_pose.csv data)")
                continue

            print(f"\n-> Run: {item}")

            if is_section5:
                # 1. Unified 4-panel report figure for Section 5
                out_summary = os.path.join(subfolder, "report_point_summary.png")
                plot_point_controller_summary(subfolder, data, goal=(1.0, 1.0), save_path=out_summary)
                print(f"   [Report Fig] Generated {out_summary}")

            elif is_section6:
                traj_type = detect_trajectory_type(subfolder, data)
                # 1. Standalone {x-y} report plot (exact requirement in Conclusions rubric!)
                out_xy = os.path.join(subfolder, "report_trajectory_xy.png")
                plot_trajectory_xy_standalone(subfolder, data, traj_type, save_path=out_xy)
                print(f"   [Report Fig] Generated {out_xy} ({traj_type})")

                # 2. Standalone tracking error plot (linear & angular on one single plot)
                out_err = os.path.join(subfolder, "report_errors.png")
                plot_tracking_errors_single_plot(subfolder, data, traj_type, save_path=out_err)
                print(f"   [Report Fig] Generated {out_err}")

                # 3. Comprehensive 3-panel figure
                out_summary = os.path.join(subfolder, "report_trajectory_summary.png")
                plot_trajectory_summary(subfolder, data, traj_type, save_path=out_summary)
                print(f"   [Report Fig] Generated {out_summary}")

    # Generate tuning comparisons and stability table
    sec5_path = "MTE544_Lab2_Results/Section 5"
    sec6_path = "MTE544_Lab2_Results/Section 6"
    if os.path.exists(sec5_path) and os.path.exists(sec6_path):
        print("\n==========================================")
        print("Generating Tuning Comparisons & Report Tables")
        print("==========================================")
        generate_tuning_comparison_plots(sec5_path, sec6_path)
        generate_stability_analysis_report(
            sec5_path,
            "MTE544_Lab2_Results/stability_analysis_table.csv",
            "MTE544_Lab2_Results/stability_analysis_table.md"
        )

        # Combined bonus error plot for pid_9 and pid_sigmoid_improved
        pid9_dir = os.path.join(sec6_path, "pid_9")
        sigmoid_dir = os.path.join(sec6_path, "pid_sigmoid_improved")
        if os.path.exists(pid9_dir) and os.path.exists(sigmoid_dir):
            combined_err_path = os.path.join(sec6_path, "report_bonus_errors_parabola_sigmoid.png")
            plot_bonus_errors_combined(pid9_dir, sigmoid_dir, save_path=combined_err_path)
            print(f"Saved: {combined_err_path}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Generate rubric-optimized plots for MTE544 Lab 2 Report.')
    parser.add_argument('--batch', action='store_true', default=True,
                        help='Batch process all runs in Section 5 and Section 6')
    parser.add_argument('--dirs', nargs='+', default=None,
                        help='Custom base directories to process')
    args = parser.parse_args()

    target_dirs = args.dirs if args.dirs else [
        "MTE544_Lab2_Results/Section 5",
        "MTE544_Lab2_Results/Section 6"
    ]
    process_all_subfolders(target_dirs)

