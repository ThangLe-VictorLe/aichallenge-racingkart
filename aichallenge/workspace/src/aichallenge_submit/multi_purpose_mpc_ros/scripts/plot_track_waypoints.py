#!/usr/bin/env python3

import os
import sys
import argparse
import yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

def load_yaml(filepath):
    if not os.path.exists(filepath):
        return {}
    with open(filepath, 'r') as f:
        return yaml.safe_load(f) or {}

def plot_track_waypoints(
    csv_path: str,
    steer_gain_path: str,
    ref_vel_path: str,
    output_png: str,
    step: int = 5
):
    # Load csv waypoints
    if not os.path.exists(csv_path):
        print(f"Error: CSV path {csv_path} not found.")
        return

    data = np.genfromtxt(csv_path, delimiter=',', names=True)
    if 'x_m' in data.dtype.names and 'y_m' in data.dtype.names:
        raw_x = data['x_m'].tolist()
        raw_y = data['y_m'].tolist()
    elif 'x' in data.dtype.names and 'y' in data.dtype.names:
        raw_x = data['x'].tolist()
        raw_y = data['y'].tolist()
    else:
        print("Error: Could not parse x,y from CSV")
        return

    # Emulate ReferencePath interpolation (resolution=0.6, smoothing_distance=2, circular=True)
    resolution = 0.6
    smoothing_distance = 2
    raw_x = raw_x + raw_x[:smoothing_distance * 3]
    raw_y = raw_y + raw_y[:smoothing_distance * 3]

    n_wp = [max(1, int(np.sqrt((raw_x[i + 1] - raw_x[i]) ** 2 +
                               (raw_y[i + 1] - raw_y[i]) ** 2) /
                       resolution)) for i in range(len(raw_x) - 1)]

    gp_x, gp_y = raw_x[-1], raw_y[-1]
    wp_x = [np.linspace(raw_x[i], raw_x[i+1], n_wp[i], endpoint=False).tolist() for i in range(len(raw_x)-1)]
    wp_x = [wp for segment in wp_x for wp in segment] + [gp_x]
    wp_y = [np.linspace(raw_y[i], raw_y[i+1], n_wp[i], endpoint=False).tolist() for i in range(len(raw_y)-1)]
    wp_y = [wp for segment in wp_y for wp in segment] + [gp_y]

    wp_xs = []
    wp_ys = []
    for wp_id in range(smoothing_distance, len(wp_x) - smoothing_distance):
        wp_xs.append(np.mean(wp_x[wp_id - smoothing_distance:wp_id + smoothing_distance + 1]))
        wp_ys.append(np.mean(wp_y[wp_id - smoothing_distance:wp_id + smoothing_distance + 1]))

    x = np.array(wp_xs)
    y = np.array(wp_ys)


    # Load steer gain and ref vel configs
    steer_cfg = load_yaml(steer_gain_path).get('steer_gain_configurator', {})
    ref_cfg = load_yaml(ref_vel_path).get('ref_vel_configulator', {})

    # Map wp_id -> gain and wp_id -> ref_vel
    gain_map = {}
    for k, v in steer_cfg.items():
        if isinstance(v, dict) and 'wp_id' in v and 'gain' in v:
            gain_map[int(v['wp_id'])] = float(v['gain'])

    vel_map = {}
    for k, v in ref_cfg.items():
        if isinstance(v, dict) and 'wp_id' in v and 'ref_vel' in v:
            vel_map[int(v['wp_id'])] = float(v['ref_vel'])

    fig, ax = plt.subplots(figsize=(16, 12), dpi=150)
    fig.patch.set_facecolor('#1e1e1e')
    ax.set_facecolor('#1e1e1e')

    # Plot full raceline path
    ax.plot(x, y, color='#00adb5', linewidth=2.5, alpha=0.7, label='Raceline Path')
    ax.scatter(x[0], y[0], color='#ff5722', s=120, zorder=5, label='Start/Finish Line (WP 0)')

    num_wps = len(x)

    # Plot labels every 'step' waypoints
    for i in range(0, num_wps, step):
        px, py = x[i], y[i]

        # Lookup nearest or exact gain & vel
        gain_str = f"{gain_map.get(i, ''):.2f}" if i in gain_map else ""
        vel_str = f"{vel_map.get(i, ''):.1f}" if i in vel_map else ""

        label_text = f"wp{i}"
        if vel_str:
            label_text += f"\n{vel_str}k"
        if gain_str:
            label_text += f"\ng:{gain_str}"

        # Draw point marker
        ax.scatter(px, py, color='#ffc107', s=35, zorder=4)

        # Offset text slightly to avoid overlapping line
        ax.annotate(
            label_text,
            (px, py),
            textcoords="offset points",
            xytext=(0, 10),
            ha='center',
            fontsize=7,
            color='#ffffff',
            bbox=dict(boxstyle="round,pad=0.2", fc="#222831", ec="#393e46", lw=1, alpha=0.8)
        )

    ax.set_title(f"Track Waypoints Map (Total: {num_wps} WPs, Label Step: {step})", color='#ffffff', fontsize=16, pad=15)
    ax.set_xlabel("X (m)", color='#eeeeee', fontsize=12)
    ax.set_ylabel("Y (m)", color='#eeeeee', fontsize=12)
    ax.tick_params(colors='#eeeeee')
    ax.axis('equal')
    ax.grid(True, color='#333333', linestyle='--', alpha=0.5)
    ax.legend(facecolor='#222831', edgecolor='#393e46', labelcolor='#eeeeee', loc='upper right')

    os.makedirs(os.path.dirname(os.path.abspath(output_png)), exist_ok=True)
    plt.savefig(output_png, bbox_inches='tight', facecolor=fig.get_facecolor())
    plt.close()
    print(f"[SUCCESS] Saved track waypoint map image to: {output_png}")

def main():
    parser = argparse.ArgumentParser(description="Plot 2D Track Waypoint Map with ref_vel and steer_gain")
    parser.add_argument("-c", "--csv", type=str, help="Path to mintime2.csv")
    parser.add_argument("-s", "--steer-gain", type=str, help="Path to steer_gain.yaml")
    parser.add_argument("-r", "--ref-vel", type=str, help="Path to ref_vel.yaml")
    parser.add_argument("-o", "--output", type=str, help="Output PNG path")
    parser.add_argument("--step", type=int, default=5, help="Waypoint label interval step")

    args = parser.parse_args()

    pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    csv_path = args.csv or os.path.join(pkg_dir, "env", "final_ver3", "mintime2.csv")
    steer_gain_path = args.steer_gain or os.path.join(pkg_dir, "config", "steer_gain.yaml")
    ref_vel_path = args.ref_vel or os.path.join(pkg_dir, "config", "ref_vel.yaml")
    output_png = args.output or os.path.join(pkg_dir, "config", "track_waypoint_map.png")

    plot_track_waypoints(csv_path, steer_gain_path, ref_vel_path, output_png, step=args.step)

if __name__ == '__main__':
    main()
