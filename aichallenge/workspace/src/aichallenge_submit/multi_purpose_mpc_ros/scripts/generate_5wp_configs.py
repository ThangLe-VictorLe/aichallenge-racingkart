#!/usr/bin/env python3

"""
Universal 5-WP YAML Section Generator & Resampler Utility

This script converts coarse keypoint section YAML files (or directories of YAML files)
into fine-grained, uniform waypoint-sliced YAML files (e.g. 5-wp steps: 0, 5, 10, 15, ...).

Works universally for both `ref_vel.yaml` (ref_vel) and `steer_gain.yaml` (gain) or any custom section config.

python3 scripts/generate_5wp_configs.py
fine tune:
# Change section 12's gain to 1.8 live
ros2 param set /mpc_controller steer_gain.s12.gain 1.8

# Save the live changes back to steer_gain.yaml so they aren't lost
ros2 param set /mpc_controller steer_gain.save true
how often labels appear (e.g. every 5, 10, or 20 waypoints) live in ROS 2
ros2 param set /mpc_controller steer_gain.label_step 10
"""

import os
import sys
import argparse
import yaml
import numpy as np
from collections import OrderedDict
from typing import Dict, List, Tuple, Optional, Optional


def compute_max_wp_from_csv(csv_path: str, resolution: float = 0.6, smoothing_distance: int = 2) -> int:
    """
    Mirrors build_track_xy() from plot_track_waypoints.py.
    Returns the exact interpolated WP count produced by ReferencePath for this CSV.
    """
    data = np.genfromtxt(csv_path, delimiter=',', names=True)
    if 'x_m' in data.dtype.names and 'y_m' in data.dtype.names:
        raw_x = data['x_m'].tolist()
        raw_y = data['y_m'].tolist()
    elif 'x' in data.dtype.names and 'y' in data.dtype.names:
        raw_x = data['x'].tolist()
        raw_y = data['y'].tolist()
    else:
        raise ValueError("CSV has no x/y columns")

    raw_x = raw_x + raw_x[:smoothing_distance * 3]
    raw_y = raw_y + raw_y[:smoothing_distance * 3]

    n_wp = [max(1, int(np.sqrt((raw_x[i+1]-raw_x[i])**2 +
                               (raw_y[i+1]-raw_y[i])**2) / resolution))
            for i in range(len(raw_x)-1)]

    gp_x = raw_x[-1]
    wp_x = [np.linspace(raw_x[i], raw_x[i+1], n_wp[i], endpoint=False).tolist()
            for i in range(len(raw_x)-1)]
    wp_x = [w for seg in wp_x for w in seg] + [gp_x]

    num_wps = len(wp_x) - 2 * smoothing_distance
    return max(0, num_wps - 1)  # last valid wp_id

# ______________________________________________
# Edit here
# ______________________________________________
def resample_yaml_content(
    config_dict: Dict,
    step: int = 5,
    max_wp: int = 355,
    default_val: float = 1.0,
    interpolate: bool = False
) -> Dict:
    """
    Resamples a section config dictionary into uniform step intervals.
    Auto-detects root key (e.g., 'ref_vel_configulator' or 'steer_gain_configurator')
    and value field name (e.g., 'ref_vel' or 'gain').
    """
    if not config_dict or not isinstance(config_dict, dict):
        raise ValueError("Invalid YAML dictionary content")

    root_key = list(config_dict.keys())[0]
    sections = config_dict[root_key]

    if isinstance(sections, list):
        sec_dict = {}
        for item in sections:
            if isinstance(item, tuple) or isinstance(item, list):
                name, data = item[0], item[1]
                sec_dict[name] = data
        sections = sec_dict

    if not isinstance(sections, dict) or not sections:
        raise ValueError(f"Root key '{root_key}' has no section entries")

    # Detect value field name (the field that is NOT 'wp_id')
    first_section = list(sections.values())[0]
    val_field = None
    if isinstance(first_section, dict):
        for k in first_section.keys():
            if k != "wp_id":
                val_field = k
                break

    if val_field is None:
        val_field = "val"

    # Extract keypoints sorted by wp_id
    keypoints: List[Tuple[int, float]] = []
    for sec_name, sec_data in sections.items():
        if isinstance(sec_data, dict) and "wp_id" in sec_data and val_field in sec_data:
            keypoints.append((int(sec_data["wp_id"]), float(sec_data[val_field])))

    keypoints.sort(key=lambda x: x[0])

    def get_val_at_wp(wp: int) -> float:
        if not keypoints:
            return default_val

        num_keys = len(keypoints)
        if not interpolate:
            # Step-hold lookup (matches coarse section behavior)
            for i in range(num_keys):
                start_wp, val = keypoints[i]
                end_wp = keypoints[(i + 1) % num_keys][0]
                if start_wp <= end_wp:
                    if start_wp <= wp < end_wp:
                        return val
                else:
                    if wp >= start_wp or wp < end_wp:
                        return val
            return keypoints[-1][1] if wp >= keypoints[-1][0] else keypoints[0][1]
        else:
            # Linear interpolation
            for i in range(num_keys):
                start_wp, start_val = keypoints[i]
                end_wp, end_val = keypoints[(i + 1) % num_keys]
                if start_wp < end_wp:
                    if start_wp <= wp <= end_wp:
                        dist = end_wp - start_wp
                        ratio = (wp - start_wp) / dist if dist > 0 else 0.0
                        return start_val + ratio * (end_val - start_val)
                else:
                    if wp >= start_wp or wp <= end_wp:
                        total_dist = (max_wp + 1 - start_wp) + end_wp
                        curr_dist = (wp - start_wp) if wp >= start_wp else (max_wp + 1 - start_wp + wp)
                        ratio = min(1.0, max(0.0, curr_dist / total_dist)) if total_dist > 0 else 0.0
                        return start_val + ratio * (end_val - start_val)

            return keypoints[0][1]

    # Generate resampled sections
    new_sections = {}
    idx = 0
    for wp in range(0, max_wp + 1, step):
        sec_name = f"s{idx}"
        val = get_val_at_wp(wp)
        new_sections[sec_name] = {
            val_field: round(float(val), 3),
            "wp_id": int(wp)
        }
        idx += 1

    return {root_key: new_sections}


def process_file(
    input_path: str,
    output_path: str,
    step: int = 5,
    max_wp: int = 355,
    default_val: float = 1.0,
    interpolate: bool = False
):
    """Process a single YAML file."""
    with open(input_path, "r", encoding="utf-8") as f:
        try:
            config_dict = yaml.full_load(f)
        except Exception:
            f.seek(0)
            config_dict = yaml.unsafe_load(f)

    resampled = resample_yaml_content(
        config_dict=config_dict,
        step=step,
        max_wp=max_wp,
        default_val=default_val,
        interpolate=interpolate
    )

    os.makedirs(os.path.dirname(os.path.abspath(output_path)), exist_ok=True)
    with open(output_path, "w", encoding="utf-8") as f:
        yaml.dump(resampled, f, allow_unicode=True, default_flow_style=False, sort_keys=False)

    print(f"[SUCCESS] Resampled {input_path} -> {output_path} ({len(resampled[list(resampled.keys())[0]])} sections, step={step})")


def process_directory(
    input_dir: str,
    output_dir: str,
    step: int = 5,
    max_wp: int = 355,
    interpolate: bool = False
):
    """Process all YAML files in input_dir."""
    if not os.path.exists(input_dir):
        raise FileNotFoundError(f"Input directory not found: {input_dir}")

    os.makedirs(output_dir, exist_ok=True)
    for fname in os.listdir(input_dir):
        if fname.endswith(".yaml") or fname.endswith(".yml"):
            in_file = os.path.join(input_dir, fname)
            out_file = os.path.join(output_dir, fname)
            process_file(in_file, out_file, step=step, max_wp=max_wp, interpolate=interpolate)


def generate_default_configs(config_dir: str, step: int = 5, max_wp: int = None):
    # Auto-compute from CSV if not supplied
    if max_wp is None:
        pkg_dir = os.path.dirname(config_dir)
        try:
            with open(os.path.join(config_dir, "config.yaml"), 'r') as f:
                cfg = yaml.safe_load(f)
                csv_rel = cfg.get("reference_path", {}).get("csv_path", "")
            csv_path = os.path.join(pkg_dir, csv_rel) if csv_rel else None
            if not csv_path or not os.path.exists(csv_path):
                csv_path = os.path.join(pkg_dir, "env", "final_ver3", "mincurv_edit.csv")
            max_wp = compute_max_wp_from_csv(csv_path)
            print(f"[INFO] Auto-detected max_wp={max_wp} from {os.path.basename(csv_path)}")
        except Exception as e:
            max_wp = 560
            print(f"[WARN] Could not compute max_wp from CSV ({e}), falling back to {max_wp}")
    # Default ref_vel map
    def get_default_vel(wp_id):
        return 26.0

    # Default steer_gain map
    def get_default_gain(wp_id):
        return 1.6

    # ref_vel
    ref_vel_dict = {"ref_vel_configulator": {}}
    idx = 0
    for wp in range(0, max_wp + 1, step):
        ref_vel_dict["ref_vel_configulator"][f"s{idx}"] = {
            "ref_vel": float(get_default_vel(wp)),
            "wp_id": int(wp)
        }
        idx += 1

    ref_vel_path = os.path.join(config_dir, "ref_vel.yaml")
    with open(ref_vel_path, "w", encoding="utf-8") as f:
        yaml.dump(ref_vel_dict, f, allow_unicode=True, default_flow_style=False, sort_keys=False)
    print(f"[SUCCESS] Generated {ref_vel_path} ({idx} sections)")

    # steer_gain
    steer_gain_dict = {"steer_gain_configurator": {}}
    idx = 0
    for wp in range(0, max_wp + 1, step):
        steer_gain_dict["steer_gain_configurator"][f"s{idx}"] = {
            "gain": float(get_default_gain(wp)),
            "wp_id": int(wp)
        }
        idx += 1

    steer_gain_path = os.path.join(config_dir, "steer_gain.yaml")
    with open(steer_gain_path, "w", encoding="utf-8") as f:
        yaml.dump(steer_gain_dict, f, allow_unicode=True, default_flow_style=False, sort_keys=False)
    print(f"[SUCCESS] Generated {steer_gain_path} ({idx} sections)")


def main():
    parser = argparse.ArgumentParser(
        description="Universal 5-WP YAML Section Generator & Resampler Utility"
    )
    parser.add_argument("-i", "--input", type=str, help="Input YAML file or directory path")
    parser.add_argument("-o", "--output", type=str, help="Output YAML file or directory path")
    parser.add_argument("-s", "--step", type=int, default=5, help="Waypoint step size (default: 5)")
    parser.add_argument("-m", "--max-wp", type=int, default=None,
                        help="Maximum waypoint index (default: auto-computed from CSV)")
    parser.add_argument("--interpolate", action="store_true", help="Use linear interpolation when resampling")
    parser.add_argument("--default-val", type=float, default=1.0, help="Fallback default value if needed")

    args = parser.parse_args()

    pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    config_dir = os.path.join(pkg_dir, "config")

    if not args.input:
        print("No input specified. Generating standard default configs in pkg config dir...")
        generate_default_configs(config_dir, step=args.step, max_wp=args.max_wp)
        return

    input_path = os.path.abspath(args.input)

    if os.path.isdir(input_path):
        output_dir = os.path.abspath(args.output) if args.output else input_path
        process_directory(input_path, output_dir, step=args.step, max_wp=args.max_wp, interpolate=args.interpolate)
    else:
        output_path = os.path.abspath(args.output) if args.output else input_path
        process_file(input_path, output_path, step=args.step, max_wp=args.max_wp, default_val=args.default_val, interpolate=args.interpolate)

    # Auto-generate 2D track map image
    try:
        from plot_track_waypoints import plot_track_waypoints
        # Read the reference path dynamically from config.yaml
        config_yaml_path = os.path.join(config_dir, "config.yaml")
        csv_path = None
        try:
            with open(config_yaml_path, 'r') as f:
                cfg_data = __import__("yaml").safe_load(f)
                csv_path_rel = cfg_data.get("reference_path", {}).get("csv_path", "")
                if csv_path_rel:
                    csv_path = os.path.join(pkg_dir, csv_path_rel)
        except Exception:
            pass
        if not csv_path or not os.path.exists(csv_path):
            csv_path = os.path.join(pkg_dir, "env", "final_ver3", "mintime2.csv") # fallback
            
        steer_gain_path = os.path.join(config_dir, "steer_gain.yaml")
        ref_vel_path = os.path.join(config_dir, "ref_vel.yaml")
        output_png = os.path.join(config_dir, "track_waypoint_map.png")
        plot_track_waypoints(csv_path, steer_gain_path, ref_vel_path, output_png, step=args.step)
    except Exception as e:
        print(f"Note: Could not auto-generate track_waypoint_map.png: {e}")


if __name__ == "__main__":
    main()
