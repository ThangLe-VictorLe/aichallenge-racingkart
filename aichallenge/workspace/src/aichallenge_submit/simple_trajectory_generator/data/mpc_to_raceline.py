#!/usr/bin/env python3
"""
Convert traj_race_cl.csv (TUM format) → simple_trajectory_generator CSV format.

Input columns:  s_m, x_m, y_m, psi_rad, kappa_radpm, vx_mps, ax_mps2
Output columns: x, y, z, x_quat, y_quat, z_quat, w_quat, speed

The heading angle psi_rad is converted to a quaternion rotating around Z-axis:
  qz = sin(psi/2),  qw = cos(psi/2),  qx = qy = 0
"""

import csv
import math
import sys
import os

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

INPUT_CSV = os.path.join(
    SCRIPT_DIR,
    "../../../aichallenge_submit/multi_purpose_mpc_ros/env/final_ver3/mintime2.csv"
)
OUTPUT_CSV = os.path.join(SCRIPT_DIR, "raceline_mintime2.csv")

# Z height of the track (match existing raceline files)
Z_HEIGHT = 6.5

def psi_to_quaternion(psi_rad):
    """Yaw-only quaternion: rotation around Z axis."""
    half = psi_rad / 2.0
    return 0.0, 0.0, math.sin(half), math.cos(half)  # x, y, z, w

def main():
    if not os.path.exists(INPUT_CSV):
        abs_path = os.path.abspath(INPUT_CSV)
        print(f"ERROR: Input file not found:\n  {abs_path}")
        sys.exit(1)

    rows_written = 0
    with open(INPUT_CSV, newline="", encoding="utf-8-sig") as fin, \
         open(OUTPUT_CSV, "w", newline="") as fout:

        reader = csv.DictReader(fin)
        writer = csv.writer(fout)
        writer.writerow(["x", "y", "z", "x_quat", "y_quat", "z_quat", "w_quat", "speed"])

        for row in reader:
            x     = float(row["x_m"])
            y     = float(row["y_m"])
            psi   = float(row["psi_rad"])
            speed = float(row["vx_mps"])   # already in m/s

            qx, qy, qz, qw = psi_to_quaternion(psi)

            writer.writerow([
                f"{x:.8f}", f"{y:.8f}", f"{Z_HEIGHT}",
                f"{qx}", f"{qy}", f"{qz:.16f}", f"{qw:.16f}",
                f"{speed:.4f}"
            ])
            rows_written += 1

    print(f"Converted {rows_written} points.")
    print(f"Output: {os.path.abspath(OUTPUT_CSV)}")

if __name__ == "__main__":
    main()
