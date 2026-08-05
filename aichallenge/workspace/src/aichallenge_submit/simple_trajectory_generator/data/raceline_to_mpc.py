#!/usr/bin/env python3
"""
Convert simple_trajectory_generator CSV (RViz format) → MPC TUM format.

Input columns  (raceline_mincurv_trailbraking_1.csv):
    x, y, z, qx, qy, qz, qw, speed

Output columns (multi_purpose_mpc_ros/env/final_ver3/*.csv):
    s_m, x_m, y_m, psi_rad, kappa_radpm, vx_mps, ax_mps2

Notes:
  - s_m         : cumulative arc-length computed from (x, y) differences
  - psi_rad     : heading angle derived from quaternion  psi = 2*atan2(qz, qw)
  - kappa_radpm : curvature estimated via finite differences (MPC recalculates)
  - ax_mps2     : longitudinal accel estimated via finite differences (MPC recalculates)
  - vx_mps      : speed taken directly from the 'speed' column

Usage:
    python3 raceline_to_mpc.py [input.csv] [output.csv]

    If no arguments are given the defaults below are used.
"""

import csv
import math
import sys
import os

# ── Paths ─────────────────────────────────────────────────────────────────────
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

DEFAULT_INPUT  = os.path.join(SCRIPT_DIR, "raceline_mincurv_edit.csv")
DEFAULT_OUTPUT = os.path.join(
    SCRIPT_DIR,
    "../../../aichallenge_submit/multi_purpose_mpc_ros/env/final_ver3/"
    "mincurv_edit.csv"
)

# ── Helpers ───────────────────────────────────────────────────────────────────

def quat_to_psi(qz: float, qw: float) -> float:
    """Convert a yaw-only quaternion (qx=qy=0) to heading angle [rad]."""
    return 2.0 * math.atan2(qz, qw)


def compute_arc_length(points):
    """Return cumulative arc-length distances starting at 0.0."""
    s = [0.0]
    for i in range(1, len(points)):
        dx = points[i][0] - points[i - 1][0]
        dy = points[i][1] - points[i - 1][1]
        s.append(s[-1] + math.hypot(dx, dy))
    return s


def compute_curvature(points, s_vals):
    """
    Estimate curvature kappa = dpsi/ds via finite differences of the
    geometry-derived heading angle.
    """
    n = len(points)
    # Geometry heading from consecutive points
    psi = []
    for i in range(n):
        if i < n - 1:
            dx = points[i + 1][0] - points[i][0]
            dy = points[i + 1][1] - points[i][1]
        else:
            dx = points[i][0] - points[i - 1][0]
            dy = points[i][1] - points[i - 1][1]
        psi.append(math.atan2(dy, dx))

    kappa = [0.0] * n
    for i in range(n):
        if i == 0:
            dpsi = psi[1] - psi[0]
            ds   = s_vals[1] - s_vals[0]
        elif i == n - 1:
            dpsi = psi[-1] - psi[-2]
            ds   = s_vals[-1] - s_vals[-2]
        else:
            dpsi = psi[i + 1] - psi[i - 1]
            ds   = s_vals[i + 1] - s_vals[i - 1]
        # Wrap to [-pi, pi]
        dpsi = (dpsi + math.pi) % (2 * math.pi) - math.pi
        kappa[i] = dpsi / ds if abs(ds) > 1e-9 else 0.0
    return kappa


def compute_acceleration(vx, s_vals):
    """Estimate ax = dvx/ds * vx via finite differences."""
    n = len(vx)
    ax = [0.0] * n
    for i in range(n):
        if i == 0:
            dvx = vx[1] - vx[0]
            ds  = s_vals[1] - s_vals[0]
        elif i == n - 1:
            dvx = vx[-1] - vx[-2]
            ds  = s_vals[-1] - s_vals[-2]
        else:
            dvx = vx[i + 1] - vx[i - 1]
            ds  = s_vals[i + 1] - s_vals[i - 1]
        dvx_ds = dvx / ds if abs(ds) > 1e-9 else 0.0
        ax[i] = dvx_ds * vx[i]
    return ax


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    input_csv  = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_INPUT
    output_csv = sys.argv[2] if len(sys.argv) > 2 else DEFAULT_OUTPUT

    if not os.path.exists(input_csv):
        print(f"ERROR: Input file not found:\n  {os.path.abspath(input_csv)}")
        sys.exit(1)

    # 1. Read input
    rows = []
    with open(input_csv, newline="", encoding="utf-8-sig") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append(row)

    if not rows:
        print("ERROR: Input file is empty.")
        sys.exit(1)

    # 2. Parse fields
    points  = [(float(r["x"]), float(r["y"])) for r in rows]
    speeds  = [float(r["speed"])              for r in rows]
    qz_list = [float(r["qz"])                 for r in rows]
    qw_list = [float(r["qw"])                 for r in rows]

    # 3. Derived quantities
    s_vals     = compute_arc_length(points)
    psi_vals   = [quat_to_psi(qz_list[i], qw_list[i]) for i in range(len(rows))]
    kappa_vals = compute_curvature(points, s_vals)
    ax_vals    = compute_acceleration(speeds, s_vals)

    # 4. Write output
    os.makedirs(os.path.dirname(os.path.abspath(output_csv)), exist_ok=True)
    with open(output_csv, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["s_m", "x_m", "y_m", "psi_rad", "kappa_radpm", "vx_mps", "ax_mps2"])
        for i in range(len(rows)):
            writer.writerow([
                f"{s_vals[i]:.7f}",
                f"{points[i][0]:.7f}",
                f"{points[i][1]:.7f}",
                f"{psi_vals[i]:.7f}",
                f"{kappa_vals[i]:.7f}",
                f"{speeds[i]:.7f}",
                f"{ax_vals[i]:.7f}",
            ])

    print(f"Converted {len(rows)} waypoints.")
    print(f"Input  : {os.path.abspath(input_csv)}")
    print(f"Output : {os.path.abspath(output_csv)}")


if __name__ == "__main__":
    main()
