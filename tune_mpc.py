#!/usr/bin/env python3
"""
tune_mpc.py  –  Greedy Hill-Climbing MPC Parameter Optimizer
=============================================================
Iteratively tweaks config.yaml parameters by ±10%, runs make dev, collects
3 laps via the /awsim/status ROS2 topic, averages laps 2-3, and keeps
changes that improve lap time.

Strategy per parameter
----------------------
  1. Try +10% repeatedly while lap time improves.
  2. If +10% worsens immediately, try -10% from original instead.
  3. Keep going -10% while it improves.
  4. If neither direction helps, restore original and move to next param.

Usage
-----
  # From project root (/mnt/ZorinData/aichallenge-racingkart):
  pip install ruamel.yaml
  python3 tune_mpc.py                     # run 1 round
  python3 tune_mpc.py --rounds 3          # run 3 rounds
  python3 tune_mpc.py --dry-run           # simulate without calling make
  python3 tune_mpc.py --start-param N     # resume from a specific param
"""

import argparse
import json
import re
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

try:
    from ruamel.yaml import YAML
except ImportError:
    print("ERROR: ruamel.yaml is not installed.")
    print("       Run:  pip install ruamel.yaml")
    sys.exit(1)

# ─────────────────────────────────────────────────────────────────────────────
#  Paths & global settings
# ─────────────────────────────────────────────────────────────────────────────

CONFIG_PATH = Path(
    "aichallenge/workspace/src/aichallenge_submit"
    "/multi_purpose_mpc_ros/config/config.yaml"
)
RESULTS_FILE = Path("tune_results.json")

# Timing
SIM_STARTUP_SEC    = 22    # seconds to wait after `make dev` for containers
LAP_TIMEOUT_SEC    = 65    # per-lap ceiling (normal lap ~50 s)
LAPS_TO_COLLECT    = 3     # total laps to wait for; average laps 2..N
DOWN_GRACE_SEC     = 5     # time to wait after `make down`

# Tuning
TUNE_STEP       = 0.10   # 10 % perturbation
ROS_DOMAIN_ID   = 1      # Autoware uses domain 1 by default

# ─────────────────────────────────────────────────────────────────────────────
#  Parameter definitions
#  (yaml_dot_path, display_name, is_integer, min_value, max_value)
# ─────────────────────────────────────────────────────────────────────────────
#
#  List elements are addressed as <key>.<index>, e.g. "mpc.R.0"
#
PARAMS = [
    # Control horizon
    ("mpc.N",                            "N",                     True,  5,    60   ),
    # Input cost (velocity, steering angle)
    ("mpc.R.0",                          "R[0] v-cost",           False, 1e3,  1e9  ),
    ("mpc.R.1",                          "R[1] delta-cost",       False, 1e3,  1e9  ),
    # Steering limits
    ("mpc.delta_max_deg",                "delta_max_deg",         False, 5.0,  45.0 ),
    ("mpc.steer_rate_max",               "steer_rate_max",        False, 0.3,  6.0  ),
    # Controller rate
    ("mpc.control_rate",                 "control_rate",          False, 10.0, 100.0),
    # Vehicle model correction
    ("mpc.steering_tire_angle_gain_var", "steer_tire_gain",       False, 0.1,  5.0  ),
    # Low-pass filters (0.01-1.0)
    ("mpc.accel_low_pass_gain",          "accel_lp_gain",         False, 0.01, 1.0  ),
    ("mpc.steer_low_pass_gain",          "steer_lp_gain",         False, 0.01, 1.0  ),
    # Waypoint look-ahead offset
    ("mpc.wp_id_offset",                 "wp_id_offset",          True,  0,    15   ),
    # Lateral-acceleration cap for speed profile
    ("mpc.ay_max",                       "ay_max",                False, 3.0,  30.0 ),
    # State cost (lateral error, heading error, time)
    ("mpc.Q.0",                          "Q[0] e_y",              False, 1e4,  1e10 ),
    ("mpc.Q.1",                          "Q[1] e_psi",            False, 1e4,  1e10 ),
    ("mpc.Q.2",                          "Q[2] t",                False, 1e4,  1e10 ),
    # Terminal cost
    ("mpc.QN.0",                         "QN[0]",                 False, 1e4,  1e10 ),
    ("mpc.QN.1",                         "QN[1]",                 False, 1e4,  1e10 ),
    ("mpc.QN.2",                         "QN[2]",                 False, 1e4,  1e10 ),
]

# ─────────────────────────────────────────────────────────────────────────────
#  YAML helpers  (ruamel.yaml preserves all comments on round-trip)
# ─────────────────────────────────────────────────────────────────────────────

_yaml = YAML()
_yaml.preserve_quotes = True
_yaml.default_flow_style = None


def load_config():
    with open(CONFIG_PATH, "r") as fh:
        return _yaml.load(fh)


def save_config(cfg):
    with open(CONFIG_PATH, "w") as fh:
        _yaml.dump(cfg, fh)


def get_value(cfg, dot_path: str):
    """Retrieve a scalar by dot-path.  List indices are numeric segments."""
    node = cfg
    for seg in dot_path.split("."):
        node = node[int(seg)] if seg.isdigit() else node[seg]
    return node


def set_value(cfg, dot_path: str, value):
    """Set a scalar by dot-path."""
    parts = dot_path.split(".")
    node = cfg
    for seg in parts[:-1]:
        node = node[int(seg)] if seg.isdigit() else node[seg]
    last = parts[-1]
    if last.isdigit():
        node[int(last)] = value
    else:
        node[last] = value


# ─────────────────────────────────────────────────────────────────────────────
#  Shell helpers
# ─────────────────────────────────────────────────────────────────────────────

PROJECT_ROOT = Path(__file__).parent


def _run_make(target: str):
    """Run `make <target>` in the project root, inheriting stdio."""
    subprocess.run(["make", target], cwd=PROJECT_ROOT, check=False)


def sim_start(dry_run: bool):
    print("  [SIM] Starting  ->  make dev ...")
    if not dry_run:
        _run_make("dev")
        print(f"  [SIM] Waiting {SIM_STARTUP_SEC}s for containers to be ready ...")
        time.sleep(SIM_STARTUP_SEC)


def sim_stop(dry_run: bool):
    print("  [SIM] Stopping  ->  make down ...")
    if not dry_run:
        _run_make("down")
        time.sleep(DOWN_GRACE_SEC)


# ─────────────────────────────────────────────────────────────────────────────
#  Lap-time collection via /awsim/status topic
# ─────────────────────────────────────────────────────────────────────────────
#
#  /awsim/status  (std_msgs/Float32MultiArray)
#    data[0] = unused
#    data[1] = laps completed counter
#    data[2] = rolling lap timer (resets at each new lap)
#
#  Logic mirrors graph_panel.cpp:
#    When data[1] increments, the PREVIOUS data[2] is the finished lap time.
#

_DATA_ITEM_RE = re.compile(r"^- (.+)$")


def collect_lap_times(dry_run: bool) -> float:
    """
    Subscribe to /awsim/status and return the average of laps 2..LAPS_TO_COLLECT.
    Returns float('inf') on failure / timeout.
    """
    if dry_run:
        import random
        fake = random.uniform(45.0, 55.0)
        print(f"  [DRY-RUN] Fake avg lap time: {fake:.2f}s")
        return fake

    cmd = [
        "docker", "compose", "exec", "-T", "autoware",
        "bash", "-lc",
        (
            "source /opt/ros/humble/setup.bash && "
            f"ROS_DOMAIN_ID={ROS_DOMAIN_ID} "
            "ros2 topic echo /awsim/status std_msgs/msg/Float32MultiArray"
        ),
    ]

    total_timeout = LAP_TIMEOUT_SEC * LAPS_TO_COLLECT + 30

    try:
        proc = subprocess.Popen(
            cmd,
            cwd=PROJECT_ROOT,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
        )
    except Exception as exc:
        print(f"  ERROR starting docker exec: {exc}")
        return float("inf")

    lap_times = []
    current_lap = -1
    last_rolling = 0.0
    data_buf = []
    in_data_block = False
    deadline = time.time() + total_timeout

    try:
        for raw in proc.stdout:
            if time.time() > deadline:
                print(f"  TIMEOUT: only collected {len(lap_times)} laps")
                break

            line = raw.strip()

            if line.startswith("data:"):
                in_data_block = True
                data_buf = []
                continue

            if in_data_block:
                m = _DATA_ITEM_RE.match(line)
                if m:
                    try:
                        data_buf.append(float(m.group(1)))
                    except ValueError:
                        pass
                    continue

            if line == "---":
                if len(data_buf) >= 3:
                    lap_count  = int(data_buf[1])
                    rolling_t  = float(data_buf[2])

                    if current_lap == -1:
                        # First message: initialise
                        current_lap  = max(1, lap_count)
                        last_rolling = rolling_t
                    elif lap_count > current_lap:
                        # A lap just finished - last_rolling is authoritative
                        lap_times.append(last_rolling)
                        print(f"    Lap {current_lap} finished: {last_rolling:.2f}s")
                        current_lap = lap_count
                        if len(lap_times) >= LAPS_TO_COLLECT:
                            break

                    last_rolling = rolling_t

                in_data_block = False
                data_buf = []
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()

    if len(lap_times) < 2:
        print(f"  WARNING: collected only {len(lap_times)} laps - marking as infinity")
        return float("inf")

    # Average laps 2 and 3 (skip warm-up lap 1)
    valid = lap_times[1:LAPS_TO_COLLECT]
    avg   = sum(valid) / len(valid)
    lap_labels = "-".join(str(i + 1) for i in range(1, len(lap_times)))
    print(f"    -> Avg laps {lap_labels}: {avg:.2f}s")
    return avg


# ─────────────────────────────────────────────────────────────────────────────
#  Single simulation trial
# ─────────────────────────────────────────────────────────────────────────────

TOTAL_TRIALS = 0


def run_trial(cfg, dot_path: str, new_val, display_name: str, dry_run: bool) -> float:
    global TOTAL_TRIALS
    TOTAL_TRIALS += 1
    print(f"\n  -- Trial #{TOTAL_TRIALS}: {display_name} = {new_val}")
    set_value(cfg, dot_path, new_val)
    save_config(cfg)
    sim_start(dry_run)
    t = collect_lap_times(dry_run)
    sim_stop(dry_run)
    return t


# ─────────────────────────────────────────────────────────────────────────────
#  Value helpers
# ─────────────────────────────────────────────────────────────────────────────

def clamp(val, lo, hi):
    return max(lo, min(hi, val))


def step_value(current, direction: int, is_int: bool, lo, hi):
    """
    Apply +-TUNE_STEP in `direction` (+1 or -1).
    Returns the clamped candidate, or None if already at the boundary.
    """
    candidate = current * (1.0 + direction * TUNE_STEP)
    candidate = clamp(candidate, lo, hi)
    if is_int:
        candidate = int(round(candidate))
        # If rounding did not move us, force +-1 step
        if candidate == int(round(current)):
            candidate = int(round(current)) + direction
            candidate = max(int(lo), min(int(hi), candidate))
    # Signal boundary hit
    if is_int and candidate == int(round(current)):
        return None
    if not is_int and abs(candidate - current) < 1e-12:
        return None
    return candidate


# ─────────────────────────────────────────────────────────────────────────────
#  Hill-climbing for a single parameter
# ─────────────────────────────────────────────────────────────────────────────

def hill_climb_param(
    dot_path: str,
    display_name: str,
    is_int: bool,
    lo,
    hi,
    best_time: float,
    cfg,
    log: list,
    dry_run: bool,
):
    """
    Tune one parameter.  Returns (best_value_found, best_time_achieved).
    """
    original_value = get_value(cfg, dot_path)
    best_value     = original_value

    print(f"\n{'='*62}")
    print(f"  Tuning -> {display_name}")
    print(f"           current = {original_value},  best time so far = {best_time:.2f}s")
    print(f"{'='*62}")

    def explore_direction(start_value, direction: int, label: str) -> bool:
        """
        Repeatedly step in `direction` from `start_value` until no improvement.
        Returns True if at least one improvement was found.
        """
        nonlocal best_value, best_time
        current = start_value
        improved_any = False

        while True:
            candidate = step_value(current, direction, is_int, lo, hi)
            if candidate is None:
                print(f"  [{label}] Boundary reached - stopping.")
                break

            t = run_trial(cfg, dot_path, candidate, display_name, dry_run)

            if t < best_time:
                delta = best_time - t
                print(f"  [{label}] OK  {best_time:.2f}s -> {t:.2f}s  (-{delta:.2f}s)  "
                      f"keeping {candidate}")
                best_time  = t
                best_value = candidate
                current    = candidate
                improved_any = True
                log.append({
                    "param": display_name, "direction": label,
                    "value": candidate, "time": t, "result": "improved",
                })
            else:
                print(f"  [{label}] NO  {t:.2f}s >= {best_time:.2f}s  -  reverting")
                log.append({
                    "param": display_name, "direction": label,
                    "value": candidate, "time": t, "result": "reverted",
                })
                break

        return improved_any

    # Phase 1 - try +10 %
    improved = explore_direction(original_value, +1, "+10%")

    if not improved:
        # Phase 2 - try -10 % (only if +10 % never helped, reset from original)
        explore_direction(original_value, -1, "-10%")

    # Always restore the current best before moving on
    set_value(cfg, dot_path, best_value)
    save_config(cfg)

    if best_value != original_value:
        direction_arrow = "^" if best_value > original_value else "v"
        print(f"\n  OK {display_name}: {original_value} {direction_arrow} {best_value}  "
              f"[best time: {best_time:.2f}s]")
    else:
        print(f"\n  -- {display_name}: no change (kept {original_value})")

    return best_value, best_time


# ─────────────────────────────────────────────────────────────────────────────
#  Baseline
# ─────────────────────────────────────────────────────────────────────────────

def measure_baseline(cfg, dry_run: bool) -> float:
    print("\n" + "=" * 62)
    print("  BASELINE MEASUREMENT  (current config as-is)")
    print("=" * 62)
    sim_start(dry_run)
    t = collect_lap_times(dry_run)
    sim_stop(dry_run)
    if t == float("inf"):
        print("  ERROR: baseline failed - cannot proceed.")
    else:
        print(f"  Baseline avg lap time: {t:.2f}s")
    return t


# ─────────────────────────────────────────────────────────────────────────────
#  Persistence
# ─────────────────────────────────────────────────────────────────────────────

def save_results(log: list, baseline: float, best: float):
    data = {
        "timestamp":              datetime.now().isoformat(),
        "baseline_avg_lap_time":  baseline,
        "best_avg_lap_time":      best,
        "improvement_seconds":    round(baseline - best, 3),
        "total_trials":           TOTAL_TRIALS,
        "trial_log":              log,
    }
    with open(RESULTS_FILE, "w") as fh:
        json.dump(data, fh, indent=2)
    print(f"  [saved] {RESULTS_FILE}")


# ─────────────────────────────────────────────────────────────────────────────
#  Summary
# ─────────────────────────────────────────────────────────────────────────────

def print_summary(baseline: float, best: float, round_data: list, log: list):
    bar = "=" * 62
    print(f"\n{bar}")
    print("  TUNING COMPLETE")
    print(bar)
    print(f"  Baseline avg lap time : {baseline:.2f} s")
    print(f"  Best avg lap time     : {best:.2f} s")
    if baseline > 0:
        imp_s   = baseline - best
        imp_pct = imp_s / baseline * 100.0
        print(f"  Total improvement     : {imp_s:+.2f} s  ({imp_pct:.1f} %)")
    print(f"  Total simulation runs : {TOTAL_TRIALS}")

    # Per-round table
    if len(round_data) > 1:
        print(f"\n  {'Round':^6}  {'Start (s)':>10}  {'End (s)':>10}  {'Delta (s)':>10}")
        print("  " + "-" * 44)
        for rnd, s, e, d in round_data:
            print(f"  {rnd:^6}  {s:>10.2f}  {e:>10.2f}  {d:>+10.2f}")

    # Parameters that changed
    changed = {}
    for entry in log:
        if entry["result"] == "improved":
            changed[entry["param"]] = entry["value"]
    if changed:
        print(f"\n  Parameters kept (last improved value):")
        for name, val in changed.items():
            print(f"    {name:<35s} = {val}")
    else:
        print("\n  No parameters were improved.")
    print(bar)


# ─────────────────────────────────────────────────────────────────────────────
#  Main
# ─────────────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(
        description="MPC greedy hill-climbing tuner",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    ap.add_argument(
        "--dry-run", action="store_true",
        help="Simulate everything without calling make or docker",
    )
    ap.add_argument(
        "--rounds", type=int, default=1, metavar="N",
        help="Number of full tuning rounds (default: 1)",
    )
    ap.add_argument(
        "--start-param", type=str, default=None, metavar="NAME",
        help="Resume tuning from this parameter display name (skip earlier ones)",
    )
    args = ap.parse_args()

    if not CONFIG_PATH.exists():
        print(f"ERROR: config not found: {CONFIG_PATH}")
        print("       Run this script from the project root directory.")
        sys.exit(1)

    if args.dry_run:
        print("*** DRY-RUN mode - no make or docker commands will execute ***\n")

    log = []
    round_data = []
    cfg = load_config()

    # Baseline
    baseline_time = measure_baseline(cfg, args.dry_run)
    if baseline_time == float("inf"):
        sys.exit(1)

    best_time = baseline_time
    skip_to   = args.start_param

    for rnd in range(1, args.rounds + 1):
        print(f"\n{'#'*62}")
        print(f"  ROUND {rnd} / {args.rounds}   (current best: {best_time:.2f}s)")
        print(f"{'#'*62}")
        round_start = best_time

        for dot_path, name, is_int, lo, hi in PARAMS:
            # --start-param resume logic
            if skip_to is not None:
                if name == skip_to:
                    skip_to = None
                else:
                    print(f"  [skip] {name}")
                    continue

            try:
                _, best_time = hill_climb_param(
                    dot_path, name, is_int, lo, hi,
                    best_time, cfg, log, args.dry_run,
                )
            except KeyboardInterrupt:
                print("\n\nInterrupted by user - saving progress ...")
                save_results(log, baseline_time, best_time)
                sim_stop(args.dry_run)
                sys.exit(0)

            # Autosave after every parameter
            save_results(log, baseline_time, best_time)

        round_delta = round_start - best_time
        round_data.append((rnd, round_start, best_time, round_delta))
        print(f"\n  Round {rnd} done.  Delta = {round_delta:+.2f}s  "
              f"({round_start:.2f}s -> {best_time:.2f}s)")

    print_summary(baseline_time, best_time, round_data, log)
    save_results(log, baseline_time, best_time)


if __name__ == "__main__":
    main()
