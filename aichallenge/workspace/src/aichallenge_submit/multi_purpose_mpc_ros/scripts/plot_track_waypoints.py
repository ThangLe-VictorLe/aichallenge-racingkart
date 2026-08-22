#!/usr/bin/env python3
"""
plot_track_waypoints.py
─────────────────────────────────────────────────────────────────────────────
Interactive track-waypoint map with:
  • File-watcher thread  – detects saves to steer_gain.yaml / ref_vel.yaml
                           and schedules a redraw WITHOUT touching the window
  • [Reload] button      – manual refresh at any time
  • [Save PNG] button    – export current view as a PNG
  • --save flag          – non-interactive PNG-export mode (same as before)
─────────────────────────────────────────────────────────────────────────────
"""

import os
import sys
import argparse
import threading
import time
import yaml

# ── choose a non-popping backend before importing pyplot ──────────────────
# We test-import each backend module first; broken ones (e.g. TkAgg when
# PIL.ImageTk is missing) raise ImportError before we commit to them.
import importlib
import matplotlib
for _be in ('Qt5Agg', 'QtAgg', 'TkAgg', 'GTK3Agg', 'WXAgg'):
    try:
        importlib.import_module(f'matplotlib.backends.backend_{_be.lower()}')
        matplotlib.use(_be)
        break
    except Exception:
        pass

import matplotlib.pyplot as plt
from matplotlib.widgets import Button
import numpy as np


# ═══════════════════════════════════════════════════════════════════════════
# YAML / CSV helpers
# ═══════════════════════════════════════════════════════════════════════════

def load_yaml(filepath):
    if not os.path.exists(filepath):
        return {}
    with open(filepath, 'r') as f:
        return yaml.safe_load(f) or {}


def build_track_xy(csv_path, resolution=0.6, smoothing_distance=2):
    """Mirrors ReferencePath interpolation logic.
    Returns (x, y, track_length_m) where track_length_m is the arc-length from s_m column.
    """
    data = np.genfromtxt(csv_path, delimiter=',', names=True)
    if 'x_m' in data.dtype.names and 'y_m' in data.dtype.names:
        raw_x = data['x_m'].tolist()
        raw_y = data['y_m'].tolist()
    elif 'x' in data.dtype.names and 'y' in data.dtype.names:
        raw_x = data['x'].tolist()
        raw_y = data['y'].tolist()
    else:
        raise ValueError("Could not parse x,y columns from CSV")

    # Read arc-length for display info
    track_length_m = float(data['s_m'][-1]) if 's_m' in data.dtype.names else 0.0

    raw_x = raw_x + raw_x[:smoothing_distance * 3]
    raw_y = raw_y + raw_y[:smoothing_distance * 3]

    n_wp = [max(1, int(np.sqrt((raw_x[i+1]-raw_x[i])**2 +
                               (raw_y[i+1]-raw_y[i])**2) / resolution))
            for i in range(len(raw_x)-1)]

    gp_x, gp_y = raw_x[-1], raw_y[-1]
    wp_x = [np.linspace(raw_x[i], raw_x[i+1], n_wp[i], endpoint=False).tolist()
            for i in range(len(raw_x)-1)]
    wp_x = [w for seg in wp_x for w in seg] + [gp_x]
    wp_y = [np.linspace(raw_y[i], raw_y[i+1], n_wp[i], endpoint=False).tolist()
            for i in range(len(raw_y)-1)]
    wp_y = [w for seg in wp_y for w in seg] + [gp_y]

    xs, ys = [], []
    for i in range(smoothing_distance, len(wp_x)-smoothing_distance):
        xs.append(np.mean(wp_x[i-smoothing_distance:i+smoothing_distance+1]))
        ys.append(np.mean(wp_y[i-smoothing_distance:i+smoothing_distance+1]))
    return np.array(xs), np.array(ys), track_length_m


def parse_gain_vel_maps(steer_gain_path, ref_vel_path):
    steer_cfg = load_yaml(steer_gain_path).get('steer_gain_configurator', {})
    ref_cfg   = load_yaml(ref_vel_path).get('ref_vel_configulator', {})

    gain_map = {int(v['wp_id']): float(v['gain'])
                for v in steer_cfg.values()
                if isinstance(v, dict) and 'wp_id' in v and 'gain' in v}

    vel_map  = {int(v['wp_id']): float(v['ref_vel'])
                for v in ref_cfg.values()
                if isinstance(v, dict) and 'wp_id' in v and 'ref_vel' in v}

    return gain_map, vel_map


# ═══════════════════════════════════════════════════════════════════════════
# Drawing
# ═══════════════════════════════════════════════════════════════════════════

def draw(ax, x, y, gain_map, vel_map, step, status_text="", track_length_m=0.0):
    """Redraw the track on *ax* (does NOT call canvas.draw)."""
    ax.clear()
    ax.set_facecolor('#1e1e1e')

    ax.plot(x, y, color='#00adb5', linewidth=2.5, alpha=0.7, label='Raceline Path')
    ax.scatter(x[0], y[0], color='#ff5722', s=120, zorder=5, label='Start/Finish Line (WP 0)')

    num_wps = len(x)
    for i in range(0, num_wps, step):
        px, py = x[i], y[i]
        gain_str = f"{gain_map[i]:.2f}" if i in gain_map else ""
        vel_str  = f"{vel_map[i]:.1f}"  if i in vel_map  else ""

        label = f"wp{i}"
        if vel_str:  label += f"\n{vel_str}k"
        if gain_str: label += f"\ng:{gain_str}"

        ax.scatter(px, py, color='#ffc107', s=35, zorder=4)
        ax.annotate(label, (px, py),
                    textcoords="offset points", xytext=(0, 10),
                    ha='center', fontsize=7, color='#ffffff',
                    bbox=dict(boxstyle="round,pad=0.2", fc="#222831",
                              ec="#393e46", lw=1, alpha=0.8))

    arc_info = f"  |  Track: {track_length_m:.1f} m @ 0.6 m/WP" if track_length_m else ""
    title = (f"Track Waypoints Map  (WP 0 – {num_wps-1}  |  {num_wps} total{arc_info}  |  step: {step})\n"
             f"Watching steer_gain.yaml & ref_vel.yaml  {status_text}")
    ax.set_title(title, color='#ffffff', fontsize=13, pad=12)
    ax.set_xlabel("X (m)", color='#eeeeee', fontsize=12)
    ax.set_ylabel("Y (m)", color='#eeeeee', fontsize=12)
    ax.tick_params(colors='#eeeeee')
    ax.axis('equal')
    ax.grid(True, color='#333333', linestyle='--', alpha=0.5)
    ax.legend(facecolor='#222831', edgecolor='#393e46',
              labelcolor='#eeeeee', loc='upper right')


# ═══════════════════════════════════════════════════════════════════════════
# File-watcher thread  (inotify if available, else mtime polling)
# ═══════════════════════════════════════════════════════════════════════════

def _get_mtimes(paths):
    out = {}
    for p in paths:
        try:
            out[p] = os.path.getmtime(p)
        except OSError:
            out[p] = None
    return out


def make_watcher_thread(paths, on_change_cb, poll_sec=1.0):
    """
    Return a daemon thread that watches *paths* for mtime changes.
    Calls on_change_cb(changed_basenames) when any file changes.
    Uses inotify (Linux) if available, otherwise mtime polling.
    """
    # ── try inotify (zero-latency, no polling) ───────────────────────────
    try:
        import inotify.adapters

        def _run_inotify():
            watch = inotify.adapters.Inotify()
            dirs_seen = set()
            for p in paths:
                d = os.path.dirname(p)
                if d not in dirs_seen:
                    watch.add_watch(d)
                    dirs_seen.add(d)

            watched_names = {os.path.basename(p) for p in paths}
            for event in watch.event_gen(yield_nones=False):
                _, type_names, path, filename = event
                if filename in watched_names and any(
                        t in type_names for t in
                        ('IN_CLOSE_WRITE', 'IN_MOVED_TO', 'IN_MODIFY')):
                    on_change_cb([filename])

        t = threading.Thread(target=_run_inotify, daemon=True, name="inotify-watcher")
        print("[WATCHER] Using inotify (zero-latency file events)")
        return t

    except ImportError:
        pass

    # ── fallback: mtime polling ──────────────────────────────────────────
    last = _get_mtimes(paths)

    def _run_poll():
        nonlocal last
        while True:
            time.sleep(poll_sec)
            current = _get_mtimes(paths)
            changed = [os.path.basename(p) for p in paths
                       if current[p] != last[p]]
            if changed:
                last = current
                on_change_cb(changed)

    t = threading.Thread(target=_run_poll, daemon=True, name="mtime-watcher")
    print(f"[WATCHER] Using mtime polling every {poll_sec}s")
    return t


# ═══════════════════════════════════════════════════════════════════════════
# Interactive live window
# ═══════════════════════════════════════════════════════════════════════════

def live_plot(csv_path, steer_gain_path, ref_vel_path, step, output_png):
    print(f"[INFO] CSV       : {csv_path}")
    print(f"[INFO] Steer gain: {steer_gain_path}")
    print(f"[INFO] Ref vel   : {ref_vel_path}")
    print(f"[INFO] Label step: every {step} waypoints")

    x, y, track_length_m = build_track_xy(csv_path)
    print(f"[INFO] Track: {track_length_m:.1f} m arc-length → {len(x)} WPs (WP 0 – {len(x)-1})")

    # ── state shared between thread and UI ───────────────────────────────
    _pending = threading.Event()   # set by watcher, cleared after redraw
    _status  = [""]                # last status message (list for mutability)

    def on_file_changed(changed_names):
        _status[0] = f"↻ reloaded ({', '.join(changed_names)})"
        _pending.set()
        print(f"[RELOAD] {', '.join(changed_names)} changed → scheduling redraw")

    # ── build figure ─────────────────────────────────────────────────────
    fig = plt.figure(figsize=(16, 12))
    fig.patch.set_facecolor('#1e1e1e')

    # Main axes (leave room at bottom for buttons)
    ax = fig.add_axes([0.05, 0.10, 0.92, 0.83])

    gain_map, vel_map = parse_gain_vel_maps(steer_gain_path, ref_vel_path)
    draw(ax, x, y, gain_map, vel_map, step, "", track_length_m)
    fig.canvas.draw()

    # ── Reload button ─────────────────────────────────────────────────────
    ax_btn_reload = fig.add_axes([0.05, 0.02, 0.12, 0.045])
    btn_reload = Button(ax_btn_reload, '⟳  Reload',
                        color='#222831', hovercolor='#393e46')
    btn_reload.label.set_color('#00adb5')
    btn_reload.label.set_fontsize(11)

    # ── Save PNG button ───────────────────────────────────────────────────
    ax_btn_save = fig.add_axes([0.19, 0.02, 0.12, 0.045])
    btn_save = Button(ax_btn_save, '💾  Save PNG',
                      color='#222831', hovercolor='#393e46')
    btn_save.label.set_color('#ffc107')
    btn_save.label.set_fontsize(11)

    # ── status label ─────────────────────────────────────────────────────
    status_text_obj = fig.text(0.35, 0.025, "", color='#aaaaaa', fontsize=9,
                               va='center')

    def do_reload(src="manual"):
        nonlocal gain_map, vel_map
        try:
            gain_map, vel_map = parse_gain_vel_maps(steer_gain_path, ref_vel_path)
            draw(ax, x, y, gain_map, vel_map, step, _status[0], track_length_m)
            status_text_obj.set_text(f"[{src}] reloaded at {time.strftime('%H:%M:%S')}")
            fig.canvas.draw_idle()
        except Exception as e:
            status_text_obj.set_text(f"[ERROR] {e}")
            fig.canvas.draw_idle()
            print(f"[WARN] {e}")

    def on_btn_reload(event):
        _status[0] = ""
        do_reload("button")

    def on_btn_save(event):
        try:
            os.makedirs(os.path.dirname(os.path.abspath(output_png)), exist_ok=True)
            fig.savefig(output_png, bbox_inches='tight',
                        facecolor=fig.get_facecolor(), dpi=150)
            status_text_obj.set_text(f"[saved] {output_png}")
            fig.canvas.draw_idle()
            print(f"[SAVE] → {output_png}")
        except Exception as e:
            print(f"[WARN] Save failed: {e}")

    btn_reload.on_clicked(on_btn_reload)
    btn_save.on_clicked(on_btn_save)

    # ── canvas timer – checks _pending flag ──────────────────────────────
    # Fires every 200 ms but does NO redraw unless the flag is set.
    # This keeps the window completely still between file saves.
    def _timer_tick():
        if _pending.is_set():
            _pending.clear()
            do_reload("auto")

    timer = fig.canvas.new_timer(interval=200)
    timer.add_callback(_timer_tick)
    timer.start()

    # ── start watcher thread ──────────────────────────────────────────────
    watcher = make_watcher_thread(
        [steer_gain_path, ref_vel_path],
        on_file_changed,
        poll_sec=1.0
    )
    watcher.start()

    plt.show()   # blocks here; window stays focused until user closes it
    timer.stop()


# ═══════════════════════════════════════════════════════════════════════════
# Non-interactive PNG save mode
# ═══════════════════════════════════════════════════════════════════════════

def save_plot(csv_path, steer_gain_path, ref_vel_path, output_png, step):
    matplotlib.use('Agg')
    x, y, track_length_m = build_track_xy(csv_path)
    gain_map, vel_map = parse_gain_vel_maps(steer_gain_path, ref_vel_path)

    fig, ax = plt.subplots(figsize=(16, 12), dpi=150)
    fig.patch.set_facecolor('#1e1e1e')
    draw(ax, x, y, gain_map, vel_map, step, "", track_length_m)

    os.makedirs(os.path.dirname(os.path.abspath(output_png)), exist_ok=True)
    plt.savefig(output_png, bbox_inches='tight', facecolor=fig.get_facecolor())
    plt.close()
    print(f"[SUCCESS] Saved → {output_png}")


# ═══════════════════════════════════════════════════════════════════════════
# Main
# ═══════════════════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(
        description=(
            "Interactive track waypoint map.\n"
            "Live window by default; use --save to export PNG."
        )
    )
    parser.add_argument("-c", "--csv",        help="Path to mincurv CSV (overrides config.yaml)")
    parser.add_argument("-s", "--steer-gain", help="Path to steer_gain.yaml")
    parser.add_argument("-r", "--ref-vel",    help="Path to ref_vel.yaml")
    parser.add_argument("-o", "--output",     help="Output PNG path")
    parser.add_argument("--step", type=int, default=5,
                        help="Waypoint label interval (default: 5)")
    parser.add_argument("--save", action="store_true",
                        help="Export PNG and exit instead of opening window")
    args = parser.parse_args()

    pkg_dir    = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    config_dir = os.path.join(pkg_dir, "config")

    steer_gain_path = args.steer_gain or os.path.join(config_dir, "steer_gain.yaml")
    ref_vel_path    = args.ref_vel    or os.path.join(config_dir, "ref_vel.yaml")

    # ── resolve CSV ───────────────────────────────────────────────────────
    csv_path = args.csv
    if not csv_path:
        try:
            with open(os.path.join(config_dir, "config.yaml"), 'r') as f:
                cfg = yaml.safe_load(f)
                rel = cfg.get("reference_path", {}).get("csv_path", "")
                if rel:
                    cand = os.path.join(pkg_dir, rel)
                    if os.path.exists(cand):
                        csv_path = cand
        except Exception:
            pass

    if not csv_path or not os.path.exists(csv_path):
        csv_path = os.path.join(pkg_dir, "env", "preliminary", "optimized_traj_mincurv.csv")

    if not os.path.exists(csv_path):
        print(f"[ERROR] CSV not found: {csv_path}")
        sys.exit(1)

    output_png = args.output or os.path.join(config_dir, "track_waypoint_map.png")

    if args.save:
        save_plot(csv_path, steer_gain_path, ref_vel_path, output_png, step=args.step)
    else:
        live_plot(csv_path, steer_gain_path, ref_vel_path, step=args.step,
                  output_png=output_png)


if __name__ == '__main__':
    main()
