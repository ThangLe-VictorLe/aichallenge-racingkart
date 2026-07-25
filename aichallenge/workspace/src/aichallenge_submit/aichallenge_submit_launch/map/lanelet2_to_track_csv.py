#!/usr/bin/env python3
"""
lanelet2_to_track_csv.py

Purpose
-------
Solve two problems at once:
  1) Your lanelet2 IDs are not in order -> use the lanelet2 RoutingGraph to
     walk the lap in physical driving order, instead of sorting by ID.
  2) You have no explicit "width" file -> a lanelet already IS two boundary
     lines (leftBound, rightBound). Width at any point = distance from that
     point to those two lines. We compute it directly from geometry.

Output: inputs/tracks/<name>.csv in the TUM global_racetrajectory_optimization
format: x_m, y_m, w_tr_right_m, w_tr_left_m
(https://github.com/TUMFTM/global_racetrajectory_optimization)

Requirements
------------
Run this INSIDE your Autoware workspace (it already has the lanelet2 python
bindings: `lanelet2`, built as part of autoware_lanelet2_extension / core).

    python3 lanelet2_to_track_csv.py \
        --map /path/to/lanelet2_map.osm \
        --start-lanelet-id 1234 \
        --out track.csv \
        --resolution 1.0
"""

import argparse
import csv
import math

import lanelet2
from lanelet2.core import BasicPoint2d
from lanelet2.geometry import distance as ll2_distance
from lanelet2.projection import UtmProjector


def load_map(map_path: str):
    # AWSIM/Autoware lanelet2 maps are usually lat/lon (MGRS/UTM projected).
    # If your map load fails here, your team's origin lat/lon needs to be set;
    # check the `map_projector_info.yaml` that ships with your AWSIM map.
    projector = UtmProjector(lanelet2.io.Origin(0.0, 0.0))
    lmap = lanelet2.io.load(map_path, projector)
    return lmap


def build_routing_graph(lmap):
    traffic_rules = lanelet2.traffic_rules.create(
        lanelet2.traffic_rules.Locations.Germany,
        lanelet2.traffic_rules.Participants.Vehicle,
    )
    graph = lanelet2.routing.RoutingGraph(lmap, traffic_rules)
    return graph, traffic_rules


def order_lanelets_around_loop(lmap, graph, start_id: int):
    """
    Walk the lap using the routing graph's 'following' relation, instead of
    trusting lanelet ID order. Stops when we loop back to the start lanelet
    (closed circuit) or hit a dead end (open track -> handle that branch
    manually if your track isn't a closed loop).
    """
    start = lmap.laneletLayer[start_id]
    ordered = [start]
    current = start
    visited_ids = {start.id}

    while True:
        followers = graph.following(current)
        if not followers:
            print("WARNING: reached a dead end. Track is not a closed loop, "
                  "or routing graph is missing a connection. Stopping here.")
            break

        nxt = followers[0]  # for a single-lane race track there should be
                             # exactly one; if you get branches (pit lane,
                             # etc.) filter followers by your own rule here.

        if nxt.id == start.id:
            print(f"Closed the loop after {len(ordered)} lanelets.")
            break
        if nxt.id in visited_ids:
            print("WARNING: revisited a lanelet without reaching start "
                  "(routing graph may have a branch). Stopping here.")
            break

        ordered.append(nxt)
        visited_ids.add(nxt.id)
        current = nxt

    return ordered


def resample_polyline(points_xy, resolution_m: float):
    """Resample a list of (x,y) to roughly even spacing."""
    if len(points_xy) < 2:
        return points_xy

    # cumulative arc length
    cum = [0.0]
    for i in range(1, len(points_xy)):
        x0, y0 = points_xy[i - 1]
        x1, y1 = points_xy[i]
        cum.append(cum[-1] + math.hypot(x1 - x0, y1 - y0))
    total_len = cum[-1]
    if total_len <= 0:
        return points_xy

    n_samples = max(2, int(total_len / resolution_m))
    targets = [i * total_len / n_samples for i in range(n_samples)]

    out = []
    j = 0
    for t in targets:
        while j < len(cum) - 2 and cum[j + 1] < t:
            j += 1
        seg_len = cum[j + 1] - cum[j]
        ratio = 0.0 if seg_len <= 0 else (t - cum[j]) / seg_len
        x0, y0 = points_xy[j]
        x1, y1 = points_xy[j + 1]
        out.append((x0 + ratio * (x1 - x0), y0 + ratio * (y1 - y0)))
    return out


def widths_at_point(pt_xy, left_bound, right_bound):
    p = BasicPoint2d(pt_xy[0], pt_xy[1])
    w_left = ll2_distance(p, left_bound)
    w_right = ll2_distance(p, right_bound)
    return w_right, w_left  # TUM order is (right, left)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--map", required=True, help="path to lanelet2 .osm file")
    ap.add_argument("--start-lanelet-id", required=True, type=int)
    ap.add_argument("--out", required=True, help="output track csv path")
    ap.add_argument("--resolution", type=float, default=1.0,
                     help="centerline sampling resolution in meters")
    args = ap.parse_args()

    lmap = load_map(args.map)
    graph, _ = build_routing_graph(lmap)
    ordered = order_lanelets_around_loop(lmap, graph, args.start_lanelet_id)

    rows = []
    for lanelet in ordered:
        centerline = lanelet.centerline  # lanelet2 computes this for you
        left = lanelet.leftBound
        right = lanelet.rightBound

        raw_pts = [(p.x, p.y) for p in centerline]
        pts = resample_polyline(raw_pts, args.resolution)

        for pt in pts:
            w_right, w_left = widths_at_point(pt, left, right)
            rows.append((pt[0], pt[1], w_right, w_left))

    with open(args.out, "w", newline="") as f:
        writer = csv.writer(f, delimiter=";")
        writer.writerow(["# x_m", "y_m", "w_tr_right_m", "w_tr_left_m"])
        for r in rows:
            writer.writerow([f"{v:.4f}" for v in r])

    print(f"Wrote {len(rows)} track points to {args.out}")
    print("Feed this into TUMFTM/global_racetrajectory_optimization as "
          "inputs/tracks/<name>.csv and run the mintime optimizer.")


if __name__ == "__main__":
    main()
