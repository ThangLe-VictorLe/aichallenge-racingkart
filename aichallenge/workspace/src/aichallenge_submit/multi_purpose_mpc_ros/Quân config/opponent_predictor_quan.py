#!/usr/bin/env python3

import math
import yaml
import json
from typing import Dict, List, Tuple, Optional
import numpy as np

# ROS 2
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from rclpy.parameter import Parameter
from std_msgs.msg import String
from nav_msgs.msg import Odometry

# Autoware / V2X
from v2x_msgs.msg import V2XVehiclePositionArray

# Multi-purpose MPC modules
from multi_purpose_mpc_ros.common import convert_to_namedtuple
from multi_purpose_mpc_ros.core.map import Map, Obstacle
from multi_purpose_mpc_ros.core.reference_path import ReferencePath
from multi_purpose_mpc_ros.core.utils import load_waypoints, load_ref_path
from multi_purpose_mpc_ros.v2x_vehicle_tracker import V2XVehicleTracker


class OpponentPredictor(Node):
    PKG_PATH: str = get_package_share_directory('multi_purpose_mpc_ros') + "/"

    def __init__(self) -> None:
        super().__init__("opponent_predictor") # type: ignore

        # Declare parameters
        self.declare_parameter("config_path", self.PKG_PATH + "config/config.yaml")
        self.declare_parameter("prediction_horizon_sec", 2.0)
        self.declare_parameter("update_rate_hz", 10.0)
        self.declare_parameter("min_required_clearance", 1.45) # m (width of kart + small margin)
        self.declare_parameter("min_commitment_ticks", 15)      # ticks to lock side choice (1.5s at 10Hz)

        config_path = self.get_parameter("config_path").get_parameter_value().string_value
        self._cfg = self._load_config(config_path)

        # Map and reference path
        self._map = Map(self.in_pkg_share(self._cfg.map.yaml_path)) # type: ignore
        cfg_ref_path = self._cfg.reference_path # type: ignore
        if cfg_ref_path.csv_path != "":
            wp_x, wp_y, _, _ = load_ref_path(self.in_pkg_share(cfg_ref_path.csv_path))
        else:
            wp_x, wp_y = load_waypoints(self.in_pkg_share(self._cfg.waypoints.csv_path)) # type: ignore

        self._reference_path = ReferencePath(
            self._map,
            wp_x,
            wp_y,
            cfg_ref_path.resolution,
            cfg_ref_path.smoothing_distance,
            cfg_ref_path.max_width,
            cfg_ref_path.circular)

        # Player state
        self._player_x: Optional[float] = None
        self._player_y: Optional[float] = None

        # V2X Tracker
        v2x_cfg = self._cfg.v2x_obstacle_avoidance # type: ignore
        self._v2x_tracker = V2XVehicleTracker(
            v_max_safety=float(v2x_cfg.v_max_safety),
            position_jump_threshold=float(v2x_cfg.position_jump_threshold),
            warn_callback=self.get_logger().warn,
        )
        self._v2x_vehicle_radius = float(v2x_cfg.vehicle_radius)

        # MPC Horizon properties
        mpc_N = int(self._cfg.mpc.N) # type: ignore
        t_horizon = self.get_parameter("prediction_horizon_sec").get_parameter_value().double_value
        self._v2x_t_samples = [
            k * t_horizon / max(mpc_N - 1, 1) for k in range(mpc_N)
        ]

        # Side commitment state
        self._current_side: Optional[str] = None  # 'left', 'right', or None
        self._commitment_ticks_remaining = 0

        # Subscriptions and Publishers
        self._odom_sub = self.create_subscription(
            Odometry, "/localization/kinematic_state", self._odom_callback, 1)
        self._v2x_sub = self.create_subscription(
            V2XVehiclePositionArray, "/v2x/vehicle_positions", self._v2x_callback, 1)
        self._forecast_pub = self.create_publisher(
            String, "/opponent_forecast", 1)

        # Timer
        update_hz = self.get_parameter("update_rate_hz").get_parameter_value().double_value
        self._timer = self.create_timer(1.0 / update_hz, self._predict_and_publish)

        # Set sim time
        param = Parameter("use_sim_time", Parameter.Type.BOOL, True)
        self.set_parameters([param])

        self.get_logger().info("Opponent Predictor Node initialized successfully.")

    def in_pkg_share(self, file_path: str) -> str:
        return self.PKG_PATH + file_path

    def _load_config(self, config_path: str):
        with open(config_path, "r") as f:
            cfg = convert_to_namedtuple(yaml.safe_load(f))
        return cfg

    def _odom_callback(self, msg: Odometry) -> None:
        self._player_x = msg.pose.pose.position.x
        self._player_y = msg.pose.pose.position.y

    def _v2x_callback(self, msg: V2XVehiclePositionArray) -> None:
        self._v2x_tracker.update(msg)

    def _predict_path_aware(self, current_x: float, current_y: float, speed: float) -> List[Tuple[float, float]]:
        """
        Project the opponent's position forward along the track geometry to prevent
        wall penetration during curves.
        """
        closest_wp_idx = self._reference_path.get_closest_waypoint(current_x, current_y)
        predicted_points = []
        n_wps = self._reference_path.n_waypoints
        
        for t in self._v2x_t_samples:
            d = speed * t
            accumulated_dist = 0.0
            curr_idx = closest_wp_idx
            
            while accumulated_dist < d:
                next_idx = (curr_idx + 1) % n_wps
                wp_curr = self._reference_path.get_waypoint(curr_idx)
                wp_next = self._reference_path.get_waypoint(next_idx)
                dist = math.hypot(wp_next.x - wp_curr.x, wp_next.y - wp_curr.y)
                if accumulated_dist + dist >= d:
                    ratio = (d - accumulated_dist) / dist
                    x_interp = wp_curr.x + ratio * (wp_next.x - wp_curr.x)
                    y_interp = wp_curr.y + ratio * (wp_next.y - wp_curr.y)
                    predicted_points.append((x_interp, y_interp))
                    break
                accumulated_dist += dist
                curr_idx = next_idx
            else:
                wp = self._reference_path.get_waypoint(curr_idx)
                predicted_points.append((wp.x, wp.y))
                
        return predicted_points

    def _predict_and_publish(self) -> None:
        # If player position is not yet received, skip
        if self._player_x is None or self._player_y is None:
            return

        active_vids = self._v2x_tracker.active_vehicle_ids()
        n_wps = self._reference_path.n_waypoints
        player_wp_idx = self._reference_path.get_closest_waypoint(self._player_x, self._player_y)

        opponents_forecasts = {}
        closest_opp_id: Optional[str] = None
        min_progress_diff = float('inf')

        # Generate forecast points for all active opponents
        for vid in active_vids:
            samples = self._v2x_tracker._samples.get(vid)
            if not samples:
                continue
            t_last, x_last, y_last = samples[-1]

            # Filter out self (ego vehicle) — skip if V2X position matches player position (< 0.8m)
            if math.hypot(x_last - self._player_x, y_last - self._player_y) < 0.8:
                continue

            vx, vy = self._v2x_tracker.velocity(vid)
            speed = math.hypot(vx, vy)

            # Project opponent positions along track waypoints
            predicted_path = self._predict_path_aware(x_last, y_last, speed)
            opponents_forecasts[vid] = {
                'x': x_last,
                'y': y_last,
                'speed': speed,
                'path': predicted_path
            }

            # Identify if this opponent is ahead of us on the track
            opp_wp_idx = self._reference_path.get_closest_waypoint(x_last, y_last)
            progress_diff = (opp_wp_idx - player_wp_idx) % n_wps
            
            # Opponent is ahead if progress diff is in the front half of the loop
            if progress_diff < n_wps / 2:
                if progress_diff < min_progress_diff:
                    min_progress_diff = progress_diff
                    closest_opp_id = vid

        # Overtake decision logic
        decided_side = 'none'
        min_clearance = self.get_parameter("min_required_clearance").get_parameter_value().double_value

        if closest_opp_id is not None:
            opp = opponents_forecasts[closest_opp_id]
            opp_wp_idx = self._reference_path.get_closest_waypoint(opp['x'], opp['y'])
            wp_opp = self._reference_path.get_waypoint(opp_wp_idx)
            
            # Project opponent to get lateral error e_y
            # We use t2s coordinate projection mapping
            spatial_state = self._reference_path.t2s(
                [opp['x'], opp['y'], wp_opp.psi]
            )
            e_y_opp = spatial_state[0] # lateral error relative to centerline

            # Calculate clearances at opponent's current waypoint
            # wp_opp.ub is positive left limit, wp_opp.lb is negative right limit
            left_clearance = wp_opp.ub - e_y_opp - self._v2x_vehicle_radius
            right_clearance = e_y_opp - wp_opp.lb - self._v2x_vehicle_radius

            # Determine optimal side exit bias
            # If opponent is left of centerline (e_y > 0), passing right is aligned.
            # If opponent is right of centerline (e_y < 0), passing left is aligned.
            raceline_bias = 'right' if e_y_opp > 0 else 'left'

            # Apply side logic
            new_side = 'none'
            if left_clearance >= min_clearance and right_clearance >= min_clearance:
                new_side = raceline_bias
            elif left_clearance >= min_clearance:
                new_side = 'left'
            elif right_clearance >= min_clearance:
                new_side = 'right'
            else:
                new_side = 'none'

            # Handle lock-in side selection hysteresis
            min_ticks = self.get_parameter("min_commitment_ticks").get_parameter_value().integer_value
            if self._commitment_ticks_remaining > 0:
                self._commitment_ticks_remaining -= 1
                current_clearance = left_clearance if self._current_side == 'left' else right_clearance
                if self._current_side != 'none' and current_clearance >= min_clearance:
                    decided_side = self._current_side
                else:
                    self._commitment_ticks_remaining = 0

            if self._commitment_ticks_remaining == 0:
                if new_side != self._current_side:
                    self._commitment_ticks_remaining = min_ticks
                self._current_side = new_side
                decided_side = new_side

        # Publish the JSON forecast
        forecast_payload = {
            'recommended_pass_side': decided_side,
            'opponents': {vid: opp['path'] for vid, opp in opponents_forecasts.items()}
        }

        msg = String()
        msg.data = json.dumps(forecast_payload)
        self._forecast_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OpponentPredictor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
