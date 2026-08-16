import re

with open("/mnt/ZorinData/aichallenge-racingkart/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/multi_purpose_mpc_ros/mpc_controller.py", "r") as f:
    content = f.read()

old_v2x = """        if self._v2x_avoidance_active:
            predictions = self._v2x_tracker.predict_all(self._v2x_t_samples)
            new_dynamic = predictions_to_obstacles(predictions, self._v2x_vehicle_radius)
            # Only trigger a map rebuild if obstacle positions changed meaningfully (>0.1m).
            # This prevents continuous corridor redraw on every V2X callback when the
            # opponent is stationary or barely moving (e.g. ego kart stopped in manual mode).
            if self._obstacles_changed(new_dynamic):
                self._dynamic_obstacles = new_dynamic"""

new_v2x = """        if self._v2x_avoidance_active:
            predictions = self._v2x_tracker.predict_all(self._v2x_t_samples)
            new_dynamic = predictions_to_obstacles(predictions, self._v2x_vehicle_radius)
            
            # Populate Frenet s for each obstacle to prevent ghost projection bugs in sharp curves
            if self._waypoint_xy.size > 0:
                for ob in new_dynamic:
                    dxy = self._waypoint_xy - np.array([ob.cx, ob.cy], dtype=np.float64)
                    dists_sq = np.einsum('ij,ij->i', dxy, dxy)
                    nearest_wp = int(np.argmin(dists_sq))
                    ob.s = self._waypoint_s[nearest_wp]

            # Only trigger a map rebuild if obstacle positions changed meaningfully (>0.1m).
            # This prevents continuous corridor redraw on every V2X callback when the
            # opponent is stationary or barely moving (e.g. ego kart stopped in manual mode).
            if self._obstacles_changed(new_dynamic):
                self._dynamic_obstacles = new_dynamic"""

content = content.replace(old_v2x, new_v2x)

# Fix 2: Only pass dynamic_obs if overtaking is active (not just tailing)
old_tailing = """        # === TAILING LOGIC (Idea A) ===
        if self.USE_OBSTACLE_AVOIDANCE and getattr(self, '_v2x_avoidance_active', False):
            nearest_vid = getattr(self, '_nearest_opponent_vid', None)
            min_ds = getattr(self, '_nearest_opponent_ds', float('inf'))
            
            if nearest_vid is not None and self._v2x_launch_distance < min_ds < self._v2x_tailing_distance:
                opp_vx, opp_vy = self._v2x_tracker.velocity(nearest_vid)
                opp_speed_mps = np.hypot(opp_vx, opp_vy)
                opp_speed_kmph = opp_speed_mps * 3.6
                
                if opp_speed_kmph >= self._v2x_ignore_tailing_vel:
                    speed_delta = max(0.5, min_ds * 0.1) # m/s
                    tail_speed_mps = opp_speed_mps + speed_delta
                    target_v_max_mps = min(target_v_max_mps, tail_speed_mps)
                    self.get_logger().info(f"[TAILING] Opponent '{nearest_vid}' is {min_ds:.1f}m ahead. Speed: {opp_speed_kmph:.1f} km/h. Target v_max: {target_v_max_mps*3.6:.1f} km/h.", throttle_duration_sec=1.0)
                else:
                    self.get_logger().info(f"[TAILING] Opponent is slow ({opp_speed_kmph:.1f} km/h). Ignoring tailing, executing overtake immediately.", throttle_duration_sec=1.0)
            elif nearest_vid is not None and min_ds <= self._v2x_launch_distance:
                self.get_logger().info(f"[TAILING] OVERTAKE! Launch distance reached ({min_ds:.1f}m <= {self._v2x_launch_distance:.1f}m)! Full speed ahead.", throttle_duration_sec=1.0)

        # === SMOOTH RAMP DOWN OF V_MAX ==="""

new_tailing = """        # === TAILING LOGIC (Idea A) ===
        is_overtaking_active = False
        if self.USE_OBSTACLE_AVOIDANCE and getattr(self, '_v2x_avoidance_active', False):
            nearest_vid = getattr(self, '_nearest_opponent_vid', None)
            min_ds = getattr(self, '_nearest_opponent_ds', float('inf'))
            
            if nearest_vid is not None:
                opp_vx, opp_vy = self._v2x_tracker.velocity(nearest_vid)
                opp_speed_mps = np.hypot(opp_vx, opp_vy)
                opp_speed_kmph = opp_speed_mps * 3.6
                
                if min_ds <= self._v2x_launch_distance or opp_speed_kmph < self._v2x_ignore_tailing_vel:
                    is_overtaking_active = True
                
                if self._v2x_launch_distance < min_ds < self._v2x_tailing_distance:
                    if opp_speed_kmph >= self._v2x_ignore_tailing_vel:
                        speed_delta = max(0.5, min_ds * 0.1) # m/s
                        tail_speed_mps = opp_speed_mps + speed_delta
                        target_v_max_mps = min(target_v_max_mps, tail_speed_mps)
                        self.get_logger().info(f"[TAILING] Opponent '{nearest_vid}' is {min_ds:.1f}m ahead. Speed: {opp_speed_kmph:.1f} km/h. Target v_max: {target_v_max_mps*3.6:.1f} km/h.", throttle_duration_sec=1.0)
                    else:
                        self.get_logger().info(f"[TAILING] Opponent is slow ({opp_speed_kmph:.1f} km/h). Ignoring tailing, executing overtake immediately.", throttle_duration_sec=1.0)
                elif min_ds <= self._v2x_launch_distance:
                    self.get_logger().info(f"[TAILING] OVERTAKE! Launch distance reached ({min_ds:.1f}m <= {self._v2x_launch_distance:.1f}m)! Full speed ahead.", throttle_duration_sec=1.0)

        # === SMOOTH RAMP DOWN OF V_MAX ==="""

content = content.replace(old_tailing, new_tailing)

old_get_control = """        # === MPC SOLVER ===
        with self._stats.time_block("control"):
            dynamic_obs = self._dynamic_obstacles if (self.USE_OBSTACLE_AVOIDANCE and self._v2x_avoidance_active) else []
            u, max_delta = self._mpc.get_control(dynamic_obstacles=dynamic_obs)"""

new_get_control = """        # === MPC SOLVER ===
        with self._stats.time_block("control"):
            dynamic_obs = self._dynamic_obstacles if (self.USE_OBSTACLE_AVOIDANCE and is_overtaking_active) else []
            u, max_delta = self._mpc.get_control(dynamic_obstacles=dynamic_obs)"""

content = content.replace(old_get_control, new_get_control)

with open("/mnt/ZorinData/aichallenge-racingkart/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/multi_purpose_mpc_ros/mpc_controller.py", "w") as f:
    f.write(content)
