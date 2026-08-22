import re

with open("/mnt/ZorinData/aichallenge-racingkart/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/multi_purpose_mpc_ros/mpc_controller.py", "r") as f:
    content = f.read()

old_tailing = """        # === TAILING LOGIC (Idea A) ===
        is_overtaking_active = False
        in_no_overtake_zone = False
        ego_wp_id = self._mpc.model.wp_id if self._mpc is not None else 0
        ego_s = getattr(self, '_waypoint_s', np.array([]))
        if ego_s.size > 0:
            current_s = ego_s[min(ego_wp_id, len(ego_s) - 1)]
            for zone in getattr(self, '_no_overtake_zones', []):
                if len(zone) == 2 and zone[0] <= current_s <= zone[1]:
                    in_no_overtake_zone = True
                    break

        if self.USE_OBSTACLE_AVOIDANCE and getattr(self, '_v2x_avoidance_active', False):
            nearest_vid = getattr(self, '_nearest_opponent_vid', None)
            min_ds = getattr(self, '_nearest_opponent_ds', float('inf'))
            
            if nearest_vid is not None:
                opp_vx, opp_vy = self._v2x_tracker.velocity(nearest_vid)
                opp_speed_mps = np.hypot(opp_vx, opp_vy)
                opp_speed_kmph = opp_speed_mps * 3.6
                
                # We can only overtake if we are not in a no-overtake zone
                # UNLESS the opponent has crashed/stopped and we are stuck behind them!
                opponent_crashed = opp_speed_kmph < 2.0 and min_ds < self._v2x_launch_distance
                can_overtake = (not in_no_overtake_zone) or opponent_crashed
                
                if can_overtake and (min_ds <= self._v2x_launch_distance or opp_speed_kmph < self._v2x_ignore_tailing_vel):
                    is_overtaking_active = True
                
                # Tailing condition: Either we are far enough (but in tailing range), or we are forced to wait
                if (self._v2x_launch_distance < min_ds < self._v2x_tailing_distance) or (in_no_overtake_zone and min_ds < self._v2x_tailing_distance):
                    if in_no_overtake_zone or opp_speed_kmph >= self._v2x_ignore_tailing_vel:
                        speed_delta = max(0.5, min_ds * 0.1) # m/s
                        tail_speed_mps = opp_speed_mps + speed_delta
                        target_v_max_mps = min(target_v_max_mps, tail_speed_mps)
                        if in_no_overtake_zone:
                            self.get_logger().info(f"[TAILING] No-Overtake Zone (s={current_s:.1f}). Tailing '{nearest_vid}' at {min_ds:.1f}m. Target v_max: {target_v_max_mps*3.6:.1f} km/h.", throttle_duration_sec=1.0)
                        else:
                            self.get_logger().info(f"[TAILING] Opponent '{nearest_vid}' is {min_ds:.1f}m ahead. Speed: {opp_speed_kmph:.1f} km/h. Target v_max: {target_v_max_mps*3.6:.1f} km/h.", throttle_duration_sec=1.0)
                    else:
                        self.get_logger().info(f"[TAILING] Opponent is slow ({opp_speed_kmph:.1f} km/h). Ignoring tailing, executing overtake immediately.", throttle_duration_sec=1.0)
                elif min_ds <= self._v2x_launch_distance and can_overtake:
                    self.get_logger().info(f"[TAILING] OVERTAKE! Launch distance reached ({min_ds:.1f}m <= {self._v2x_launch_distance:.1f}m)! Full speed ahead.", throttle_duration_sec=1.0)"""

new_tailing = """        # === TAILING & LAUNCH MODE LOGIC ===
        is_overtaking_active = False
        self._is_launch_mode_active = False
        in_no_overtake_zone = False
        
        ego_wp_id = self._mpc.model.wp_id if self._mpc is not None else 0
        ego_s = getattr(self, '_waypoint_s', np.array([]))
        if ego_s.size > 0:
            current_s = ego_s[min(ego_wp_id, len(ego_s) - 1)]
            for zone in getattr(self, '_no_overtake_zones', []):
                if len(zone) == 2 and zone[0] <= current_s <= zone[1]:
                    in_no_overtake_zone = True
                    break

        if self.USE_OBSTACLE_AVOIDANCE and getattr(self, '_v2x_avoidance_active', False):
            nearest_vid = getattr(self, '_nearest_opponent_vid', None)
            min_ds = getattr(self, '_nearest_opponent_ds', float('inf'))
            
            if nearest_vid is not None:
                opp_vx, opp_vy = self._v2x_tracker.velocity(nearest_vid)
                opp_speed_mps = np.hypot(opp_vx, opp_vy)
                opp_speed_kmph = opp_speed_mps * 3.6
                
                launch_distance_reached = (min_ds <= self._v2x_launch_distance)
                oppo_crash_or_too_slow = (opp_speed_kmph < self._v2x_ignore_tailing_vel)
                
                speed_delta = max(0.5, min_ds * 0.1) # m/s
                tail_speed_mps = opp_speed_mps + speed_delta
                
                if not in_no_overtake_zone:
                    if launch_distance_reached or oppo_crash_or_too_slow:
                        # Launch mode activate!
                        self._is_launch_mode_active = True
                        is_overtaking_active = True
                        # target_v_max_mps remains self._mpc_cfg.v_max
                        self.get_logger().info(f"[LAUNCH MODE] Launch distance reached ({min_ds:.1f}m) or opponent slow! Full speed & min steer gain.", throttle_duration_sec=1.0)
                    else:
                        # Tailing outside no overtake zone
                        if min_ds < self._v2x_tailing_distance:
                            target_v_max_mps = min(target_v_max_mps, tail_speed_mps)
                            self.get_logger().info(f"[TAILING] Opponent '{nearest_vid}' is {min_ds:.1f}m ahead. Speed: {opp_speed_kmph:.1f} km/h. Target v_max: {target_v_max_mps*3.6:.1f} km/h.", throttle_duration_sec=1.0)
                else: # Inside no overtake zone
                    if oppo_crash_or_too_slow and min_ds < self._v2x_launch_distance:
                        # Overtake (Emergency Pass)
                        self._is_launch_mode_active = True
                        is_overtaking_active = True
                        self.get_logger().info(f"[LAUNCH MODE] No-Overtake Zone Override (s={current_s:.1f})! Opponent crashed/slow. Executing pass.", throttle_duration_sec=1.0)
                    elif min_ds < self._v2x_tailing_distance:
                        # Tailing inside no overtake zone
                        target_v_max_mps = min(target_v_max_mps, tail_speed_mps)
                        self.get_logger().info(f"[TAILING] No-Overtake Zone (s={current_s:.1f}). Tailing '{nearest_vid}' at {min_ds:.1f}m. Target v_max: {target_v_max_mps*3.6:.1f} km/h.", throttle_duration_sec=1.0)"""

content = content.replace(old_tailing, new_tailing)

old_smooth_ramp = """        # === SMOOTH RAMP DOWN OF V_MAX ===
        # If we instantly drop v_max in the MPC constraints, the solver assumes the kart is ALREADY slow
        # and doesn't calculate proper braking logic, leading to crashes. We ramp it down smoothly.
        if hasattr(self, '_current_v_max_mps'):
            current_v_max_mps = self._current_v_max_mps
        else:
            current_v_max_mps = self._mpc_cfg.v_max

        if target_v_max_mps < current_v_max_mps:
            # Ramp down
            new_v_max = max(target_v_max_mps, current_v_max_mps - 2.0 * dt)
        else:
            # Ramp up
            new_v_max = min(target_v_max_mps, current_v_max_mps + 1.0 * dt)"""

new_smooth_ramp = """        # === SMOOTH RAMP DOWN OF V_MAX ===
        # If we instantly drop v_max in the MPC constraints, the solver assumes the kart is ALREADY slow
        # and doesn't calculate proper braking logic, leading to crashes. We ramp it down smoothly.
        if hasattr(self, '_current_v_max_mps'):
            current_v_max_mps = self._current_v_max_mps
        else:
            current_v_max_mps = self._mpc_cfg.v_max

        if self._is_launch_mode_active:
            # Bypass ramp-up in launch mode to allow instant full a_max acceleration!
            new_v_max = self._mpc_cfg.v_max
        elif target_v_max_mps < current_v_max_mps:
            # Ramp down
            new_v_max = max(target_v_max_mps, current_v_max_mps - 2.0 * dt)
        else:
            # Ramp up
            new_v_max = min(target_v_max_mps, current_v_max_mps + 1.0 * dt)"""

content = content.replace(old_smooth_ramp, new_smooth_ramp)

old_steer = """        # V2X overtaking steer gain takes priority ALWAYS, even during recovery/cooldown,
        # so that overtaking corridor calculations are never disrupted by wall recovery mode.
        if self.USE_OBSTACLE_AVOIDANCE and self._v2x_avoidance_active:"""

new_steer = """        # V2X overtaking steer gain takes priority ALWAYS, even during recovery/cooldown,
        # so that overtaking corridor calculations are never disrupted by wall recovery mode.
        # NOTE: We only apply the min_overtaking_steer_gain when LAUNCH MODE is active!
        if self.USE_OBSTACLE_AVOIDANCE and self._is_launch_mode_active:"""

content = content.replace(old_steer, new_steer)

with open("/mnt/ZorinData/aichallenge-racingkart/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/multi_purpose_mpc_ros/mpc_controller.py", "w") as f:
    f.write(content)

