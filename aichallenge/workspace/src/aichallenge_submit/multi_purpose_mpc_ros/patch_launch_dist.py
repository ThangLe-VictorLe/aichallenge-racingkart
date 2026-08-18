with open("/mnt/ZorinData/aichallenge-racingkart/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/multi_purpose_mpc_ros/mpc_controller.py", "r") as f:
    content = f.read()

old_logic = """                opp_vx, opp_vy = self._v2x_tracker.velocity(nearest_vid)
                opp_speed_mps = np.hypot(opp_vx, opp_vy)
                opp_speed_kmph = opp_speed_mps * 3.6
                
                # Add hysteresis so it doesn't flutter on/off at the boundary
                hysteresis_dist = self._v2x_launch_distance + 4.0 if getattr(self, '_prev_launch_mode_active', False) else self._v2x_launch_distance
                launch_distance_reached = (min_ds <= hysteresis_dist)
                
                speed_delta = max(0.5, min_ds * 0.1) # m/s
                tail_speed_mps = opp_speed_mps + speed_delta"""

new_logic = """                opp_vx, opp_vy = self._v2x_tracker.velocity(nearest_vid)
                opp_speed_mps = np.hypot(opp_vx, opp_vy)
                opp_speed_kmph = opp_speed_mps * 3.6
                
                # Interpolate dynamic N between slow_n and fast_n
                import numpy as np
                launch_N_raw = np.interp(
                    opp_speed_kmph, 
                    [self._v2x_slow_oppo_vel, self._v2x_fast_oppo_vel], 
                    [self._v2x_slow_oppo_n, self._v2x_fast_oppo_n]
                )
                self._current_launch_N = int(round(launch_N_raw))
                
                # Convert the N-steps into a spatial launch distance
                dt = 1.0 / self._mpc_cfg.control_rate # 0.025s
                dynamic_launch_dist = self._current_launch_N * dt * target_v_max_mps
                
                # Add hysteresis so it doesn't flutter on/off at the boundary
                hysteresis_dist = dynamic_launch_dist + 4.0 if getattr(self, '_prev_launch_mode_active', False) else dynamic_launch_dist
                launch_distance_reached = (min_ds <= hysteresis_dist)
                
                speed_delta = max(0.5, min_ds * 0.1) # m/s
                tail_speed_mps = opp_speed_mps + speed_delta"""

content = content.replace(old_logic, new_logic)

# Make sure we initialize self._current_launch_N
content = content.replace("self._is_launch_mode_active = False\n        in_no_overtake_zone = False", "self._is_launch_mode_active = False\n        in_no_overtake_zone = False\n        self._current_launch_N = None")

with open("/mnt/ZorinData/aichallenge-racingkart/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/multi_purpose_mpc_ros/mpc_controller.py", "w") as f:
    f.write(content)
