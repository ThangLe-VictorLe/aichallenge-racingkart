import re

# 1. Patch reference_path.py
with open("/mnt/ZorinData/aichallenge-racingkart/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/multi_purpose_mpc_ros/core/reference_path.py", "r") as f:
    ref_content = f.read()

old_ref_log = """                        if not has_printed_corridor_choice and chosen_side != "NONE":
                            print(f"[TAILING-CORRIDOR] wp_id: {wp_id + n} | Chosen: {chosen_side} | W_left: {W_left:.2f}m, W_right: {W_right:.2f}m", flush=True)"""

new_ref_log = """                        if not has_printed_corridor_choice and chosen_side != "NONE":
                            print(f"[TAILING-CORRIDOR] ego_wp: {wp_id} | wp_id: {wp_id + n} | Chosen: {chosen_side} | W_left: {W_left:.2f}m, W_right: {W_right:.2f}m", flush=True)"""

ref_content = ref_content.replace(old_ref_log, new_ref_log)

with open("/mnt/ZorinData/aichallenge-racingkart/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/multi_purpose_mpc_ros/core/reference_path.py", "w") as f:
    f.write(ref_content)

# 2. Patch mpc_controller.py
with open("/mnt/ZorinData/aichallenge-racingkart/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/multi_purpose_mpc_ros/mpc_controller.py", "r") as f:
    mpc_content = f.read()

# Add _prev_launch_mode_active to __init__
old_init = """            self._v2x_avoidance_active = False
            self._is_launch_mode_active = False"""

new_init = """            self._v2x_avoidance_active = False
            self._is_launch_mode_active = False
            self._prev_launch_mode_active = False"""

mpc_content = mpc_content.replace(old_init, new_init)

# Add logging when launch mode deactivates
old_tailing_end = """                        self.get_logger().info(f"[TAILING] No-Overtake Zone (s={current_s:.1f}). Tailing '{nearest_vid}' at {min_ds:.1f}m. Target v_max: {target_v_max_mps*3.6:.1f} km/h.", throttle_duration_sec=1.0)

        # === SMOOTH RAMP DOWN OF V_MAX ==="""

new_tailing_end = """                        self.get_logger().info(f"[TAILING] No-Overtake Zone (s={current_s:.1f}). Tailing '{nearest_vid}' at {min_ds:.1f}m. Target v_max: {target_v_max_mps*3.6:.1f} km/h.", throttle_duration_sec=1.0)

        if getattr(self, '_prev_launch_mode_active', False) and not self._is_launch_mode_active:
            self.get_logger().info("[LAUNCH MODE] Overtaking maneuver finished. Returning to normal steer gain.")
        self._prev_launch_mode_active = self._is_launch_mode_active

        # === SMOOTH RAMP DOWN OF V_MAX ==="""

mpc_content = mpc_content.replace(old_tailing_end, new_tailing_end)

with open("/mnt/ZorinData/aichallenge-racingkart/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/multi_purpose_mpc_ros/mpc_controller.py", "w") as f:
    f.write(mpc_content)

