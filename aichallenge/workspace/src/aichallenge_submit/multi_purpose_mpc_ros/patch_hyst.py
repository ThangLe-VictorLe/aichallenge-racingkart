import re

with open("/mnt/ZorinData/aichallenge-racingkart/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/multi_purpose_mpc_ros/mpc_controller.py", "r") as f:
    content = f.read()

# Fix 1: Add hysteresis to launch_distance_reached
old_launch_cond = """                launch_distance_reached = (min_ds <= self._v2x_launch_distance)"""
new_launch_cond = """                # Add hysteresis so it doesn't flutter on/off at the boundary
                hysteresis_dist = self._v2x_launch_distance + 4.0 if getattr(self, '_prev_launch_mode_active', False) else self._v2x_launch_distance
                launch_distance_reached = (min_ds <= hysteresis_dist)"""

content = content.replace(old_launch_cond, new_launch_cond)


# Fix 2: Add throttle to the finish log
old_finish_log = """        if getattr(self, '_prev_launch_mode_active', False) and not self._is_launch_mode_active:
            self.get_logger().info("[LAUNCH MODE] Overtaking maneuver finished. Returning to normal steer gain.")
        self._prev_launch_mode_active = self._is_launch_mode_active"""

new_finish_log = """        if getattr(self, '_prev_launch_mode_active', False) and not self._is_launch_mode_active:
            self.get_logger().info("[LAUNCH MODE] Overtaking maneuver finished. Returning to normal steer gain.", throttle_duration_sec=2.0)
        self._prev_launch_mode_active = self._is_launch_mode_active"""

content = content.replace(old_finish_log, new_finish_log)

with open("/mnt/ZorinData/aichallenge-racingkart/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/multi_purpose_mpc_ros/mpc_controller.py", "w") as f:
    f.write(content)
