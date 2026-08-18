with open("/mnt/ZorinData/aichallenge-racingkart/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/multi_purpose_mpc_ros/mpc_controller.py", "r") as f:
    content = f.read()

old_logic = """        # === MPC SOLVER ===
        with self._stats.time_block("control"):
            dynamic_obs = self._dynamic_obstacles if (self.USE_OBSTACLE_AVOIDANCE and is_overtaking_active) else []
            u, max_delta = self._mpc.get_control(dynamic_obstacles=dynamic_obs)"""

new_logic = """        # === MPC SOLVER ===
        with self._stats.time_block("control"):
            dynamic_obs = self._dynamic_obstacles if (self.USE_OBSTACLE_AVOIDANCE and is_overtaking_active) else []
            launch_n_arg = getattr(self, '_current_launch_N', None) if is_overtaking_active else None
            u, max_delta = self._mpc.get_control(dynamic_obstacles=dynamic_obs, launch_N=launch_n_arg)"""

content = content.replace(old_logic, new_logic)

with open("/mnt/ZorinData/aichallenge-racingkart/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/multi_purpose_mpc_ros/mpc_controller.py", "w") as f:
    f.write(content)
