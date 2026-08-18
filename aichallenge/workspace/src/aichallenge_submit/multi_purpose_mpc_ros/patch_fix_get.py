with open("/mnt/ZorinData/aichallenge-racingkart/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/multi_purpose_mpc_ros/mpc_controller.py", "r") as f:
    content = f.read()

old_logic = """            # Dynamic Launch & N-Step
            dyn_launch = getattr(v2x_cfg, 'dynamic_launch', {})
            self._v2x_fast_oppo_vel = float(dyn_launch.get('fast_oppo_vel_kmh', 26.0))
            self._v2x_fast_oppo_n = float(dyn_launch.get('fast_oppo_n', 20))
            self._v2x_slow_oppo_vel = float(dyn_launch.get('slow_oppo_vel_kmh', 5.0))
            self._v2x_slow_oppo_n = float(dyn_launch.get('slow_oppo_n', 50))"""

new_logic = """            # Dynamic Launch & N-Step
            dyn_launch = getattr(v2x_cfg, 'dynamic_launch', None)
            if dyn_launch is not None:
                self._v2x_fast_oppo_vel = float(getattr(dyn_launch, 'fast_oppo_vel_kmh', 26.0))
                self._v2x_fast_oppo_n = float(getattr(dyn_launch, 'fast_oppo_n', 20))
                self._v2x_slow_oppo_vel = float(getattr(dyn_launch, 'slow_oppo_vel_kmh', 5.0))
                self._v2x_slow_oppo_n = float(getattr(dyn_launch, 'slow_oppo_n', 50))
            else:
                self._v2x_fast_oppo_vel = 26.0
                self._v2x_fast_oppo_n = 20
                self._v2x_slow_oppo_vel = 5.0
                self._v2x_slow_oppo_n = 50"""

content = content.replace(old_logic, new_logic)

with open("/mnt/ZorinData/aichallenge-racingkart/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/multi_purpose_mpc_ros/mpc_controller.py", "w") as f:
    f.write(content)
