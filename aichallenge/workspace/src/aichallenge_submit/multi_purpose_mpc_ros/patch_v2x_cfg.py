with open("/mnt/ZorinData/aichallenge-racingkart/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/multi_purpose_mpc_ros/mpc_controller.py", "r") as f:
    content = f.read()

old_cfg = """            # Tailing & Overtaking Config
            self._v2x_tailing_distance = float(getattr(v2x_cfg, 'tailing_distance_m', 15.0))
            self._v2x_launch_distance = float(getattr(v2x_cfg, 'launch_distance_m', 8.0))
            self._v2x_ignore_tailing_vel = float(getattr(v2x_cfg, 'ignore_tailing_vel_kmh', 10.0))
            self._no_overtake_zones = getattr(v2x_cfg, 'no_overtake_zones', [])"""

new_cfg = """            # Tailing & Overtaking Config
            self._v2x_tailing_distance = float(getattr(v2x_cfg, 'tailing_distance_m', 15.0))
            self._v2x_ignore_tailing_vel = float(getattr(v2x_cfg, 'ignore_tailing_vel_kmh', 10.0))
            self._no_overtake_zones = getattr(v2x_cfg, 'no_overtake_zones', [])
            
            # Dynamic Launch & N-Step
            dyn_launch = getattr(v2x_cfg, 'dynamic_launch', {})
            self._v2x_fast_oppo_vel = float(dyn_launch.get('fast_oppo_vel_kmh', 26.0))
            self._v2x_fast_oppo_n = float(dyn_launch.get('fast_oppo_n', 20))
            self._v2x_slow_oppo_vel = float(dyn_launch.get('slow_oppo_vel_kmh', 5.0))
            self._v2x_slow_oppo_n = float(dyn_launch.get('slow_oppo_n', 50))"""

content = content.replace(old_cfg, new_cfg)

with open("/mnt/ZorinData/aichallenge-racingkart/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/multi_purpose_mpc_ros/mpc_controller.py", "w") as f:
    f.write(content)
