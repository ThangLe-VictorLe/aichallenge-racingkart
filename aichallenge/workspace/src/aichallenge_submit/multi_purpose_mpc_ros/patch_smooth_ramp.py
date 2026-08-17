import re

with open("/mnt/ZorinData/aichallenge-racingkart/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/multi_purpose_mpc_ros/mpc_controller.py", "r") as f:
    content = f.read()

old_ramp = """        if target_v_max_mps < effective_start_v_max:
            new_v_max_mps = max(target_v_max_mps, effective_start_v_max - max_decel_step_mps)
        else:
            new_v_max_mps = target_v_max_mps"""

new_ramp = """        if getattr(self, '_is_launch_mode_active', False):
            new_v_max_mps = self._mpc_cfg.v_max
        elif target_v_max_mps < effective_start_v_max:
            new_v_max_mps = max(target_v_max_mps, effective_start_v_max - max_decel_step_mps)
        else:
            new_v_max_mps = target_v_max_mps"""

content = content.replace(old_ramp, new_ramp)

with open("/mnt/ZorinData/aichallenge-racingkart/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/multi_purpose_mpc_ros/mpc_controller.py", "w") as f:
    f.write(content)
