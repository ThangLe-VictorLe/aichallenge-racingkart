import re

with open("/mnt/ZorinData/aichallenge-racingkart/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/multi_purpose_mpc_ros/mpc_controller.py", "r") as f:
    content = f.read()

old_log_start = """    def _v2x_diagnostic_log(self, msg, ego_wp_id, ego_s, track_len,
                             activate_threshold, deactivate_threshold) -> None:
        \"\"\"Throttled terminal log: prints V2X vehicle positions/distances and gate state.\"\"\"
        self._v2x_dbg_counter += 1
        if self._v2x_dbg_counter % 10 != 1:
            return"""

new_log_start = """    def _v2x_diagnostic_log(self, msg, ego_wp_id, ego_s, track_len,
                             activate_threshold, deactivate_threshold) -> None:
        \"\"\"Throttled terminal log: prints V2X vehicle positions/distances and gate state.\"\"\"
        if getattr(self, '_recovery_state', "NORMAL") != "NORMAL":
            return
            
        self._v2x_dbg_counter += 1
        if self._v2x_dbg_counter % 10 != 1:
            return"""

content = content.replace(old_log_start, new_log_start)

with open("/mnt/ZorinData/aichallenge-racingkart/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/multi_purpose_mpc_ros/mpc_controller.py", "w") as f:
    f.write(content)
