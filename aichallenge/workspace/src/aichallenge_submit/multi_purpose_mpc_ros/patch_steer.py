import re

with open("/mnt/ZorinData/aichallenge-racingkart/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/multi_purpose_mpc_ros/tools/steer_gain_configurator.py", "r") as f:
    content = f.read()

# Remove _current_lap
content = re.sub(r'\s*# Lap-aware gain.*?\n\s*self\._current_lap: int = 1\n', '\n', content, flags=re.DOTALL)

# Remove set_lap
set_lap_regex = r'\s*def set_lap\(self, lap: int\) -> None:.*?self\._update_marker\(\)  # rebuilds _sorted_wp_id_gain_map with new effective gains\n'
content = re.sub(set_lap_regex, '\n', content, flags=re.DOTALL)

# Simplify _get_effective_gain
old_eff = """    def _get_effective_gain(self, section_dict: dict) -> float:
        \"\"\"Return gain_warm if lap >= 2 and the field exists, else fall back to gain.\"\"\"
        if getattr(self, '_current_lap', 1) >= 2 and "gain_warm" in section_dict:
            return float(section_dict["gain_warm"])
        return float(section_dict["gain"])"""

# it might be hardcoded as `self._current_lap >= 2`
eff_regex = r'\s*def _get_effective_gain\(self, section_dict: dict\) -> float:.*?return float\(section_dict\["gain"\]\)\n'
new_eff = """
    def _get_effective_gain(self, section_dict: dict) -> float:
        return float(section_dict["gain"])
"""
content = re.sub(eff_regex, new_eff, content, flags=re.DOTALL)

with open("/mnt/ZorinData/aichallenge-racingkart/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/multi_purpose_mpc_ros/tools/steer_gain_configurator.py", "w") as f:
    f.write(content)
