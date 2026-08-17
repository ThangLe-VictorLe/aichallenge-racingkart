import re
import glob

for filepath in glob.glob("/mnt/ZorinData/aichallenge-racingkart/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/multi_purpose_mpc_ros/mpc_controller*.py"):
    with open(filepath, "r") as f:
        content = f.read()

    # Remove the set_lap call block
    lap_call_regex = r'\s*# Notify SteerGainConfigurator so it switches to warm-tire gains from lap 2 onward\n\s*if getattr\(self, \'_steer_gain_configurator\', None\) is not None:\n\s*self\._steer_gain_configurator\.set_lap\(self\._current_laps\)\n'
    content = re.sub(lap_call_regex, '\n', content)
    
    # Also handle the direct check case
    lap_call_regex2 = r'\s*# Notify SteerGainConfigurator so it switches to warm-tire gains from lap 2 onward\n\s*if self\._steer_gain_configurator is not None:\n\s*self\._steer_gain_configurator\.set_lap\(self\._current_laps\)\n'
    content = re.sub(lap_call_regex2, '\n', content)

    with open(filepath, "w") as f:
        f.write(content)
