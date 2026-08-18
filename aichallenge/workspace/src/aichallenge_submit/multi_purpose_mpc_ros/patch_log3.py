with open("/mnt/ZorinData/aichallenge-racingkart/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/multi_purpose_mpc_ros/mpc_controller.py", "r") as f:
    content = f.read()

old_log = \"\"\"                self.get_logger().warn(
                    "[V2X] Overtaking mode ACTIVATED — opponent within "
                    f"{activate_threshold:.0f} m ahead. Min steer gain -> {self._min_overtaking_steer_gain}")\"\"\"

new_log = \"\"\"                self.get_logger().warn(
                    "[V2X] Overtaking mode ACTIVATED — opponent within "
                    f"{activate_threshold:.0f} m ahead.")\"\"\"

content = content.replace(old_log, new_log)

with open("/mnt/ZorinData/aichallenge-racingkart/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/multi_purpose_mpc_ros/mpc_controller.py", "w") as f:
    f.write(content)
