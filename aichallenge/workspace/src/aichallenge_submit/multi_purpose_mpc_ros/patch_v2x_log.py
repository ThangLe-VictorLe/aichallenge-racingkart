import re

with open("/mnt/ZorinData/aichallenge-racingkart/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/multi_purpose_mpc_ros/mpc_controller.py", "r") as f:
    content = f.read()

old_log = """            opp_s = self._waypoint_s[nearest_wp]
            ds = (opp_s - ego_s) % track_len
            dist_to_ego = np.hypot(opp_x - ego_x, opp_y - ego_y)"""

new_log = """            opp_s = self._waypoint_s[nearest_wp]
            ds = (opp_s - ego_s) % track_len
            ego_x = float(self._odom.pose.pose.position.x) if self._odom is not None else 0.0
            ego_y = float(self._odom.pose.pose.position.y) if self._odom is not None else 0.0
            dist_to_ego = np.hypot(opp_x - ego_x, opp_y - ego_y)"""

content = content.replace(old_log, new_log)

with open("/mnt/ZorinData/aichallenge-racingkart/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/multi_purpose_mpc_ros/mpc_controller.py", "w") as f:
    f.write(content)
