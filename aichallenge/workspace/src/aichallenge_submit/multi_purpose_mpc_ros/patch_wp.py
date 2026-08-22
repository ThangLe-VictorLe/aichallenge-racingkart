with open("/mnt/ZorinData/aichallenge-racingkart/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/multi_purpose_mpc_ros/mpc_controller.py", "r") as f:
    content = f.read()

old_logic = """        ego_wp_id = self._mpc.model.wp_id if self._mpc is not None else 0
        ego_s = getattr(self, '_waypoint_s', np.array([]))
        if ego_s.size > 0:
            current_s = ego_s[min(ego_wp_id, len(ego_s) - 1)]
            for zone in getattr(self, '_no_overtake_zones', []):
                if len(zone) == 2 and zone[0] <= current_s <= zone[1]:
                    in_no_overtake_zone = True
                    break"""

new_logic = """        ego_wp_id = self._mpc.model.wp_id if self._mpc is not None else 0
        for zone in getattr(self, '_no_overtake_zones', []):
            if len(zone) == 2 and zone[0] <= ego_wp_id <= zone[1]:
                in_no_overtake_zone = True
                break"""

content = content.replace(old_logic, new_logic)

# Replace the logs that use current_s
content = content.replace("No-Overtake Zone Override (s={current_s:.1f})!", "No-Overtake Zone Override (wp={ego_wp_id})!")
content = content.replace("No-Overtake Zone (s={current_s:.1f}).", "No-Overtake Zone (wp={ego_wp_id}).")

with open("/mnt/ZorinData/aichallenge-racingkart/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/multi_purpose_mpc_ros/mpc_controller.py", "w") as f:
    f.write(content)

