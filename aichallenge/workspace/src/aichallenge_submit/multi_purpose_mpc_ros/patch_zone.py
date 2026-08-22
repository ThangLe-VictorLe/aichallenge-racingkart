import re

with open("/mnt/ZorinData/aichallenge-racingkart/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/multi_purpose_mpc_ros/mpc_controller.py", "r") as f:
    content = f.read()

old_zone = """        ego_wp_id = self._mpc.model.wp_id if self._mpc is not None else 0
        ego_s = getattr(self, '_waypoint_s', np.array([]))
        if ego_s.size > 0:
            current_s = ego_s[min(ego_wp_id, len(ego_s) - 1)]
            for zone in getattr(self, '_no_overtake_zones', []):
                if len(zone) == 2 and zone[0] <= current_s <= zone[1]:
                    in_no_overtake_zone = True
                    break"""

new_zone = """        ego_wp_id = self._mpc.model.wp_id if self._mpc is not None else 0
        ego_s = getattr(self, '_waypoint_s', np.array([]))
        if ego_s.size > 0:
            current_s = ego_s[min(ego_wp_id, len(ego_s) - 1)]
            # Check if we are currently in a zone, OR if we will enter one very soon (20m lookahead).
            # This prevents us from starting an overtake right before a dangerous corner!
            lookahead_s = current_s + 20.0
            
            for zone in getattr(self, '_no_overtake_zones', []):
                if len(zone) == 2:
                    z_start, z_end = zone[0], zone[1]
                    # Overlap condition: max(current_s, z_start) <= min(lookahead_s, z_end)
                    if max(current_s, z_start) <= min(lookahead_s, z_end):
                        in_no_overtake_zone = True
                        break
            
            # If we were already committed to an overtake and are alongside the opponent, DO NOT ABORT!
            # Aborting removes the obstacle and causes us to steer into them.
            if in_no_overtake_zone and getattr(self, '_is_launch_mode_active', False):
                nearest_vid = getattr(self, '_nearest_opponent_vid', None)
                min_ds = getattr(self, '_nearest_opponent_ds', float('inf'))
                if min_ds < 3.0: # Extremely close or alongside
                    in_no_overtake_zone = False # Ignore the zone to finish the pass safely"""

content = content.replace(old_zone, new_zone)

with open("/mnt/ZorinData/aichallenge-racingkart/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/multi_purpose_mpc_ros/mpc_controller.py", "w") as f:
    f.write(content)
