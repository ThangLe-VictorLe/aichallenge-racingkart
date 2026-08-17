import re

with open("/mnt/ZorinData/aichallenge-racingkart/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/multi_purpose_mpc_ros/mpc_controller.py", "r") as f:
    content = f.read()

# Remove the 20m lookahead block and restore strict zone check
lookahead_block = """            # 20m Lookahead: If we are in a zone, OR will enter one within 20m, we are 'in_no_overtake_zone'.
            # This ensures we don't start an overtake right before a corner, which caused the kart
            # to enter overtaking mode, then immediately disable it mid-corner (blinding itself to the opponent).
            lookahead_s = current_s + 20.0
            
            for zone in getattr(self, '_no_overtake_zones', []):
                if len(zone) == 2:
                    z_start, z_end = zone[0], zone[1]
                    # Overlap happens if max(start1, start2) <= min(end1, end2)
                    if max(current_s, z_start) <= min(lookahead_s, z_end):
                        in_no_overtake_zone = True
                        break"""

strict_zone_block = """            for zone in getattr(self, '_no_overtake_zones', []):
                if len(zone) == 2 and zone[0] <= current_s <= zone[1]:
                    in_no_overtake_zone = True
                    break"""

content = content.replace(lookahead_block, strict_zone_block)

# Fix the oppo_crash_or_too_slow logic to use 2.0 km/h inside no-overtake zones to prevent false positives in slow corners.
old_tailing_logic = """                launch_distance_reached = (min_ds <= self._v2x_launch_distance)
                oppo_crash_or_too_slow = (opp_speed_kmph < self._v2x_ignore_tailing_vel)
                
                speed_delta = max(0.5, min_ds * 0.1) # m/s
                tail_speed_mps = opp_speed_mps + speed_delta
                
                if not in_no_overtake_zone:
                    if launch_distance_reached or oppo_crash_or_too_slow:"""

new_tailing_logic = """                launch_distance_reached = (min_ds <= self._v2x_launch_distance)
                
                speed_delta = max(0.5, min_ds * 0.1) # m/s
                tail_speed_mps = opp_speed_mps + speed_delta
                
                if not in_no_overtake_zone:
                    oppo_too_slow_outside = (opp_speed_kmph < self._v2x_ignore_tailing_vel)
                    if launch_distance_reached or oppo_too_slow_outside:"""

content = content.replace(old_tailing_logic, new_tailing_logic)

old_tailing_logic_2 = """                else: # Inside no overtake zone
                    if oppo_crash_or_too_slow and min_ds < self._v2x_launch_distance:"""

new_tailing_logic_2 = """                else: # Inside no overtake zone
                    # Inside the zone, ONLY trigger emergency pass if opponent is practically crashed (< 2.0 km/h).
                    # Using ignore_tailing_vel (11 km/h) here causes false overtakes because karts naturally slow down in corners!
                    oppo_crashed_inside = (opp_speed_kmph < 2.0)
                    if oppo_crashed_inside and min_ds < self._v2x_launch_distance:"""

content = content.replace(old_tailing_logic_2, new_tailing_logic_2)

with open("/mnt/ZorinData/aichallenge-racingkart/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/multi_purpose_mpc_ros/mpc_controller.py", "w") as f:
    f.write(content)
