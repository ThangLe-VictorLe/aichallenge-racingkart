import re

with open("/mnt/ZorinData/aichallenge-racingkart/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/multi_purpose_mpc_ros/mpc_controller.py", "r") as f:
    content = f.read()

old_log = """            ds = (opp_s - ego_s) % track_len
            is_ego = nearest_dist < 0.8
            lines.append(
                f"  vid={v.vehicle_id}  pos=({opp_x:.1f},{opp_y:.1f})"
                f"  wp={nearest_wp}  dist_to_wp={nearest_dist:.2f}m"
                f"  ds={ds:.1f}m ({'BEHIND' if ds > track_len / 2 else 'AHEAD'})"
                + ("  [EGO SKIP]" if is_ego else "")
            )"""

new_log = """            ds = (opp_s - ego_s) % track_len
            dist_to_ego = np.hypot(opp_x - ego_x, opp_y - ego_y)
            is_ego = (dist_to_ego < 0.5)
            lines.append(
                f"  vid={v.vehicle_id}  pos=({opp_x:.1f},{opp_y:.1f})"
                f"  wp={nearest_wp}  dist_to_wp={nearest_dist:.2f}m"
                f"  ds={ds:.1f}m ({'BEHIND' if ds > track_len / 2 else 'AHEAD'})"
                + ("  [EGO SKIP]" if is_ego else "")
            )"""

content = content.replace(old_log, new_log)

with open("/mnt/ZorinData/aichallenge-racingkart/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/multi_purpose_mpc_ros/mpc_controller.py", "w") as f:
    f.write(content)
