---
trigger: always_on
---

-main control method: multi_purpose_mpc_ros

Stucture:
-change accel/brake/steer: aichallenge/workspace/src/aichallenge_submit/aichallenge_submit_launch/data
-mpc config: aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/config/config.yaml
-mpc driving map: aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/env/final_ver3
-traj generator: aichallenge/workspace/src/aichallenge_submit/simple_trajectory_generator/data
-rviz panel: aichallenge/workspace/src/aichallenge_system/autoware_overlay_rviz_plugin