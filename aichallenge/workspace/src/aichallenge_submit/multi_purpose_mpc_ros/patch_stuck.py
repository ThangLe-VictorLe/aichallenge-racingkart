import re

with open("/mnt/ZorinData/aichallenge-racingkart/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/multi_purpose_mpc_ros/mpc_controller.py", "r") as f:
    content = f.read()

old_logic = """                    # When MPC crashes, it uses leftover predictions for a few frames (which triggers the stuck timer).
                    # But once it exhausts them, it returns [0.0, 0.0]. We must keep the stuck timer running in this case!
                    mpc_exhausted = (self._mpc.infeasibility_counter > 0 and u[0] == 0.0 and u[1] == 0.0)
                    is_commanding_forward = (u[0] > 3.6 or mpc_exhausted)"""

new_logic = """                    # When MPC crashes (infeasibility), it uses leftover predictions for up to 1.25s (50 frames).
                    # During this playback, the planned velocity might dip below 3.6 km/h, which would reset the stuck timer!
                    # If the solver is failing (infeasibility_counter > 0), we must keep the stuck timer running regardless of the leftover u[0].
                    is_commanding_forward = (u[0] > 3.6 or self._mpc.infeasibility_counter > 0)"""

content = content.replace(old_logic, new_logic)

with open("/mnt/ZorinData/aichallenge-racingkart/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/multi_purpose_mpc_ros/mpc_controller.py", "w") as f:
    f.write(content)
