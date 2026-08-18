with open("/mnt/ZorinData/aichallenge-racingkart/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/multi_purpose_mpc_ros/core/reference_path.py", "r") as f:
    content = f.read()

content = content.replace(
    "def update_path_constraints(self, wp_id, pose, N, model_length, model_width, safety_margin, dynamic_obstacles=None):",
    "def update_path_constraints(self, wp_id, pose, N, model_length, model_width, safety_margin, dynamic_obstacles=None, launch_N=None):"
)

old_loop_start = """        for n in range(N):
            wp = self.get_waypoint(wp_id + n)
            
            # Static bounds minus safety margin
            ub_sm = wp.ub - safety_margin
            lb_sm = wp.lb + safety_margin
            
            W_left = 0.0
            W_right = 0.0
            chosen_side = "NONE"
            valid_left = False
            valid_right = False"""

new_loop_start = """        for n in range(N):
            wp = self.get_waypoint(wp_id + n)
            
            # Static bounds minus safety margin
            ub_sm = wp.ub - safety_margin
            lb_sm = wp.lb + safety_margin
            
            # If launch_N is specified, ignore dynamic obstacles for steps >= launch_N
            ignore_dynamic = False
            if launch_N is not None and n >= launch_N:
                ignore_dynamic = True
            
            W_left = 0.0
            W_right = 0.0
            chosen_side = "NONE"
            valid_left = False
            valid_right = False"""

content = content.replace(old_loop_start, new_loop_start)

old_check = """            if dynamic_obstacles is not None and len(dynamic_obstacles) > 0:"""
new_check = """            if dynamic_obstacles is not None and len(dynamic_obstacles) > 0 and not ignore_dynamic:"""
content = content.replace(old_check, new_check)

with open("/mnt/ZorinData/aichallenge-racingkart/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/multi_purpose_mpc_ros/core/reference_path.py", "w") as f:
    f.write(content)
