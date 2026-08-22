import re

with open('multi_purpose_mpc_ros/core/reference_path.py', 'r') as f:
    content = f.read()

# Find the start of update_path_constraints
start_idx = content.find('    def update_path_constraints(self, wp_id, pose, N, model_length, model_width, safety_margin')

# Find the end of update_path_constraints
# It's the last method, so it ends where the 'if __name__ == "__main__":' or global scope starts
end_idx = content.find('if __name__ == "__main__":', start_idx)
if end_idx == -1:
    end_idx = content.find("if __name__ == '__main__':", start_idx)

new_func = """    def update_path_constraints(self, wp_id, pose, N, model_length, model_width, safety_margin, dynamic_obstacles=None):
        \"\"\"
        Compute upper and lower bounds of the drivable area orthogonal to
        the given waypoint.
        \"\"\"
        min_segment_length = 0.1

        # container for constraints and border cells
        ub_hor = []
        lb_hor = []
        border_cells_hor = []
        border_cells_hor_sm = []

        C_prev = 0.0

        for n in range(N):
            wp = self.get_waypoint(wp_id + n)
            
            # Static bounds minus safety margin
            ub_sm = wp.ub - safety_margin
            lb_sm = wp.lb + safety_margin

            if ub_sm < lb_sm:
                ub_sm = 0.0
                lb_sm = 0.0

            if dynamic_obstacles is not None:
                for obs in dynamic_obstacles:
                    dx = obs.cx - wp.x
                    dy = obs.cy - wp.y
                    
                    # Project obstacle center onto path coordinate frame
                    s_dist = dx * np.cos(wp.psi) + dy * np.sin(wp.psi)
                    d_dist = -dx * np.sin(wp.psi) + dy * np.cos(wp.psi)
                    
                    # Effective radius including safety margin
                    R_eff = obs.radius + safety_margin
                    
                    if abs(s_dist) < R_eff:
                        w = np.sqrt(R_eff**2 - s_dist**2)
                        
                        # Two choices: pass on right (I_right) or left (I_left)
                        I_right = [lb_sm, min(ub_sm, d_dist - w)]
                        I_left = [max(lb_sm, d_dist + w), ub_sm]
                        
                        W_right = I_right[1] - I_right[0]
                        W_left = I_left[1] - I_left[0]
                        
                        valid_right = W_right >= min_segment_length
                        valid_left = W_left >= min_segment_length
                        
                        if valid_right and valid_left:
                            C_right = (I_right[0] + I_right[1]) / 2.0
                            C_left = (I_left[0] + I_left[1]) / 2.0
                            if abs(C_right - C_prev) < abs(C_left - C_prev):
                                lb_sm, ub_sm = I_right
                            else:
                                lb_sm, ub_sm = I_left
                        elif valid_right:
                            lb_sm, ub_sm = I_right
                        elif valid_left:
                            lb_sm, ub_sm = I_left
                        else:
                            # Infeasible
                            lb_sm, ub_sm = 0.0, 0.0
            
            if ub_sm < lb_sm:
                ub_sm = 0.0
                lb_sm = 0.0

            C_prev = (ub_sm + lb_sm) / 2.0

            # Compute absolute angle of bound cell
            angle_ub = np.mod(math.pi / 2 + wp.psi + math.pi, 2 * math.pi) - math.pi
            angle_lb = np.mod(-math.pi / 2 + wp.psi + math.pi, 2 * math.pi) - math.pi
            
            # Compute cell on bound for computed distance ub_sm and lb_sm
            ub_sm_ls = (wp.x + ub_sm * np.cos(angle_ub), wp.y + ub_sm * np.sin(angle_ub))
            lb_sm_ls = (wp.x - lb_sm * np.cos(angle_lb), wp.y - lb_sm * np.sin(angle_lb))
            bound_cells_sm = (ub_sm_ls, lb_sm_ls)

            # Compute actual physical boundary for reference (without safety margin)
            # Reconstruct actual bounds based on current chosen side
            # This is an approximation for visual/logging purposes
            ub = ub_sm + safety_margin
            lb = lb_sm - safety_margin
            ub_ls = (wp.x + ub * np.cos(angle_ub), wp.y + ub * np.sin(angle_ub))
            lb_ls = (wp.x - lb * np.cos(angle_lb), wp.y - lb * np.sin(angle_lb))
            bound_cells = (ub_ls, lb_ls)

            # Append results
            ub_hor.append(ub_sm)
            lb_hor.append(lb_sm)
            border_cells_hor.append(list(bound_cells))
            border_cells_hor_sm.append(list(bound_cells_sm))

            # Assign dynamic border cells to waypoints
            wp.dynamic_border_cells = bound_cells_sm
            wp.ub_sm = ub_sm
            wp.lb_sm = lb_sm

        self.set_path_constraints(
            ub_hor, lb_hor, self.n_waypoints - 1, N)
        self.set_border_cells(
            border_cells_hor_sm, border_cells_hor_sm, self.n_waypoints - 1, N)

        return np.array(ub_hor), np.array(lb_hor), border_cells_hor_sm

"""

new_content = content[:start_idx] + new_func + content[end_idx:]

with open('multi_purpose_mpc_ros/core/reference_path.py', 'w') as f:
    f.write(new_content)

