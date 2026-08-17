import sys

with open("/mnt/ZorinData/aichallenge-racingkart/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/multi_purpose_mpc_ros/core/reference_path.py", "r") as f:
    content = f.read()

# 1. Add obstacle_choices dictionary
old_vars = """        has_printed_corridor_choice = False

        # container for constraints and border cells"""

new_vars = """        has_printed_corridor_choice = False
        obstacle_choices = {} # mapping vid to chosen_side

        # container for constraints and border cells"""

content = content.replace(old_vars, new_vars)


# 2. Update corridor selection logic
old_logic = """                        chosen_side = "NONE"
                        
                        if valid_right and valid_left:
                            C_right = (I_right[0] + I_right[1]) / 2.0
                            C_left = (I_left[0] + I_left[1]) / 2.0
                            
                            # Width-biased corridor selection
                            width_bias_threshold = 0.2  # meters
                            
                            if W_left > W_right + width_bias_threshold:
                                # Left gap is significantly wider, force left
                                lb_sm, ub_sm = I_left
                                chosen_side = "LEFT"
                            elif W_right > W_left + width_bias_threshold:
                                # Right gap is significantly wider, force right
                                lb_sm, ub_sm = I_right
                                chosen_side = "RIGHT"
                            else:
                                # Gaps are similar in width, prefer the one closest to the previous center for smoothness
                                if abs(C_right - C_prev) < abs(C_left - C_prev):
                                    lb_sm, ub_sm = I_right
                                    chosen_side = "RIGHT (smooth)"
                                else:
                                    lb_sm, ub_sm = I_left
                                    chosen_side = "LEFT (smooth)"
                        elif valid_right:
                            lb_sm, ub_sm = I_right
                            chosen_side = "RIGHT (only valid)"
                        elif valid_left:
                            lb_sm, ub_sm = I_left
                            chosen_side = "LEFT (only valid)"
                        else:
                            # Infeasible
                            lb_sm, ub_sm = 0.0, 0.0
                            chosen_side = "INFEASIBLE"
                        
                        if not has_printed_corridor_choice and chosen_side != "NONE":
                            print(f"[TAILING-CORRIDOR] ego_wp: {wp_id} | wp_id: {wp_id + n} | Chosen: {chosen_side} | W_left: {W_left:.2f}m, W_right: {W_right:.2f}m", flush=True)
                            has_printed_corridor_choice = True"""

new_logic = """                        chosen_side = "NONE"
                        force_side = None
                        
                        if getattr(obs, 'vid', None) is not None:
                            force_side = obstacle_choices.get(obs.vid)

                        if force_side is not None:
                            if "LEFT" in force_side:
                                if valid_left:
                                    lb_sm, ub_sm = I_left
                                    chosen_side = force_side
                                elif valid_right:
                                    lb_sm, ub_sm = I_right
                                    chosen_side = "RIGHT (fallback)"
                                else:
                                    lb_sm, ub_sm = 0.0, 0.0
                                    chosen_side = "INFEASIBLE"
                            else: # RIGHT
                                if valid_right:
                                    lb_sm, ub_sm = I_right
                                    chosen_side = force_side
                                elif valid_left:
                                    lb_sm, ub_sm = I_left
                                    chosen_side = "LEFT (fallback)"
                                else:
                                    lb_sm, ub_sm = 0.0, 0.0
                                    chosen_side = "INFEASIBLE"
                        else:
                            if valid_right and valid_left:
                                C_right = (I_right[0] + I_right[1]) / 2.0
                                C_left = (I_left[0] + I_left[1]) / 2.0
                                
                                # Width-biased corridor selection
                                width_bias_threshold = 0.2  # meters
                                
                                if W_left > W_right + width_bias_threshold:
                                    lb_sm, ub_sm = I_left
                                    chosen_side = "LEFT"
                                elif W_right > W_left + width_bias_threshold:
                                    lb_sm, ub_sm = I_right
                                    chosen_side = "RIGHT"
                                else:
                                    if abs(C_right - C_prev) < abs(C_left - C_prev):
                                        lb_sm, ub_sm = I_right
                                        chosen_side = "RIGHT (smooth)"
                                    else:
                                        lb_sm, ub_sm = I_left
                                        chosen_side = "LEFT (smooth)"
                            elif valid_right:
                                lb_sm, ub_sm = I_right
                                chosen_side = "RIGHT (only valid)"
                            elif valid_left:
                                lb_sm, ub_sm = I_left
                                chosen_side = "LEFT (only valid)"
                            else:
                                lb_sm, ub_sm = 0.0, 0.0
                                chosen_side = "INFEASIBLE"
                                
                            if getattr(obs, 'vid', None) is not None and chosen_side not in ["NONE", "INFEASIBLE"] and "(fallback)" not in chosen_side:
                                obstacle_choices[obs.vid] = chosen_side
                        
                        if not has_printed_corridor_choice and chosen_side != "NONE":
                            print(f"[TAILING-CORRIDOR] ego_wp: {wp_id} | wp_id: {wp_id + n} | Chosen: {chosen_side} | W_left: {W_left:.2f}m, W_right: {W_right:.2f}m", flush=True)
                            has_printed_corridor_choice = True"""

if old_logic in content:
    content = content.replace(old_logic, new_logic)
else:
    print("Failed to replace logic!")
    sys.exit(1)

with open("/mnt/ZorinData/aichallenge-racingkart/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/multi_purpose_mpc_ros/core/reference_path.py", "w") as f:
    f.write(content)
