import re

with open("/mnt/ZorinData/aichallenge-racingkart/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/multi_purpose_mpc_ros/core/reference_path.py", "r") as f:
    content = f.read()

# Add s to Waypoint.__init__
old_wp_init = """        self.psi = psi
        self.kappa = kappa

        # Reference velocity at this waypoint according to speed profile
        self.v_ref = None"""

new_wp_init = """        self.psi = psi
        self.kappa = kappa
        self.s = 0.0

        # Reference velocity at this waypoint according to speed profile
        self.v_ref = None"""

content = content.replace(old_wp_init, new_wp_init)

# Populate wp.s in ReferencePath.__init__
old_ref_init = """        # List of waypoint objects
        self.waypoints = self._construct_path(wp_x, wp_y, wp_v)

        # Number of waypoints
        self.n_waypoints = len(self.waypoints)

        # Length of path
        self.length, self.segment_lengths = self._compute_length()"""

new_ref_init = """        # List of waypoint objects
        self.waypoints = self._construct_path(wp_x, wp_y, wp_v)

        # Number of waypoints
        self.n_waypoints = len(self.waypoints)

        # Length of path
        self.length, self.segment_lengths = self._compute_length()
        
        # Populate Frenet s for each waypoint
        current_s = 0.0
        for i, wp in enumerate(self.waypoints):
            wp.s = current_s
            if i < len(self.segment_lengths) - 1:
                current_s += self.segment_lengths[i+1]"""

content = content.replace(old_ref_init, new_ref_init)

# Use wp.s and obs.s to filter ghost obstacles in update_path_constraints
old_constraints = """            if dynamic_obstacles is not None:
                for obs in dynamic_obstacles:
                    dx = obs.cx - wp.x
                    dy = obs.cy - wp.y
                    
                    # Project obstacle center onto path coordinate frame
                    s_dist = dx * np.cos(wp.psi) + dy * np.sin(wp.psi)
                    d_dist = -dx * np.sin(wp.psi) + dy * np.cos(wp.psi)
                    
                    # Effective radius including safety margin
                    R_eff = obs.radius + safety_margin
                    
                    if abs(s_dist) < R_eff:"""

new_constraints = """            if dynamic_obstacles is not None:
                for obs in dynamic_obstacles:
                    dx = obs.cx - wp.x
                    dy = obs.cy - wp.y
                    
                    # Project obstacle center onto path coordinate frame
                    s_dist_proj = dx * np.cos(wp.psi) + dy * np.sin(wp.psi)
                    d_dist = -dx * np.sin(wp.psi) + dy * np.cos(wp.psi)
                    
                    # Effective radius including safety margin
                    R_eff = obs.radius + safety_margin
                    
                    # Calculate true along-track distance to prevent ghost obstacles on sharp curves
                    if hasattr(wp, 's') and hasattr(obs, 's') and obs.s is not None:
                        s_dist = wp.s - obs.s
                        # Handle circular track wrap-around
                        if self.circular:
                            s_dist = (s_dist + self.length / 2) % self.length - self.length / 2
                    else:
                        s_dist = s_dist_proj
                    
                    if abs(s_dist) < R_eff:"""

content = content.replace(old_constraints, new_constraints)

with open("/mnt/ZorinData/aichallenge-racingkart/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/multi_purpose_mpc_ros/core/reference_path.py", "w") as f:
    f.write(content)
