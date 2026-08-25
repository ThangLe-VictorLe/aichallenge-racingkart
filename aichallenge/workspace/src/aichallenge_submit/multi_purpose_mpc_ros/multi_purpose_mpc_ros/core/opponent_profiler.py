import math
import numpy as np
from typing import Dict, List, Optional
import rclpy
from rclpy.node import Node

class OpponentProfiler:
    """
    Records and analyzes opponent trajectories over the track to predict their 
    lateral position (ey) and categorize their racing strategy at specific corners.
    """
    
    def __init__(self, track_waypoints, logger=None):
        """
        track_waypoints: List of Waypoint objects from the reference path.
        """
        self._wps = track_waypoints
        self._logger = logger
        
        # Mapping from vid -> wp_id -> expected_ey
        self._expected_ey: Dict[str, Dict[int, float]] = {}
        
        # We no longer have a strict lap-based learning mode
        # The EWMA alpha determines how fast we adapt to a new line
        self._alpha = 0.2

    def record_opponent_at_wp(self, opp_x: float, opp_y: float, opp_vid: str, nearest_wp_id: int):
        """
        Records the opponent's position relative to a specific waypoint using EWMA.
        """
        wp = self._wps[nearest_wp_id]
        
        # Calculate lateral deviation (ey)
        dx = opp_x - wp.x
        dy = opp_y - wp.y
        
        # Cross product of track forward vector and vector to opponent
        # Track forward vector: (cos(psi), sin(psi))
        # Cross product: cos(psi)*dy - sin(psi)*dx
        # Positive ey means opponent is to the LEFT of the raceline. Negative means RIGHT.
        ey = math.cos(wp.psi) * dy - math.sin(wp.psi) * dx
        
        if opp_vid not in self._expected_ey:
            self._expected_ey[opp_vid] = {}
            
        if nearest_wp_id not in self._expected_ey[opp_vid]:
            # First time seeing them here, initialize
            self._expected_ey[opp_vid][nearest_wp_id] = ey
        else:
            # EWMA update
            current_expected = self._expected_ey[opp_vid][nearest_wp_id]
            self._expected_ey[opp_vid][nearest_wp_id] = self._alpha * ey + (1 - self._alpha) * current_expected
            
        # Logging throttle
        if not hasattr(self, '_last_logged_wp'):
            self._last_logged_wp = {}
            
        last_logged = self._last_logged_wp.get(opp_vid, -100)
        if abs(nearest_wp_id - last_logged) >= 10:
            if self._logger:
                side = "LEFT" if self._expected_ey[opp_vid][nearest_wp_id] > 0 else "RIGHT"
                self._logger.info(f"[PROFILER] Recorded {opp_vid} at wp {nearest_wp_id}. expected_ey = {abs(self._expected_ey[opp_vid][nearest_wp_id]):.2f}m to the {side}.")
            self._last_logged_wp[opp_vid] = nearest_wp_id

    def get_expected_ey(self, opp_vid: str, wp_id: int) -> float:
        """
        Returns the expected lateral deviation of the opponent at the given waypoint.
        Returns 0.0 if unknown.
        """
        if opp_vid in self._expected_ey:
            return self._expected_ey[opp_vid].get(wp_id, 0.0)
        return 0.0
        
    def get_strategy_for_corner(self, opp_vid: str, future_wp_id: int) -> str:
        """
        Analyzes the opponent's expected ey at a future waypoint and returns a strategy.
        Returns: "LEFT", "RIGHT", or "NORMAL"
        """
        # If we haven't seen this opponent at this waypoint, assume NORMAL
        if opp_vid not in self._expected_ey or future_wp_id not in self._expected_ey[opp_vid]:
            return "NORMAL"
            
        ey = self.get_expected_ey(opp_vid, future_wp_id)
        wp = self._wps[future_wp_id]
        
        # They are far left, inside/outside is open on the right
        if ey > 1.0: 
            return "RIGHT"
            
        # They are far right, inside/outside is open on the left
        if ey < -1.0:
            return "LEFT"
            
        # They are blocking center, force a cut-under on the inside of the corner
        if abs(ey) < 0.5:
            # Positive kappa means track turns left (inside is LEFT)
            # Negative kappa means track turns right (inside is RIGHT)
            if wp.kappa > 0.005:
                return "LEFT"
            elif wp.kappa < -0.005:
                return "RIGHT"
            
        return "NORMAL"
