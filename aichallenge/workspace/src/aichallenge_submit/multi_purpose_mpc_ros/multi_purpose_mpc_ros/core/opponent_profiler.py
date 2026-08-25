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
        
        # Mapping from wp_id to a list of recorded lateral deviations (ey)
        self._ey_history: Dict[int, List[float]] = {i: [] for i in range(len(self._wps))}
        self._expected_ey: Dict[int, float] = {}
        
        # State tracking
        self._learning_mode = True
        self._ego_laps = 0
        self._last_ego_wp = 0

    def update_ego_progress(self, ego_wp_id: int, total_wps: int):
        """
        Detects lap completion based on ego kart's progress.
        """
        # Simple lap detection: if we were near the end and now near the beginning
        if self._last_ego_wp > total_wps * 0.8 and ego_wp_id < total_wps * 0.2:
            self._ego_laps += 1
            if self._ego_laps >= 1 and self._learning_mode:
                self._learning_mode = False
                self._compute_expected_ey()
                if self._logger:
                    self._logger.info(f"[PROFILER] Lap {self._ego_laps} complete. Switching to Active Prediction Mode.")
                    self._logger.info(f"[PROFILER] Generated predictive map across {total_wps} waypoints based on opponent's historical line.")
            elif self._logger:
                self._logger.info(f"[PROFILER] Lap {self._ego_laps} complete. Continuing with Active Prediction Mode.")
        
        self._last_ego_wp = ego_wp_id

    def record_opponent_position(self, opp_x: float, opp_y: float, opp_vid: str):
        """
        Projects opponent position onto the raceline and records their lateral deviation (ey).
        For simplicity, we track the main opponent (e.g., 'd2').
        """
        if not self._learning_mode:
            return # We only learn on lap 1 (or we could continuously learn, but let's stick to the plan)
            
        if opp_vid == 'd3':
            return # Ignore NPC
            
        # 1. Find nearest waypoint (basic linear search or we can assume it's passed efficiently)
        # To avoid heavy computation if called frequently, we rely on the controller to pass the nearest wp_id
        pass

    def record_opponent_at_wp(self, opp_x: float, opp_y: float, opp_vid: str, nearest_wp_id: int):
        """
        Records the opponent's position relative to a specific waypoint.
        """
        if not self._learning_mode:
            return
            
        if opp_vid == 'd3':
            return
            
        wp = self._wps[nearest_wp_id]
        
        # Calculate lateral deviation (ey)
        dx = opp_x - wp.x
        dy = opp_y - wp.y
        
        # Cross product of track forward vector and vector to opponent
        # Track forward vector: (cos(psi), sin(psi))
        # Cross product: cos(psi)*dy - sin(psi)*dx
        # Positive ey means opponent is to the LEFT of the raceline. Negative means RIGHT.
        ey = math.cos(wp.psi) * dy - math.sin(wp.psi) * dx
        
        if not hasattr(self, '_last_logged_wp') or abs(nearest_wp_id - getattr(self, '_last_logged_wp', 0)) >= 10:
            if self._logger:
                side = "LEFT" if ey > 0 else "RIGHT"
                self._logger.info(f"[PROFILER] Learning mode: Recorded {opp_vid} at wp {nearest_wp_id}. Lateral deviation (ey) = {abs(ey):.2f}m to the {side}.")
            self._last_logged_wp = nearest_wp_id

        self._ey_history[nearest_wp_id].append(ey)

    def _compute_expected_ey(self):
        """
        Averages the recorded ey values to create a predictive map.
        """
        for wp_id, eys in self._ey_history.items():
            if eys:
                self._expected_ey[wp_id] = sum(eys) / len(eys)
            else:
                self._expected_ey[wp_id] = 0.0

    def get_expected_ey(self, wp_id: int) -> float:
        """
        Returns the expected lateral deviation of the opponent at the given waypoint.
        Returns 0.0 if unknown.
        """
        return self._expected_ey.get(wp_id, 0.0)
        
    def get_strategy_for_corner(self, future_wp_id: int) -> str:
        """
        Analyzes the opponent's expected ey at a future waypoint and returns a strategy.
        Returns: "LEFT", "RIGHT", or "NORMAL"
        """
        if self._learning_mode:
            return "NORMAL"
            
        ey = self.get_expected_ey(future_wp_id)
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
