"""Per-vehicle finite-difference velocity tracker for V2X positions.

This module is intentionally pure Python with no rclpy dependency: it
operates on duck-typed messages whose attributes match
``v2x_msgs/V2XVehiclePositionArray``. That keeps it cheap to unit-test
and reusable from non-ROS contexts (e.g. offline replay of rosbag CSVs).
"""

import math
from collections import deque
from typing import Deque, Dict, List, Tuple


def _stamp_to_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class V2XVehicleTracker:
    """Tracks the latest two samples per ``vehicle_id`` and exposes
    constant-velocity predictions over a caller-provided time grid."""

    def __init__(self, v_max_safety: float, position_jump_threshold: float, warn_callback=None):
        self._v_max_safety = float(v_max_safety)
        self._jump_thresh = float(position_jump_threshold)
        self._warn = warn_callback if warn_callback is not None else (lambda _msg: None)
        self._samples: Dict[str, Deque[Tuple[float, float, float]]] = {}
        self._velocities: Dict[str, Tuple[float, float]] = {}
        self._accelerations: Dict[str, Tuple[float, float]] = {}
        self._headings: Dict[str, float] = {}
        self._active: List[str] = []

    def update(self, msg) -> None:
        active: List[str] = []
        for v in msg.vehicles:
            vid = v.vehicle_id
            t = _stamp_to_seconds(v.header.stamp)
            x = float(v.position.x)
            y = float(v.position.y)
            buf = self._samples.setdefault(vid, deque(maxlen=5))

            # Detect a position jump against the previous sample (if any).
            jumped = False
            if buf:
                _t_prev, x_prev, y_prev = buf[-1]
                if math.hypot(x - x_prev, y - y_prev) > self._jump_thresh:
                    buf.clear()
                    jumped = True
                    self._warn(
                        f"V2X: position jump for vehicle '{vid}' "
                        f"(>{self._jump_thresh} m) — velocity reset")

            buf.append((t, x, y))

            if jumped or len(buf) < 2:
                self._velocities[vid] = (0.0, 0.0)
                self._accelerations[vid] = (0.0, 0.0)
                self._headings[vid] = 0.0
            else:
                vx_list = []
                vy_list = []
                for i in range(1, len(buf)):
                    dt = buf[i][0] - buf[i-1][0]
                    if dt > 0.0:
                        vx_list.append((buf[i][1] - buf[i-1][1]) / dt)
                        vy_list.append((buf[i][2] - buf[i-1][2]) / dt)

                if not vx_list:
                    self._velocities[vid] = (0.0, 0.0)
                    self._accelerations[vid] = (0.0, 0.0)
                    self._headings[vid] = 0.0
                else:
                    vx_avg = sum(vx_list) / len(vx_list)
                    vy_avg = sum(vy_list) / len(vy_list)
                    
                    if math.hypot(vx_avg, vy_avg) > self._v_max_safety:
                        self._velocities[vid] = (0.0, 0.0)
                        self._accelerations[vid] = (0.0, 0.0)
                        self._warn(
                            f"V2X: velocity for vehicle '{vid}' exceeds "
                            f"{self._v_max_safety} m/s — clamped to zero")
                    else:
                        self._velocities[vid] = (vx_avg, vy_avg)
                        # Only update heading if we have meaningful movement to avoid noise
                        if math.hypot(vx_avg, vy_avg) > 0.5:
                            self._headings[vid] = math.atan2(vy_avg, vx_avg)
                        elif vid not in self._headings:
                            self._headings[vid] = 0.0
                            
                        # Calculate acceleration if we have at least 2 velocity samples
                        if len(vx_list) >= 2:
                            dt_accel = buf[-1][0] - buf[0][0] # Time span of the buffer
                            if dt_accel > 0:
                                ax = (vx_list[-1] - vx_list[0]) / dt_accel
                                ay = (vy_list[-1] - vy_list[0]) / dt_accel
                                self._accelerations[vid] = (ax, ay)
                            else:
                                self._accelerations[vid] = (0.0, 0.0)
                        else:
                            self._accelerations[vid] = (0.0, 0.0)
            active.append(vid)
        self._active = active

    def velocity(self, vehicle_id: str) -> Tuple[float, float]:
        return self._velocities.get(vehicle_id, (0.0, 0.0))

    def acceleration(self, vehicle_id: str) -> Tuple[float, float]:
        return self._accelerations.get(vehicle_id, (0.0, 0.0))

    def heading(self, vehicle_id: str) -> float:
        return self._headings.get(vehicle_id, 0.0)

    def predict_positions(
        self, vehicle_id: str, t_samples
    ) -> List[Tuple[float, float]]:
        buf = self._samples.get(vehicle_id)
        if not buf:
            return []
        _t_last, x_last, y_last = buf[-1]
        vx, vy = self._velocities.get(vehicle_id, (0.0, 0.0))
        return [(x_last + vx * t, y_last + vy * t) for t in t_samples]

    def active_vehicle_ids(self) -> List[str]:
        return list(self._active)

    def predict_all(self, t_samples) -> Dict[str, List[Tuple[float, float]]]:
        return {vid: self.predict_positions(vid, t_samples) for vid in self._active}


def predictions_to_obstacles(predictions, vehicle_radius: float, obstacle_cls=None):
    """Flatten a ``{vehicle_id: [(x, y), ...]}`` mapping into a list of
    circular obstacles consumable by ``multi_purpose_mpc_ros.core.map``.

    ``obstacle_cls`` is injectable for testability; production callers
    leave it as ``None`` to use ``core.map.Obstacle``. The deferred
    import keeps this module's load time fast and lets the unit tests
    on hosts without ``scikit-image`` exercise the helper with a stub
    dataclass.
    """
    if obstacle_cls is None:
        from multi_purpose_mpc_ros.core.map import Obstacle as obstacle_cls
    out = []
    for _vid, points in predictions.items():
        for x, y in points:
            out.append(obstacle_cls(cx=x, cy=y, radius=vehicle_radius, vid=_vid))
    return out
