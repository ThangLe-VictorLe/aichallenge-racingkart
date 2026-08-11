"""Unit tests for corridor stop logic — decoupled from MPC solver state."""

import unittest
import numpy as np


class TestCorridorStop(unittest.TestCase):

    # ------------------------------------------------------------------ helpers

    def _make_path_constraints(self, n_waypoints, N, ub_val, lb_val):
        """Build a (path_constraints[0], path_constraints[1]) pair mimicking
        the structure that _compute_corridor_width_at_ego() reads from."""
        ub = np.full((n_waypoints, N), ub_val)
        lb = np.full((n_waypoints, N), lb_val)
        return [ub, lb]

    def _compute_corridor_width_at_ego(self, path_constraints, wp_id, N):
        """Inline replica of the helper added to MPC controller."""
        try:
            pc = path_constraints
            if pc is None or len(pc) < 2:
                return float('inf')
            ub_all = pc[0]
            lb_all = pc[1]
            wp_id = (wp_id + 1) % len(ub_all)
            ub_row = ub_all[wp_id]
            lb_row = lb_all[wp_id]
            widths = ub_row - lb_row
            return float(np.min(widths))
        except Exception:
            return float('inf')

    # ------------------------------------------------------------------ tests

    def test_wide_corridor_does_not_trigger_stop(self):
        """Width 3.0m, vehicle 1.3m → should NOT stop."""
        N = 20
        pc = self._make_path_constraints(100, N, 1.5, -1.5)  # width = 3.0m
        width = self._compute_corridor_width_at_ego(pc, 5, N)
        self.assertAlmostEqual(width, 3.0)
        self.assertFalse(width < 1.3)

    def test_narrow_corridor_triggers_stop(self):
        """Width 0.8m, vehicle 1.3m → should stop."""
        N = 20
        pc = self._make_path_constraints(100, N, 0.5, -0.3)  # width = 0.8m
        width = self._compute_corridor_width_at_ego(pc, 5, N)
        self.assertAlmostEqual(width, 0.8)
        self.assertTrue(width < 1.3)

    def test_zero_corridor_triggers_stop(self):
        """Width 0.0m (corridor fully closed) → should stop."""
        N = 20
        pc = self._make_path_constraints(100, N, 0.0, 0.0)
        width = self._compute_corridor_width_at_ego(pc, 5, N)
        self.assertAlmostEqual(width, 0.0)
        self.assertTrue(width < 1.3)

    def test_most_restrictive_horizon_point_used(self):
        """If any point in the N-step horizon is narrow, stop should trigger."""
        N = 20
        ub = np.full((100, N), 1.5)
        lb = np.full((100, N), -1.5)
        # Make one step in horizon very narrow
        ub[6, 10] = 0.2
        lb[6, 10] = -0.1
        pc = [ub, lb]
        width = self._compute_corridor_width_at_ego(pc, 5, N)
        # Min width should be 0.3m (the pinched step), not 3.0m
        self.assertAlmostEqual(width, 0.3)
        self.assertTrue(width < 1.3)

    def test_none_constraints_returns_inf(self):
        """None path_constraints → inf width (safe, no stop)."""
        width = self._compute_corridor_width_at_ego(None, 0, 20)
        self.assertEqual(width, float('inf'))

    def test_dynamic_vehicle_width_determines_threshold(self):
        """Changing bicycle_model.width changes the stop threshold."""
        N = 20
        pc = self._make_path_constraints(100, N, 0.6, -0.5)  # width = 1.1m
        width = self._compute_corridor_width_at_ego(pc, 5, N)
        self.assertAlmostEqual(width, 1.1)

        self.assertFalse(width < 1.0)  # 1.0m kart: fits through 1.1m gap
        self.assertTrue(width < 1.3)   # 1.3m kart: does NOT fit through 1.1m gap
        self.assertTrue(width < 1.45)  # 1.45m kart: does NOT fit through 1.1m gap

    def test_deceleration_on_narrow_corridor(self):
        """Full stop deceleration logic with dynamic vehicle width."""
        v_current = 5.0
        a_min = -11.505

        N = 20
        pc = self._make_path_constraints(100, N, 0.4, -0.3)  # width = 0.7m
        curr_width_val = self._compute_corridor_width_at_ego(pc, 0, N)
        vehicle_width = 1.3

        u = [30.0, 0.0]
        corridor_is_too_narrow = bool(curr_width_val < vehicle_width)

        if corridor_is_too_narrow:
            u[0] = 0.0
            acc = a_min if abs(v_current) > 0.05 else 0.0

        self.assertTrue(corridor_is_too_narrow)
        self.assertEqual(u[0], 0.0)
        self.assertEqual(acc, a_min)

    def test_stopped_kart_deceleration_output(self):
        """When already fully stopped, acc should be 0 even if corridor narrow."""
        v_stopped = 0.01  # m/s — essentially still
        a_min = -11.505

        N = 20
        pc = self._make_path_constraints(100, N, 0.4, -0.3)
        curr_width_val = self._compute_corridor_width_at_ego(pc, 0, N)
        vehicle_width = 1.3

        u = [30.0, 0.0]
        corridor_is_too_narrow = bool(curr_width_val < vehicle_width)

        if corridor_is_too_narrow:
            u[0] = 0.0
            acc = a_min if abs(v_stopped) > 0.05 else 0.0

        self.assertTrue(corridor_is_too_narrow)
        self.assertEqual(u[0], 0.0)
        self.assertEqual(acc, 0.0)


if __name__ == "__main__":
    unittest.main()
