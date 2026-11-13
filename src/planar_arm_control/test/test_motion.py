"""Unit tests for kinematic target validation and trajectory generation."""

import math
import unittest

import numpy as np

from planar_arm_control.motion import (
    clamp_joint_limit_roundoff,
    make_trajectory,
    solve_xy_target,
    trajectory_respects_constraints,
)
from planar_arm_control.controller_node import HOME_POSITIONS
from planar_arm_control.pick_place_node import (
    build_pick_place_waypoints,
    build_pick_waypoints,
)
from planar_arm_control.planar_arm import PlanarArm


class TestMotion(unittest.TestCase):
    """Verify trajectory limits and target handling with the provided arm."""

    def setUp(self):
        """Create the arm used by each independent test."""
        self.arm = PlanarArm([3.0, 2.0, 1.5])

    def test_quintic_trajectory_has_stationary_endpoints_and_bounded_speed(
        self,
    ):
        """Check endpoints, velocity scaling, and constraint sampling."""
        start = [0.3, 0.0, 0.0]
        goal = [1.3, 0.0, 0.0]
        trajectory = make_trajectory(start, goal, 0.5, 1.0)

        np.testing.assert_allclose(trajectory.sample(0.0), start)
        np.testing.assert_allclose(
            trajectory.sample(trajectory.duration), goal
        )
        self.assertGreaterEqual(trajectory.duration, 1.875)
        self.assertTrue(
            trajectory_respects_constraints(self.arm, trajectory, 0.01)
        )

        sample_times = np.linspace(0.0, trajectory.duration, 1001)
        samples = np.asarray(
            [trajectory.sample(value) for value in sample_times]
        )
        velocities = np.diff(samples, axis=0) / np.diff(sample_times)[:, None]
        self.assertLessEqual(float(np.max(np.abs(velocities))), 1.001)
        self.assertTrue(np.all(np.abs(samples[:, 0]) <= math.pi))

    def test_xy_target_uses_ik_and_returns_a_valid_solution(self):
        """Confirm IK matches the requested end-effector point."""
        known_positions = [0.6, 0.2, -0.2]
        target = self.arm.end_effector(known_positions)

        solution = solve_xy_target(self.arm, target, known_positions)

        self.assertTrue(self.arm.within_joint_limits(solution))
        self.assertTrue(self.arm.arm_above_base(solution))
        np.testing.assert_allclose(
            self.arm.end_effector(solution), target, atol=1e-4
        )

    def test_xy_target_outside_reach_is_rejected(self):
        """Reject targets beyond the arm's maximum radial reach."""
        with self.assertRaisesRegex(
            ValueError, "outside the arm's reachable radius"
        ):
            solve_xy_target(self.arm, (10.0, 0.0), [0.0, 0.0, 0.0])

    def test_feedback_roundoff_near_limit_is_clamped_but_real_error_remains(
        self
    ):
        """Clamp negligible noise without hiding real joint limit violations."""
        near_limit = clamp_joint_limit_roundoff(
            self.arm, [-1e-14, 0.0, 0.0]
        )
        self.assertEqual(near_limit[0], 0.0)
        self.assertTrue(self.arm.within_joint_limits(near_limit))

        outside_limit = clamp_joint_limit_roundoff(
            self.arm, [-1e-6, 0.0, 0.0]
        )
        self.assertEqual(outside_limit[0], -1e-6)
        self.assertFalse(self.arm.within_joint_limits(outside_limit))

    def test_pick_place_waypoints_are_reachable_and_path_safe(self):
        """Validate all pick/place stages from the configured home pose."""
        current = list(HOME_POSITIONS)
        for _, point in build_pick_place_waypoints():
            goal = solve_xy_target(self.arm, point, current)
            trajectory = make_trajectory(current, goal, 2.0, 1.0)
            self.assertTrue(
                trajectory_respects_constraints(
                    self.arm, trajectory, 0.005
                )
            )
            current = goal

    def test_requested_home_pose_is_bent_and_ground_safe(self):
        """The startup pose matches the requested compact joint angles."""
        np.testing.assert_allclose(
            HOME_POSITIONS, np.radians([75.0, 99.0, 0.0])
        )
        self.assertTrue(self.arm.within_joint_limits(HOME_POSITIONS))
        self.assertTrue(self.arm.arm_above_base(HOME_POSITIONS))
        end_x, end_y = self.arm.end_effector(HOME_POSITIONS)
        self.assertLess(math.hypot(end_x, end_y), sum(self.arm.link_lengths))

    def test_pick_and_place_waypoints_accept_operator_coordinates(self):
        """Task waypoints use user-selected source and destination points."""
        self.assertEqual(
            build_pick_waypoints((1.0, 2.0))[-1][1],
            (1.0, 2.0 + 0.3),
        )
        self.assertEqual(
            build_pick_place_waypoints(
                (1.0, 2.0), (-1.0, 2.5)
            )[-1][1],
            (-1.0, 2.5),
        )

    def test_pick_operation_includes_lifting_the_item(self):
        """Pick ends at the raised item pose before a separate place call."""
        pick_waypoints = build_pick_waypoints()
        self.assertEqual(len(pick_waypoints), 3)
        self.assertEqual(pick_waypoints[-1][0], "Lifting item")
        self.assertEqual(pick_waypoints[-1][1], (4.0, 2.25))

if __name__ == "__main__":
    unittest.main()
