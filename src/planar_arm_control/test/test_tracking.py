"""Tests for trajectory tracking controllers and timer diagnostics."""

import unittest

from planar_arm_control.tracking import (
    PIDTracker,
    TimerJitter,
    velocity_tracking_command,
)


class TestTracking(unittest.TestCase):
    """Verify controller output bounds and monotonic timer measurements."""

    def test_pid_effort_is_bounded_and_reset_clears_integral(self):
        tracker = PIDTracker(2, 4.0, 2.0, 1.0, [5.0, 8.0])

        effort = tracker.compute([2.0, -3.0], [1.0, -1.0], 0.1)

        self.assertEqual(effort, [5.0, -8.0])
        self.assertNotEqual(tracker.integral, [0.0, 0.0])
        tracker.reset()
        self.assertEqual(tracker.integral, [0.0, 0.0])

    def test_pid_rejects_nonpositive_dt(self):
        tracker = PIDTracker(1, 1.0, 0.0, 0.0, [10.0])

        with self.assertRaisesRegex(ValueError, 'dt > 0'):
            tracker.compute([0.0], [0.0], 0.0)

    def test_pid_gains_can_be_updated_without_recreating_tracker(self):
        tracker = PIDTracker(1, 1.0, 0.0, 0.0, [10.0])
        tracker.compute([0.5], [0.0], 0.1)

        tracker.set_gains(4.0, 0.5, 2.0)

        self.assertEqual((tracker.kp, tracker.ki, tracker.kd), (4.0, 0.5, 2.0))
        self.assertEqual(tracker.compute([0.5], [0.0], 0.1), [2.025])

    def test_pid_rejects_negative_live_gain_updates(self):
        tracker = PIDTracker(1, 1.0, 0.0, 0.0, [10.0])

        with self.assertRaisesRegex(ValueError, 'non-negative'):
            tracker.set_gains(1.0, -0.1, 0.0)

    def test_velocity_tracker_applies_position_correction_and_limit(self):
        command = velocity_tracking_command(
            [0.2, -0.2], [0.1, -0.5], 2.0, 0.5
        )

        self.assertEqual(command, [0.4, -0.5])

    def test_timer_jitter_tracks_recent_period_deviation(self):
        jitter = TimerJitter(0.02, history_size=2)

        self.assertIsNone(jitter.observe(1.0))
        self.assertAlmostEqual(jitter.observe(1.025), 0.005)
        jitter.observe(1.040)
        summary = jitter.summary()

        self.assertAlmostEqual(summary[0], -0.005)
        self.assertAlmostEqual(summary[1], 0.005)
        self.assertAlmostEqual(summary[2], 0.005)

    def test_timer_jitter_rejects_nonmonotonic_observations(self):
        jitter = TimerJitter(0.01)
        jitter.observe(1.0)

        with self.assertRaisesRegex(ValueError, 'monotonic'):
            jitter.observe(0.9)


if __name__ == '__main__':
    unittest.main()
