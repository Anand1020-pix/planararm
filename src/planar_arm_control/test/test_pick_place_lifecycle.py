"""Lifecycle tests for the pick/place orchestrator."""

import unittest

from planar_arm_control.pick_place_node import PickPlaceNode

import rclpy
from rclpy.lifecycle import TransitionCallbackReturn


class TestPickPlaceLifecycle(unittest.TestCase):
    """Verify managed activation and timer shutdown."""

    @classmethod
    def setUpClass(cls):
        if not rclpy.ok():
            rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    def setUp(self):
        self.node = PickPlaceNode()
        self.assertEqual(
            self.node.trigger_configure(),
            TransitionCallbackReturn.SUCCESS,
        )
        self.assertEqual(
            self.node.trigger_activate(),
            TransitionCallbackReturn.SUCCESS,
        )

    def tearDown(self):
        self.node.destroy_node()

    def test_deactivation_stops_sequence_timer(self):
        self.assertTrue(self.node._active)

        result = self.node.trigger_deactivate()

        self.assertEqual(result, TransitionCallbackReturn.SUCCESS)
        self.assertFalse(self.node._active)
        self.assertTrue(self.node._timer.is_canceled())


if __name__ == '__main__':
    unittest.main()
