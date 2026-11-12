"""Launch the standalone simulated controller and teleoperation GUI."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch the default simulated controller and GUI."""
    return LaunchDescription([
        Node(
            package="planar_arm_control",
            executable="controller_node",
            name="controller_node",
            output="screen",
            parameters=[{"backend_mode": "simulated"}],
        ),
        Node(
            package="planar_arm_control",
            executable="gui_node",
            name="gui_node",
            output="screen",
        ),
    ])
