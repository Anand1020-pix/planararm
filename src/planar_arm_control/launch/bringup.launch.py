"""Launch the simulated arm, robot description, and teleoperation GUI."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Start the controllable arm without an external simulator."""
    package_share = FindPackageShare("planar_arm_control")
    description_file = PathJoinSubstitution(
        [package_share, "description", "urdf", "planar_arm.urdf.xacro"]
    )
    robot_description = Command(["xacro ", description_file])
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("gui", default_value="true"),
        Node(
            package="planar_arm_control",
            executable="controller_node",
            name="controller_node",
            output="screen",
            parameters=[
                {"backend_mode": "simulated"},
                {"use_sim_time": use_sim_time},
            ],
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[
                {"robot_description": robot_description},
                {"use_sim_time": use_sim_time},
            ],
        ),
        Node(
            package="planar_arm_control",
            executable="gui_node",
            name="gui_node",
            output="screen",
            condition=IfCondition(LaunchConfiguration("gui")),
            parameters=[{"use_sim_time": use_sim_time}],
        ),
    ])
