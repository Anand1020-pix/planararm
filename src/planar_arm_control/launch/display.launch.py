"""Launch the simulated arm state publisher and RViz display."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Launch the arm model, simulated joint states, and RViz."""
    package_share = FindPackageShare("planar_arm_control")
    description_file = PathJoinSubstitution(
        [package_share, "description", "urdf", "planar_arm.urdf.xacro"]
    )
    rviz_config = PathJoinSubstitution(
        [package_share, "rviz", "planar_arm.rviz"]
    )
    robot_description = Command(["xacro ", description_file])
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
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
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config],
            parameters=[{"use_sim_time": use_sim_time}],
            output="screen",
        ),
    ])
