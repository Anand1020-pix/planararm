"""Launch Gazebo with the arm controlled through ros2_control."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Run Gazebo, load the arm, start controllers, and open the GUI."""
    package_share = FindPackageShare("planar_arm_control")
    gazebo_share = FindPackageShare("ros_gz_sim")
    description_file = PathJoinSubstitution(
        [package_share, "description", "urdf", "planar_arm.urdf.xacro"]
    )
    controller_config = PathJoinSubstitution(
        [package_share, "config", "ros2_controllers.yaml"]
    )
    world_file = PathJoinSubstitution(
        [package_share, "worlds", "planar_arm_world.sdf"]
    )
    gazebo_launch_file = PathJoinSubstitution(
        [gazebo_share, "launch", "gz_sim.launch.py"]
    )
    robot_description = ParameterValue(Command([
        "xacro ",
        description_file,
        " controller_config:=",
        controller_config,
    ]), value_type=str)
    use_sim_time = LaunchConfiguration("use_sim_time")

    gazebo_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_file),
        launch_arguments={
            "gz_args": ["-r -v 3 ", world_file],
        }.items(),
        condition=IfCondition(LaunchConfiguration("gazebo_gui")),
    )
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_file),
        launch_arguments={
            "gz_args": ["-s -r -v 3 ", world_file],
        }.items(),
        condition=UnlessCondition(LaunchConfiguration("gazebo_gui")),
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "planar_arm",
            "-topic", "robot_description",
            "-allow_renaming", "true",
            "-z", "0.0",
        ],
        output="screen",
    )
    spawn_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "120",
        ],
        output="screen",
    )
    spawn_trajectory_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_trajectory_controller",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "120",
        ],
        output="screen",
    )
    spawn_gripper_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "gripper_controller",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "120",
        ],
        output="screen",
    )
    controller_node = Node(
        package="planar_arm_control",
        executable="controller_node",
        name="controller_node",
        output="screen",
        parameters=[
            {"backend_mode": "ros2_control"},
            {"use_sim_time": use_sim_time},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("gazebo_gui", default_value="true"),
        gazebo_gui,
        gazebo_server,
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="clock_bridge",
            arguments=[
                "/world/planar_arm_world/clock@rosgraph_msgs/msg/Clock"
                "[gz.msgs.Clock"
            ],
            remappings=[
                ("/world/planar_arm_world/clock", "/clock"),
            ],
            output="screen",
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
            executable="pick_place_node",
            name="pick_place_node",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
        ),
        Node(
            package="planar_arm_control",
            executable="gui_node",
            name="gui_node",
            output="screen",
            condition=IfCondition(LaunchConfiguration("gui")),
            parameters=[{"use_sim_time": use_sim_time}],
        ),
        TimerAction(period=4.0, actions=[spawn_robot]),
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_robot,
                on_exit=[spawn_joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_joint_state_broadcaster,
                on_exit=[spawn_trajectory_controller],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_trajectory_controller,
                on_exit=[spawn_gripper_controller, controller_node],
            )
        ),
    ])
