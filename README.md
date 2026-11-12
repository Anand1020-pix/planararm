# Planar Arm ROS 2 Controller

ROS 2 Jazzy control, teleoperation, and RViz visualization for a 3-DoF planar
arm. The provided
[`planar_arm.py`](src/planar_arm_control/planar_arm_control/planar_arm.py)
kinematics library is used unchanged.

## Arm conventions

The planner's coordinates are planar `(x, y)` with `y >= 0` as the ground
constraint. The URDF uses ROS's Z-up convention, so planner `y` maps to robot
`z`; all three revolute joints rotate around the negative Y axis. `base_link`
is located at the shoulder; Gazebo spawns it 0.2 m above the floor. Link
lengths are `[3.0, 2.0, 1.5]`, in meters. Joint limits are:

| Joint | Lower | Upper |
|---|---:|---:|
| joint1 | 0° | 180° |
| joint2 | -120° | 120° |
| joint3 | -120° | 120° |

## Build

Install ROS 2 Jazzy and the controller/visualization dependencies:

```bash
sudo apt install \
  ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
  ros-jazzy-robot-state-publisher ros-jazzy-xacro ros-jazzy-rviz2 \
  ros-jazzy-joy python3-numpy python3-pyqt5 python3-pyqtgraph
```

From the workspace root:

```bash
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

## Launch

Standalone controller plus teleoperation GUI (in-process simulated arm):

```bash
ros2 launch planar_arm_control bringup.launch.py
```

Robot description, state publisher, and RViz with the simulated controller:

```bash
ros2 launch planar_arm_control display.launch.py
```

The gamepad listener subscribes to `/joy`; start a joystick driver separately
with `ros2 run joy joy_node`.

## Command and telemetry interfaces

* `/target_pose` (`geometry_msgs/PointStamped`): planar target with `point.x`
  as robot X and `point.z` as planner Y/robot Z, relative to `base_link`.
  `point.y` is ignored. Targets outside the arm's maximum reach, targets
  without a valid IK solution, and paths that violate joint or ground
  constraints are rejected and reported on `/controller_status`.
* `/joint_target` (`std_msgs/Float64MultiArray`): three joint positions in
  radians. This shares the same path planner and constraint checks.
* `/joint_states` (`sensor_msgs/JointState`): fixed-rate state in standalone
  mode. The `ros2_control` backend accepts feedback from a configured
  joint-state source.
* `/joint_trajectory_controller/joint_trajectory`
  (`trajectory_msgs/JointTrajectory`): optional ros2_control backend command
  stream.

Example Cartesian command:

```bash
ros2 topic pub --once /target_pose geometry_msgs/msg/PointStamped \
  "{header: {frame_id: base_link}, point: {x: 4.0, y: 0.0, z: 2.0}}"
```

The controller uses a quintic time scaling with zero endpoint velocity and
acceleration. `publish_rate_hz`, `trajectory_duration`,
`max_joint_velocity`, `backend_mode`, topic names, joystick rates/steps, and
the joystick trajectory duration are ROS parameters. `backend_mode` is
`simulated` for standalone runs and `ros2_control` for use with a configured
controller. The repository includes Gazebo model assets, but no Gazebo launch
file. The planner depends on the small
`JointCommandBackend` boundary; a hardware adapter can replace the backend
without changing IK, trajectory timing, or constraint validation.

## Teleoperation

* GUI Cartesian fields send planner XY targets (robot X/Z); joint fields send a
  joint-space target.
  The live plot shows the arm, end-effector coordinates, and joint angles.
* Keyboard controls (when the GUI window has focus): `W/A/S/D` or arrow keys
  jog the end effector in XY; `1/2`, `3/4`, and `5/6` jog joints 1, 2, and 3
  down/up.
* Gamepad: axes 0/1 jog XY (Y is inverted by default for common sticks);
  rising button pairs `0/1`, `2/3`, and `4/5` jog joints 1, 2, and 3 down/up.
  Axis sign/speed and joint jog step are parameters.

All jogs are converted into targets and pass through the same IK/trajectory
planner and safety checks.
