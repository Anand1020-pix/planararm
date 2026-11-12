# Planar Arm ROS 2 Controller

ROS 2 Jazzy control, teleoperation, and RViz visualization for a 3-DoF planar
arm. The default bringup runs an in-process simulated arm and does not launch
Gazebo. The provided
[`planar_arm.py`](src/planar_arm_control/planar_arm_control/planar_arm.py)
kinematics library is used unchanged.

## Arm conventions

The planner's coordinates are planar `(x, y)` with `y >= 0` as the ground
constraint. The URDF uses ROS's Z-up convention, so planner `y` maps to robot
`z`; all three revolute joints rotate around the negative Y axis. `base_link` is located at the shoulder. Link lengths are `[3.0, 2.0, 1.5]`,
in meters. Joint limits are:

| Joint | Lower | Upper |
|---|---:|---:|
| joint1 | 0° | 180° |
| joint2 | -120° | 120° |
| joint3 | -120° | 120° |

## Build

Install ROS 2 Jazzy and the controller/visualization dependencies:

```bash
sudo apt install \
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

Launch the simulated arm controller, robot state publisher, and teleoperation
GUI:

```bash
ros2 launch planar_arm_control bringup.launch.py
```

The GUI's XY and joint controls publish targets to the controller. Start RViz
manually in another terminal if you want to view the robot model:

```bash
rviz2 -d "$(ros2 pkg prefix --share planar_arm_control)/rviz/planar_arm.rviz"
```

Keyboard jogging works while the GUI window has focus. To use a gamepad, start
its ROS driver in another terminal:

```bash
ros2 run joy joy_node
```

Disable the GUI when launching if desired:

```bash
ros2 launch planar_arm_control bringup.launch.py gui:=false
```

For RViz-only visualization with the in-process simulated controller:

```bash
ros2 launch planar_arm_control display.launch.py
```

## Command and telemetry interfaces

* `/target_pose` (`geometry_msgs/PointStamped`): planar target with `point.x`
  as robot X and `point.z` as planner Y/robot Z, relative to `base_link`.
  `point.y` is ignored. Targets outside the arm's maximum reach, targets
  without a valid IK solution, and paths that violate joint or ground
  constraints are rejected and reported on `/controller_status`.
* `/joint_target` (`std_msgs/Float64MultiArray`): three joint positions in
  radians. This shares the same path planner and constraint checks.
* `/joint_states` (`sensor_msgs/JointState`): fixed-rate state from the
  in-process simulated backend.

Example Cartesian command:

```bash
ros2 topic pub --once /target_pose geometry_msgs/msg/PointStamped \
  "{header: {frame_id: base_link}, point: {x: 4.0, y: 0.0, z: 2.0}}"
```

The controller uses a quintic time scaling with zero endpoint velocity and
acceleration. `publish_rate_hz`, `trajectory_duration`,
`max_joint_velocity`, `backend_mode`, topic names, joystick rates/steps, and
the joystick trajectory duration are ROS parameters. `bringup.launch.py` uses
the `simulated` backend. The controller also has an optional `ros2_control`
trajectory backend for a separately configured hardware system. The planner
depends on the small `JointCommandBackend` boundary; a hardware adapter can
replace the backend without changing IK, trajectory timing, or constraint
validation.

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
