# Planar Arm ROS 2 Controller

ROS 2 Jazzy control, teleoperation, RViz visualization, and Gazebo Harmonic
simulation for a 3-DoF planar arm. The provided
[`planar_arm.py`](src/planar_arm_control/planar_arm_control/planar_arm.py)
kinematics library is used.

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
  ros-jazzy-ros-gz ros-jazzy-gz-ros2-control \
  ros-jazzy-ros2-control ros-jazzy-controller-manager \
  ros-jazzy-joint-state-broadcaster \
  ros-jazzy-joint-trajectory-controller \
  ros-jazzy-forward-command-controller \
  ros-jazzy-ros-gz-bridge ros-jazzy-ros-gz-sim \
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

To see and control the moving robot in Gazebo, launch the Gazebo-backed
simulation instead:

```bash
ros2 launch planar_arm_control simulation.launch.py
```

This opens Gazebo and the teleoperation GUI. The simulation launch spawns the
arm, starts the joint-state broadcaster and trajectory controller, and sends
the planner's joint setpoints to Gazebo through `ros2_control`. Use the GUI's
XY, joint, or Pick & Place controls to move the arm. Gazebo initializes the arm
at its compact home pose (joint1 75°, joint2 99°, joint3 0°), so it starts
clear of the pick table rather than sweeping there from a straight pose. The
Pick & Place tab lets you set planar pick/place coordinates and command the
gripper. Set `gui:=false` for a simulator-only run; set `gazebo_gui:=false` to
run the Gazebo server headlessly.

The simulation accepts the startup parameter `command_mode` with `position`,
`velocity`, or `effort`.
Position uses the joint trajectory controller; velocity and effort use
forward-command controllers. For example:

```bash
ros2 launch planar_arm_control simulation.launch.py command_mode:=velocity
ros2 launch planar_arm_control simulation.launch.py command_mode:=effort
```

Effort is a joint torque/force interface, not motor-current control. The
provided Gazebo model exposes position, velocity, and effort command
interfaces. A physical robot can use the velocity or effort modes only if its
hardware driver exports the matching interfaces and limits; the default
hardware setup remains position/trajectory control. Effort PID gains and
limits are configured with `effort_pid_kp`, `effort_pid_ki`, `effort_pid_kd`,
and `effort_limits`; these require tuning for the target arm and are not
validated for physical hardware.

The controller and pick/place orchestrator are lifecycle nodes. They
autoconfigure and activate by default; pass `autostart:=false` to the launch
file to manage lifecycle transitions externally. The arm controller runs
its command loop at `publish_rate_hz` and publishes lower-rate tracking and
timing diagnostics at `telemetry_rate_hz`.
The GUI's **− Slower** and **+ Faster** buttons adjust the active trajectory
speed from 25% to 200% in 25% steps; a speed change replans from current
feedback, and velocity/effort commands remain bounded by the configured joint
velocity and effort limits. The **PID tuning** tab applies effort PID gains
and velocity tracking gain live through ROS parameters. Adjust the gains
while watching the per-joint tracking-error plot and RMS error readout; effort
PID gain changes reset the integral accumulator. Supported tuning ranges are
effort Kp 0–500, Ki/Kd 0–100, and velocity Kp 0–100.

With `autostart:=false`, configure and activate the nodes before sending
commands:

```bash
ros2 lifecycle set /controller_node configure
ros2 lifecycle set /controller_node activate
ros2 lifecycle set /pick_place_node configure
ros2 lifecycle set /pick_place_node activate
```

The URDF includes a two-finger gripper driven by its own joint trajectory
controller (the same plugin used by the arm). Pick/place closes the fingers
at the selected pick point,
lifts the colliding object using Gazebo physics, and opens the gripper at the
selected destination; it no longer teleports or visually attaches the item.
The object must be a collidable Gazebo model positioned at the selected pick
coordinates and small enough to fit between the fingers. The demo world
contains a suitable item on the pick table. After
`ros2 control list_controllers` shows both trajectory controllers and the joint-state
controllers as `active`, the default pick can be started from another sourced
terminal:

```bash
ros2 service call /pick_place/pick std_srvs/srv/Trigger "{}"
```

The arm approaches and grips at `(4.0, 1.95)`, the center of the settled item
on the pick table, then lifts it. Call the
place service to carry it to `(-3.0, 3.0)` and release it:

```bash
ros2 service call /pick_place/place std_srvs/srv/Trigger "{}"
```

Coordinates are planar relative to `base_link`; planner Y maps to Gazebo Z.
Choose other pick/place points in the GUI or update the node's `pick_x`,
`pick_y`, `place_x`, and `place_y` parameters before calling the services.
To open the gripper and release an item at the arm's current position, call:

```bash
ros2 service call /pick_place/release std_srvs/srv/Trigger "{}"
```

Follow progress on `/pick_place/status`. The GUI also provides manual
open/close gripper buttons for positioning an object or recovering a task.

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
* `/joint_states` (`sensor_msgs/JointState`): feedback from Gazebo's
  joint-state broadcaster in `simulation.launch.py`, or fixed-rate state from
  the in-process backend in `bringup.launch.py`.
* `/joint_trajectory_controller/joint_trajectory`
  (`trajectory_msgs/JointTrajectory`): Gazebo ros2_control command stream.
* `/joint_velocity_controller/commands`
  (`std_msgs/Float64MultiArray`): joint velocity commands in radians/second
  when `command_mode:=velocity`.
* `/joint_effort_controller/commands`
  (`std_msgs/Float64MultiArray`): joint torque commands when
  `command_mode:=effort`.
* `/controller/cancel` (`std_srvs/Trigger`): cancel the active trajectory and
  hold the measured joint pose. The GUI's **Cancel movement** action also
  cancels an active pick/place sequence.
* `/pick_place/cancel` (`std_srvs/Trigger`): cancel a pick/place sequence and
  request controller cancellation.
* `/controller_error` (`std_msgs/Float64MultiArray`): per-joint
  desired-minus-measured position error in radians.
* `/controller_timing` (`std_msgs/Float64MultiArray`): latest signed timer
  jitter, rolling mean absolute jitter, and maximum absolute jitter, in
  seconds.

Example Cartesian command:

```bash
ros2 topic pub --once /target_pose geometry_msgs/msg/PointStamped \
  "{header: {frame_id: base_link},   point: {x: 4.0, y: 0.0, z: 1.95}}"
```

The controller uses a quintic time scaling with zero endpoint velocity and
acceleration. `publish_rate_hz`, `trajectory_duration`,
`max_joint_velocity`, `backend_mode`, `command_mode`, `telemetry_rate_hz`, PID
parameters, topic names, joystick rates/steps, and the joystick trajectory
duration are ROS parameters. A new valid target received during motion
preempts the active path and replans from the latest joint feedback. The GUI
plots joint tracking error and reports measured control-timer jitter.
`bringup.launch.py` uses the `simulated` position backend;
`simulation.launch.py` uses `ros2_control` and supports the three command
modes. The planner depends on the small `JointCommandBackend` boundary; a
hardware adapter can replace the backend without changing IK, trajectory
timing, or constraint validation.

Run the focused Python tests from the workspace after sourcing ROS 2:

```bash
colcon test --packages-select planar_arm_control
colcon test-result --verbose
```

If Gazebo exits while activating the trajectory controller, make sure the ROS
2 control packages are from a consistent Jazzy installation. Update the
controller manager, controller interfaces, hardware interface, Gazebo
integration, and joint controllers together before retrying:

```bash
sudo apt update
sudo apt install --only-upgrade \
  ros-jazzy-controller-manager ros-jazzy-controller-interface \
  ros-jazzy-hardware-interface ros-jazzy-ros2-control \
  ros-jazzy-gz-ros2-control ros-jazzy-joint-state-broadcaster \
  ros-jazzy-joint-trajectory-controller ros-jazzy-diagnostic-updater
```

The Gazebo server must be launched from a terminal where
`/opt/ros/jazzy/setup.bash` has been sourced. A missing-symbol error mentioning
`diagnostic_updater::Updater` indicates that the installed
`ros-jazzy-diagnostic-updater` is older than the controller manager expects;
upgrade it with the other ROS control packages above.

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
