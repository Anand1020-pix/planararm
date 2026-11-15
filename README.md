<div align="center">

# Planar Arm ROS 2 Controller

**Control, teleoperation, visualization, and physics simulation for a 3-DoF planar arm**

![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy-22314E?style=for-the-badge&logo=ros&logoColor=white)
![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-F58113?style=for-the-badge)
![Python](https://img.shields.io/badge/Python-3-3776AB?style=for-the-badge&logo=python&logoColor=white)
![Platform](https://img.shields.io/badge/Ubuntu-24.04-E95420?style=for-the-badge&logo=ubuntu&logoColor=white)

[Demo](#demo) | [Quick Start](#quick-start) | [Interfaces](#command-and-telemetry-interfaces) | [Teleoperation](#teleoperation) | [Troubleshooting](#troubleshooting)

</div>

---

## Overview

This package provides a complete ROS 2 Jazzy stack for a 3-DoF planar arm: a trajectory-planning controller, a teleoperation GUI, RViz visualization, and Gazebo Harmonic simulation through `ros2_control`. The provided [`planar_arm.py`](src/planar_arm_control/planar_arm_control/planar_arm.py) kinematics library is used for IK and constraint checking.

| Capability | Description |
|---|---|
| Control | Quintic trajectory planning with zero endpoint velocity and acceleration |
| Command modes | Position, velocity, and effort in Gazebo simulation |
| Pick and place | Physics-based grasping with a two-finger gripper |
| Teleoperation | GUI, keyboard, and gamepad, all through the same safety checks |
| Diagnostics | Live tracking-error plots and control-timer jitter reporting |
| Lifecycle | Controller and orchestrator run as managed lifecycle nodes |

---

## Demo

<div align="center">

https://github.com/user-attachments/assets/b23d02e2-f38e-486f-8414-84d4dff2dabf

<sub>Demo: the arm performing a full pick and place cycle in Gazebo Harmonic.</sub>

</div>

---

## Arm Conventions

The planner uses planar `(x, y)` coordinates with `y >= 0` as the ground constraint. The URDF uses the ROS Z-up convention, so planner `y` maps to robot `z`. All three revolute joints rotate around the negative Y axis, and `base_link` is located at the shoulder.

**Link lengths:** `[3.0, 2.0, 1.5]` meters

| Joint | Lower limit | Upper limit |
|---|---:|---:|
| `joint1` | 0 deg | 180 deg |
| `joint2` | -120 deg | 120 deg |
| `joint3` | -120 deg | 120 deg |

---

## Quick Start

### 1. Install dependencies

Install ROS 2 Jazzy and the controller and visualization dependencies:

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

### 2. Build

From the workspace root:

```bash
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

### 3. Launch

Choose the launch file that matches your goal:

| Goal | Command |
|---|---|
| Simulated controller, robot state publisher, and GUI | `ros2 launch planar_arm_control bringup.launch.py` |
| Gazebo physics simulation with GUI | `ros2 launch planar_arm_control simulation.launch.py` |
| RViz-only visualization with in-process controller | `ros2 launch planar_arm_control display.launch.py` |

---

## Launch Options

### Gazebo simulation

```bash
ros2 launch planar_arm_control simulation.launch.py
```

This opens Gazebo and the teleoperation GUI. The simulation launch spawns the arm, starts the joint-state broadcaster and trajectory controller, and sends the planner's joint setpoints to Gazebo through `ros2_control`. Use the GUI's XY, joint, or Pick & Place controls to move the arm.

Gazebo initializes the arm at its compact home pose (joint1 75 deg, joint2 99 deg, joint3 0 deg), so it starts clear of the pick table rather than sweeping there from a straight pose. The Pick & Place tab lets you set planar pick and place coordinates and command the gripper.

| Argument | Effect |
|---|---|
| `gui:=false` | Simulator-only run without the teleoperation GUI |
| `gazebo_gui:=false` | Run the Gazebo server headlessly |
| `command_mode:=position\|velocity\|effort` | Select the joint command interface |
| `autostart:=false` | Manage lifecycle transitions externally |

### Command modes

The simulation accepts the startup parameter `command_mode` with `position`, `velocity`, or `effort`. Position uses the joint trajectory controller; velocity and effort use forward-command controllers.

```bash
ros2 launch planar_arm_control simulation.launch.py command_mode:=velocity
ros2 launch planar_arm_control simulation.launch.py command_mode:=effort
```

> **Note on effort mode.** Effort is a joint torque/force interface, not motor-current control. The provided Gazebo model exposes position, velocity, and effort command interfaces. A physical robot can use the velocity or effort modes only if its hardware driver exports the matching interfaces and limits; the default hardware setup remains position/trajectory control. Effort PID gains and limits are configured with `effort_pid_kp`, `effort_pid_ki`, `effort_pid_kd`, and `effort_limits`. These require tuning for the target arm and are not validated for physical hardware.

### Lifecycle management

The controller and pick/place orchestrator are lifecycle nodes. They autoconfigure and activate by default; pass `autostart:=false` to the launch file to manage lifecycle transitions externally.

With `autostart:=false`, configure and activate the nodes before sending commands:

```bash
ros2 lifecycle set /controller_node configure
ros2 lifecycle set /controller_node activate
ros2 lifecycle set /pick_place_node configure
ros2 lifecycle set /pick_place_node activate
```

### Speed control and PID tuning

The arm controller runs its command loop at `publish_rate_hz` and publishes lower-rate tracking and timing diagnostics at `telemetry_rate_hz`.

The GUI's **- Slower** and **+ Faster** buttons adjust the active trajectory speed from 25% to 200% in 25% steps. A speed change replans from current feedback, and velocity and effort commands remain bounded by the configured joint velocity and effort limits.

The **PID tuning** tab applies effort PID gains and the velocity tracking gain live through ROS parameters. Adjust the gains while watching the per-joint tracking-error plot and RMS error readout. Effort PID gain changes reset the integral accumulator.

| Parameter | Supported range |
|---|---|
| Effort Kp | 0 to 500 |
| Effort Ki | 0 to 100 |
| Effort Kd | 0 to 100 |
| Velocity Kp | 0 to 100 |

---

## Pick and Place

The URDF includes a two-finger gripper driven by its own joint trajectory controller (the same plugin used by the arm). Pick/place closes the fingers at the selected pick point, lifts the colliding object using Gazebo physics, and opens the gripper at the selected destination. It does not teleport or visually attach the item.

The object must be a collidable Gazebo model positioned at the selected pick coordinates and small enough to fit between the fingers. The demo world contains a suitable item on the pick table.

### Running the default sequence

After `ros2 control list_controllers` shows both trajectory controllers and the joint-state controllers as `active`, start the default pick from another sourced terminal:

```bash
ros2 service call /pick_place/pick std_srvs/srv/Trigger "{}"
```

The arm approaches and grips at `(4.0, 2.0)`, the center of the settled item on the pick table, then lifts it. Call the place service to carry it to `(-3.0, 3.0)` and release it:

```bash
ros2 service call /pick_place/place std_srvs/srv/Trigger "{}"
```

To open the gripper and release an item at the arm's current position:

```bash
ros2 service call /pick_place/release std_srvs/srv/Trigger "{}"
```

### Customizing points

Coordinates are planar relative to `base_link`; planner Y maps to Gazebo Z. Choose other pick and place points in the GUI, or update the node's `pick_x`, `pick_y`, `place_x`, and `place_y` parameters before calling the services.

Follow progress on `/pick_place/status`. The GUI also provides manual open and close gripper buttons for positioning an object or recovering a task.

---

## Visualization

The GUI's XY and joint controls publish targets to the controller. Start RViz manually in another terminal to view the robot model:

```bash
rviz2 -d "$(ros2 pkg prefix --share planar_arm_control)/rviz/planar_arm.rviz"
```

To disable the GUI at launch:

```bash
ros2 launch planar_arm_control bringup.launch.py gui:=false
```

---

## Command and Telemetry Interfaces

### Topics

| Topic | Type | Purpose |
|---|---|---|
| `/target_pose` | `geometry_msgs/PointStamped` | Planar target. `point.x` is robot X and `point.z` is planner Y (robot Z), relative to `base_link`. `point.y` is ignored. |
| `/joint_target` | `std_msgs/Float64MultiArray` | Three joint positions in radians. Shares the same path planner and constraint checks. |
| `/joint_states` | `sensor_msgs/JointState` | Feedback from Gazebo's joint-state broadcaster in `simulation.launch.py`, or fixed-rate state from the in-process backend in `bringup.launch.py`. |
| `/joint_trajectory_controller/joint_trajectory` | `trajectory_msgs/JointTrajectory` | Gazebo ros2_control command stream. |
| `/joint_velocity_controller/commands` | `std_msgs/Float64MultiArray` | Joint velocity commands in rad/s when `command_mode:=velocity`. |
| `/joint_effort_controller/commands` | `std_msgs/Float64MultiArray` | Joint torque commands when `command_mode:=effort`. |
| `/controller_error` | `std_msgs/Float64MultiArray` | Per-joint desired-minus-measured position error in radians. |
| `/controller_timing` | `std_msgs/Float64MultiArray` | Latest signed timer jitter, rolling mean absolute jitter, and maximum absolute jitter, in seconds. |
| `/pick_place/status` | status stream | Pick/place progress. |

Targets outside the arm's maximum reach, targets without a valid IK solution, and paths that violate joint or ground constraints are rejected and reported on `/controller_status`.

### Services

| Service | Type | Purpose |
|---|---|---|
| `/controller/cancel` | `std_srvs/Trigger` | Cancel the active trajectory and hold the measured joint pose. The GUI's **Cancel movement** action also cancels an active pick/place sequence. |
| `/pick_place/cancel` | `std_srvs/Trigger` | Cancel a pick/place sequence and request controller cancellation. |
| `/pick_place/pick` | `std_srvs/Trigger` | Run the pick sequence. |
| `/pick_place/place` | `std_srvs/Trigger` | Carry and release the item at the place point. |
| `/pick_place/release` | `std_srvs/Trigger` | Open the gripper at the current position. |

### Example Cartesian command

```bash
ros2 topic pub --once /target_pose geometry_msgs/msg/PointStamped \
  "{header: {frame_id: base_link},   point: {x: 4.0, y: 0.0, z: 1.95}}"
```

---

## Controller Design

The controller uses a quintic time scaling with zero endpoint velocity and acceleration. A new valid target received during motion preempts the active path and replans from the latest joint feedback.

The following are ROS parameters:

* `publish_rate_hz`, `telemetry_rate_hz`
* `trajectory_duration`, `max_joint_velocity`
* `backend_mode`, `command_mode`
* PID parameters and topic names
* Joystick rates, steps, and trajectory duration

The GUI plots joint tracking error and reports measured control-timer jitter.

`bringup.launch.py` uses the `simulated` position backend; `simulation.launch.py` uses `ros2_control` and supports the three command modes. The planner depends on the small `JointCommandBackend` boundary, so a hardware adapter can replace the backend without changing IK, trajectory timing, or constraint validation.

---
User Interface
<div align="center"> <img width="1366" height="853" alt="ui" src="https://github.com/user-attachments/assets/a90767c6-eac5-4232-b36e-17ba6b25b8e1" />

<sub>Teleoperation GUI: live arm plot, XY and joint controls, Pick & Place tab, and tracking-error plot.</sub>

</div>

---

## Teleoperation

The live plot shows the arm, end-effector coordinates, and joint angles.

| Input | Controls |
|---|---|
| GUI | Cartesian fields send planner XY targets (robot X/Z). Joint fields send a joint-space target. |
| Keyboard (GUI window focused) | `W/A/S/D` or arrow keys jog the end effector in XY. `1/2`, `3/4`, and `5/6` jog joints 1, 2, and 3 down/up. |
| Gamepad | Axes 0/1 jog XY (Y is inverted by default for common sticks). Rising button pairs `0/1`, `2/3`, and `4/5` jog joints 1, 2, and 3 down/up. Axis sign, speed, and joint jog step are parameters. |

To use a gamepad, start its ROS driver in another terminal:

```bash
ros2 run joy joy_node
```

All jogs are converted into targets and pass through the same IK, trajectory planner, and safety checks.

---

## Testing

Run the focused Python tests from the workspace after sourcing ROS 2:

```bash
colcon test --packages-select planar_arm_control
colcon test-result --verbose
```

---

## Troubleshooting

### Gazebo exits while activating the trajectory controller

Make sure the ROS 2 control packages are from a consistent Jazzy installation. Update the controller manager, controller interfaces, hardware interface, Gazebo integration, and joint controllers together before retrying:

```bash
sudo apt update
sudo apt install --only-upgrade \
  ros-jazzy-controller-manager ros-jazzy-controller-interface \
  ros-jazzy-hardware-interface ros-jazzy-ros2-control \
  ros-jazzy-gz-ros2-control ros-jazzy-joint-state-broadcaster \
  ros-jazzy-joint-trajectory-controller ros-jazzy-diagnostic-updater
```

### Missing symbol error for `diagnostic_updater::Updater`

This indicates that the installed `ros-jazzy-diagnostic-updater` is older than the controller manager expects. Upgrade it with the other ROS control packages above.

### Gazebo server fails to start

The Gazebo server must be launched from a terminal where `/opt/ros/jazzy/setup.bash` has been sourced.
