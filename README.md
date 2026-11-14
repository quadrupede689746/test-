# Robodog ROS 2 Humble workspace

This workspace is a ROS 2 Humble migration of the supplied `open_quadruped`
source archive. It preserves the robot's Xacro model and STL meshes, adds the
missing `control_library` as a ROS 2 Python package, generates the custom
`JointAngles` interface with `rosidl`, and adds a ROS 2 control/Gazebo launch.

## Packages

- `open_quadruped` — robot model, custom message, joystick body-pose IK node,
  JointAngles-to-trajectory bridge, and Gazebo Classic launch.
- `control_library` — Python leg/body IK and gait-planning helper modules.
- `robodog_sim` — convenience launch alias for the OpenQuadruped simulation.

The original archive's `robodog_sim` folders were empty placeholders. There
were no separate world files, meshes, or RViz configuration to restore; the
complete robot meshes included in the archive are kept under
`open_quadruped/urdf/stl`.

## Install dependencies (Ubuntu 22.04 + ROS 2 Humble)

The simulator in this port uses Gazebo Classic and `gazebo_ros2_control`,
which are the closest ROS 2 equivalents to the Gazebo Classic setup in the
original ROS 1 package.

```bash
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-numpy \
  python3-scipy \
  ros-humble-joy \
  ros-humble-xacro \
  ros-humble-robot-state-publisher \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-ros2-control \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers
```

Install any remaining dependencies from package manifests:

```bash
cd ~/robodog_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

## Build

```bash
cd ~/robodog_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

To build just the simulation package and its dependencies:

```bash
colcon build --symlink-install --packages-up-to robodog_sim
```

## Launch simulation

```bash
source /opt/ros/humble/setup.bash
source ~/robodog_ws/install/setup.bash
ros2 launch robodog_sim sim.launch.py
```

Or launch the robot simulation directly:

```bash
ros2 launch open_quadruped sim.launch.py
```

The default joystick device index is `0`. To select another device:

```bash
ros2 launch open_quadruped sim.launch.py joy_device_id:=1
```

To start without the Gazebo GUI:

```bash
ros2 launch open_quadruped sim.launch.py gui:=false
```

## Keyboard foot control (simulation)

This experimental mode selects one leg and moves its foot in the robot's X/Y
plane using inverse kinematics. It starts Gazebo without the joystick pipeline
and initializes the foot targets from TF.

```bash
source /opt/ros/humble/setup.bash
source ~/robodog_ws/install/setup.bash
ros2 launch open_quadruped keyboard_sim.launch.py
```

Select FL/FR/BL/BR with **1/2/3/4**. Arrow keys (or **W/A/S/D**) move the
selected foot along X or Y by `gain_m` (default 0.01 m); the path raises it by `z_lift_m` (default
0.02 m), then returns it to its starting Z at the destination. Press **q** to
quit. Set the move distance, for example, with
`ros2 launch open_quadruped keyboard_sim.launch.py gain_m:=0.02`. Adjust the
temporary lift with `z_lift_m:=0.03`. To run without the Gazebo GUI, add
`gui:=false`. This is
simulation-only; the real hardware transport is not implemented in this
workspace.

## Crawl gait coordinator (simulation)

The coordinator tests a slow, one-foot-at-a-time crawl sequence using the same
foot TF, IK, and trajectory bridge. Press **g** to start or pause (pausing waits
for the active foot to land), **+/-** to adjust step length, and **q** to exit
after the active step. The sequence is FL, BR, FR, BL. Each foot alternates
between two X footholds around its measured initial position. The default
peak-to-peak step is 0.04 m with a 0.025 m swing lift.

```bash
ros2 launch open_quadruped crawl_sim.launch.py
```

If Gazebo is already running with `joint_angles_to_trajectory` active, stop the
manual foot keyboard node with **q** and run the coordinator directly instead
of launching a second Gazebo server:

```bash
ros2 run open_quadruped crawl_gait_coordinator.py --ros-args \
  -p step_length_m:=0.04 -p swing_height_m:=0.025
```

Use only one foot-command node at a time.

The current Gazebo model disables link gravity and collision response, so this
validates gait timing and IK only; it will not translate the body across the
ground. Actual static walking stability also depends on the center of mass
remaining inside the three-foot support polygon. See [the static gait study](https://pmc.ncbi.nlm.nih.gov/articles/PMC7506578/).

## Topics and control flow

```text
USB joystick
    │
    ▼
/joy  (sensor_msgs/msg/Joy)
    │
    ▼
interface_process
    │  body-pose inverse kinematics
    ▼
/joint_angles  (open_quadruped/msg/JointAngles)
    │
    ▼
joint_angles_to_trajectory
    │
    ▼
/quadruped_controller/joint_trajectory
    │
    ▼
ros2_control / Gazebo Classic
```

Useful checks:

```bash
ros2 control list_controllers
ros2 topic echo /joint_states
ros2 topic echo /joint_angles
```

The ROS 2 controller is configured for the 12 joints actually present in the
Xacro model: hip, upper-leg, and lower-leg joints for front-left, front-right,
back-left, and back-right legs. The stale ROS 1 `control.yaml` used different
joint names and has been replaced by `config/ros2_controllers.yaml`.

## What is migrated, and what remains incomplete

1. **Body-pose IK:** migrated to `rclpy`. Joystick axes 2, 5, and 0 control yaw,
   pitch, and roll respectively, preserving the original mapping and 15-degree
   defaults.
2. **Gait mode:** the supplied ROS 1 `interface_process.py` switched to gait
   mode when Joy button 7 was held, but its gait branch never generated foot
   vectors or joint angles. The ROS 2 node therefore warns and stops publishing
   new commands while that button is held. The supplied gait helper modules are
   preserved, but a complete gait configuration/command node was not present
   in the provided source and has not been invented.
3. **Real robot serial transport:** the old launch used `rosserial_python` and a
   Teensy serial port. The Teensy firmware/protocol and a ROS 2 hardware driver
   were not included, so the new `real_control.launch.py` starts only the
   joystick and IK nodes. It does not claim to drive physical hardware.
4. **Simulation assets:** the uploaded `robodog_sim` package had empty `urdf`,
   `meshes`, `worlds`, `config`, `launch`, and `rviz` directories. This port uses
   the robot description and meshes from `open_quadruped` and Gazebo's default
   empty world. It does not recreate a custom world or a separate RViz layout.
5. **Legacy model files:** old ROS 1 launch/model variants are retained under
   `open_quadruped/urdf/legacy_ros1` for reference only. The active simulation
   uses `open_quadruped/urdf/spot.urdf.xacro`.

## Gazebo note

This launch targets Gazebo Classic because that is the simulator family used
by the original model. Gazebo Classic reached end of life in January 2025; for
newer Gazebo releases, the simulation plugin and launch should be migrated to
`gz_ros2_control` rather than `gazebo_ros2_control`.
# test-
