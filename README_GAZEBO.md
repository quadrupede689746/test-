# Run a trained policy in Gazebo (ROS 2 Humble + Gazebo Classic)

`run_policy.py` only drives the MuJoCo model. For Gazebo use `gazebo_policy_node.py` (a ROS 2 node).
It reads /joint_states and /imu, rebuilds the observation the policy was trained on (dog_controller.py), and sends
12 joint targets at 50 Hz to /quadruped_controller/joint_trajectory.

IMPORTANT (gyro fix): models trained with v1-v3 used a wrong frame for the body rotation rate. Run those with
`legacy_gyro:=true` (the default of the node) / `--legacy-gyro` (run_policy.py). Models trained with this version (v4)
must be run with `legacy_gyro:=false` / without the flag.

## Terminal 1 - the simulation (do NOT use run_sim.sh / robodog_sim: it does not pass launch arguments)
    source /opt/ros/humble/setup.bash && source ~/robodog_ws/install/setup.bash
    ros2 launch open_quadruped sim.launch.py joystick_control:=false place_initial_stance:=true \
      initial_joint_positions:=0,-0.68162889,1.26606748,0,-0.68162889,1.26606748,0,-0.68162889,1.26606748,0,-0.68162889,1.26606748
 (joystick_control:=false keeps the joystick bridge from fighting the policy)

## Terminal 2 - the policy
    source /opt/ros/humble/setup.bash && source ~/robodog_ws/install/setup.bash
    cd ~/Downloads/robodog_rl          # folder with gazebo_policy_node.py, dog_controller.py, export.py
    python3 gazebo_policy_node.py --ros-args -p policy:=/full/path/to/policy.npz -p use_sim_time:=true
 (only numpy is needed; do not activate the torch environment here)
 Step 1 test: add `-p engage_policy:=false` -> the node only brings the legs to the standing pose.
 The node first glides the legs to the standing pose (about 2.5 s), then the policy takes over.

## Terminal 3 - commands
    sudo apt install ros-humble-teleop-twist-keyboard
    ros2 run teleop_twist_keyboard teleop_twist_keyboard        # i forward, , back, j / l = turn left/right (z lowers speed)
  or:  ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}"
  lie down / stand up:  ros2 topic pub --once /dog_mode std_msgs/msg/String "{data: lie}"     (walk | stand | lie)
 Commands are clamped to what the policy saw in training (vx -0.15..0.25 m/s, vy +-0.12, wz +-0.8 rad/s) and
 stop 1 s after the last /cmd_vel message (cmd_timeout).

## Check the node's logic without ROS
    python3 verify_controller.py models/<name>/policy.npz     # controller == training environment (targets identical)

## Expect differences (sim-to-sim gap)
 Gazebo and MuJoCo differ in contact, friction and how the joint controller is applied, so the dog may walk worse than
 in MuJoCo. If it does, send me what you see (tipping, slipping, jitter): the usual fixes are randomised friction/mass
 during training and matching the Gazebo PID/effort limits.

## If the robot shakes / vibrates
 1. Run the node with `-p engage_policy:=false`. If the standing pose already shakes, the problem is Gazebo / the
    controller, not the policy (check `ros2 topic hz /joint_states` and `/imu`, both should be >= 50 Hz).
 2. If it only shakes once the policy is in control: continue training with the v5 environment (README_TRAINING.md,
    section 10) and run the new model with `-p legacy_gyro:=false`.
