#!/usr/bin/env python3
"""
ROS 2 (Humble) node: runs a trained policy.npz on the Gazebo robot.

  subscribes  /joint_states (JointState)   /imu (sensor_msgs/Imu)   /cmd_vel (Twist)   /dog_mode (String: walk|stand|lie)
  publishes   /quadruped_controller/joint_trajectory (JointTrajectory, 12 joints, 50 Hz)

Run (policy.npz only needs numpy):
  python3 gazebo_policy_node.py --ros-args -p policy:=/full/path/policy.npz -p use_sim_time:=true
Parameters: policy, legacy_gyro (true for models trained with v1-v3, false for v4+), engage_policy (false = only stand up),
            rate (50), traj_time (0.04 s), cmd_timeout (1.0 s, 0 = off), min_upright (0.35)
"""
import os, sys
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from dog_controller import DogController

JOINT_ORDER = [f"motor_{leg}_{part}" for leg in ("front_left", "front_right", "back_left", "back_right")
               for part in ("hip", "upper_leg", "lower_leg")]


class PolicyNode(Node):
    def __init__(self):
        super().__init__("rl_policy_node")
        self.declare_parameter("policy", "")
        self.declare_parameter("legacy_gyro", True)
        self.declare_parameter("engage_policy", True)
        self.declare_parameter("rate", 50.0)
        self.declare_parameter("traj_time", 0.04)
        self.declare_parameter("cmd_timeout", 1.0)
        self.declare_parameter("min_upright", 0.35)
        self.declare_parameter("trajectory_topic", "/quadruped_controller/joint_trajectory")
        path = os.path.expanduser(self.get_parameter("policy").value)
        if not os.path.isfile(path):
            raise SystemExit(f"policy file not found: '{path}'  ->  use -p policy:=/full/path/policy.npz")
        self.ctrl = DogController(path, legacy_gyro=bool(self.get_parameter("legacy_gyro").value))
        self.engage = bool(self.get_parameter("engage_policy").value)
        self.rate = float(self.get_parameter("rate").value)
        self.traj_time = float(self.get_parameter("traj_time").value)
        self.cmd_timeout = float(self.get_parameter("cmd_timeout").value)
        self.min_upright = float(self.get_parameter("min_upright").value)
        self.q = self.qd = None; self.quat = self.gyro = None
        self.cmd = np.zeros(3); self.cmd_time = None; self.mode = "walk"; self.fallen = False; self.was_engaged = False
        self.pub = self.create_publisher(JointTrajectory, self.get_parameter("trajectory_topic").value, 10)
        self.create_subscription(JointState, "/joint_states", self.on_joints, 10)
        self.create_subscription(Imu, "/imu", self.on_imu, qos_profile_sensor_data)
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd, 10)
        self.create_subscription(String, "/dog_mode", self.on_mode, 10)
        self.create_timer(1.0 / self.rate, self.tick)
        self.get_logger().info(f"policy loaded: {path} (legacy_gyro={self.ctrl.legacy_gyro}). Waiting for /joint_states and /imu ...")

    # ---------------------------------------------------------------- inputs
    def on_joints(self, m):
        idx = {n: i for i, n in enumerate(m.name)}
        if not all(j in idx for j in JOINT_ORDER) or len(m.velocity) < len(m.name):
            return
        self.q = np.array([m.position[idx[j]] for j in JOINT_ORDER])
        self.qd = np.array([m.velocity[idx[j]] for j in JOINT_ORDER])

    def on_imu(self, m):
        o = m.orientation; quat = np.array([o.w, o.x, o.y, o.z])
        if np.linalg.norm(quat) < 0.5:       # orientation not provided -> estimate roll/pitch from the accelerometer
            a = m.linear_acceleration; roll = np.arctan2(a.y, a.z); pitch = np.arctan2(-a.x, np.hypot(a.y, a.z))
            cr, sr, cp, sp = np.cos(roll/2), np.sin(roll/2), np.cos(pitch/2), np.sin(pitch/2)
            quat = np.array([cr*cp, sr*cp, cr*sp, -sr*sp])
        self.quat = quat; self.gyro = np.array([m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z])

    def on_cmd(self, m):
        self.cmd = np.array([m.linear.x, m.linear.y, m.angular.z]); self.cmd_time = self.get_clock().now()

    def on_mode(self, m):
        if m.data in ("walk", "stand", "lie"):
            self.mode = m.data; self.get_logger().info(f"mode -> {m.data}")
        else:
            self.get_logger().warn("mode must be walk, stand or lie")

    # ---------------------------------------------------------------- control loop
    def tick(self):
        if self.q is None or self.quat is None:
            self.get_logger().info("waiting for /joint_states and /imu ...", throttle_duration_sec=5.0); return
        cmd = self.cmd
        if self.cmd_timeout > 0 and (self.cmd_time is None or (self.get_clock().now() - self.cmd_time).nanoseconds * 1e-9 > self.cmd_timeout):
            cmd = np.zeros(3)
        if self.mode == "walk":
            self.ctrl.set_command(*cmd, mode="walk")
        else:
            self.ctrl.set_command(0, 0, 0, mode=self.mode)
        if self.ctrl.engaged and self.ctrl.upright(self.quat) < self.min_upright:
            if not self.fallen:
                self.get_logger().error("robot is tipped over - policy output stopped (restart the node after resetting the robot)")
            self.fallen = True; return
        if not self.engage:
            self.ctrl.engaged = False
        target = self.ctrl.update(self.q, self.qd, self.quat, self.gyro, dt=1.0 / self.rate)
        if self.ctrl.engaged and not self.was_engaged:
            self.get_logger().info("stand-up finished: the policy is now in control")
        self.was_engaged = self.ctrl.engaged
        msg = JointTrajectory(); msg.joint_names = JOINT_ORDER
        pt = JointTrajectoryPoint(); pt.positions = [float(x) for x in target]
        pt.time_from_start.sec = int(self.traj_time); pt.time_from_start.nanosec = int((self.traj_time % 1) * 1e9)
        msg.points = [pt]; self.pub.publish(msg)


def main():
    rclpy.init(); node = PolicyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
