#!/usr/bin/env python3
"""Test a slow 1 cm body-lowering pose while all four foot contacts remain planted."""
import sys
import time

import rclpy
from control_msgs.msg import JointTrajectoryControllerState
from gazebo_msgs.msg import ContactsState
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from keyboard_walk import FOOT_LINKS, JOINT_NAMES, LEGS, UrdfLegKinematics

LEG_NAMES = {'fl': 'front_left', 'fr': 'front_right', 'bl': 'back_left', 'br': 'back_right'}


class StanceBendTest(Node):
    def __init__(self):
        super().__init__('stance_bend_test')
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.declare_parameter('body_lowering_m', 0.01)
        self.declare_parameter('duration_sec', 10.0)
        self.declare_parameter('wait_timeout_sec', 30.0)
        self.lowering = float(self.get_parameter('body_lowering_m').value)
        self.duration = float(self.get_parameter('duration_sec').value)
        self.timeout = float(self.get_parameter('wait_timeout_sec').value)
        self.kinematics = UrdfLegKinematics()
        self.positions = None
        self.contacts = {leg: None for leg in LEGS}
        self.controller = None
        self.safety_abort = False
        self.pub = self.create_publisher(JointTrajectory, '/quadruped_controller/joint_trajectory', 10)
        self.status = self.create_publisher(String, '/keyboard_walk/status', 10)
        self.create_subscription(JointState, '/joint_states', self._joint_cb, 10)
        self.create_subscription(JointTrajectoryControllerState,
                                 '/quadruped_controller/controller_state', self._controller_cb, 10)
        self.create_subscription(String, '/robot_logger/safety_stop', self._safety_cb, 10)
        self.contact_subs = [self.create_subscription(
            ContactsState, f'/foot_contacts/{FOOT_LINKS[leg]}/bumper_states',
            lambda msg, leg=leg: self._contact_cb(leg, msg), 10) for leg in LEGS]

    def _joint_cb(self, msg):
        values = dict(zip(msg.name, msg.position))
        if all(name in values for name in JOINT_NAMES):
            self.positions = {name: float(values[name]) for name in JOINT_NAMES}

    def _controller_cb(self, msg):
        self.controller = msg

    def _contact_cb(self, leg, msg):
        self.contacts[leg] = bool(msg.states)

    def _safety_cb(self, msg):
        self.safety_abort = True
        if self.positions:
            self._publish([self.positions[name] for name in JOINT_NAMES], 1.0)
        print(f'Safety event {msg.data}; aborting stance bend and holding measured joints.', flush=True)

    def _publish(self, positions, duration):
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = JOINT_NAMES
        point = JointTrajectoryPoint()
        point.positions = positions
        sec = int(duration)
        point.time_from_start.sec = sec
        point.time_from_start.nanosec = int((duration - sec) * 1e9)
        msg.points = [point]
        self.pub.publish(msg)

    def run_test(self):
        if self.lowering <= 0 or self.duration <= 0:
            raise ValueError('body_lowering_m and duration_sec must be positive')
        deadline = time.monotonic() + self.timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.positions and self.controller and all(v is not None for v in self.contacts.values()):
                break
        if not self.positions or self.controller is None or any(v is None for v in self.contacts.values()):
            raise RuntimeError('Timed out waiting for joint, controller, or contact feedback.')
        if not all(self.contacts.values()):
            raise RuntimeError(f'All feet must be in contact before a stance bend: {self.contacts}')

        home = {leg: [self.positions[name] for name in self.kinematics.chains[leg][1]] for leg in LEGS}
        targets = {}
        for leg in LEGS:
            target = self.kinematics.fk(leg, home[leg])
            target[2] += self.lowering  # planted feet rise in body coordinates as the body lowers
            targets[leg] = self.kinematics.ik(leg, target, home[leg])
        result = dict(self.positions)
        for leg in LEGS:
            result.update(dict(zip(self.kinematics.chains[leg][1], targets[leg])))
        q_goal = [result[name] for name in JOINT_NAMES]
        delta = max(abs(result[name] - self.positions[name]) for name in JOINT_NAMES)
        print(f'All 4 foot contacts confirmed; sending {self.lowering:.3f} m body lowering over {self.duration:.1f} s; max joint change={delta:.3f} rad.', flush=True)
        self.status.publish(String(data=f'key=stance_bend_test;body_lowering_m={self.lowering:.4f};vx=0;vy=0;yaw=0'))
        self._publish(q_goal, self.duration)
        until = time.monotonic() + self.duration + 5.0
        while rclpy.ok() and time.monotonic() < until and not self.safety_abort:
            rclpy.spin_once(self, timeout_sec=0.05)
        self.status.publish(String(data='key=stance_bend_complete;vx=0;vy=0;yaw=0'))


def main(args=None):
    rclpy.init(args=args)
    node = StanceBendTest()
    try:
        node.run_test()
    except Exception as exc:
        print(f'Stance-bend test aborted: {exc}', file=sys.stderr, flush=True)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
