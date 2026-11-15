#!/usr/bin/env python3
"""Lift one supported foot 1 cm over 10 simulated seconds after checking support."""
import math
import sys
import time

import numpy as np
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
NAME_LEGS = {value: key for key, value in LEG_NAMES.items()}


class SingleFootTest(Node):
    def __init__(self):
        super().__init__('single_foot_test')
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.declare_parameter('lift_leg', 'back_left')
        self.declare_parameter('lift_m', 0.01)
        self.declare_parameter('duration_sec', 10.0)
        self.declare_parameter('support_margin_min_m', 0.005)
        self.declare_parameter('wait_timeout_sec', 30.0)
        requested_leg = str(self.get_parameter('lift_leg').value)
        self.lift_leg = NAME_LEGS.get(requested_leg, requested_leg)
        self.lift_leg_name = LEG_NAMES.get(self.lift_leg, requested_leg)
        self.lift_m = float(self.get_parameter('lift_m').value)
        self.duration = float(self.get_parameter('duration_sec').value)
        self.margin_min = float(self.get_parameter('support_margin_min_m').value)
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
        vals = dict(zip(msg.name, msg.position))
        if all(name in vals for name in JOINT_NAMES):
            self.positions = {name: float(vals[name]) for name in JOINT_NAMES}

    def _controller_cb(self, msg):
        self.controller = msg

    def _safety_cb(self, msg):
        self.safety_abort = True
        if self.positions:
            trajectory = JointTrajectory()
            trajectory.header.stamp = self.get_clock().now().to_msg()
            trajectory.joint_names = JOINT_NAMES
            point = JointTrajectoryPoint()
            point.positions = [self.positions[name] for name in JOINT_NAMES]
            point.time_from_start.sec = 1
            trajectory.points = [point]
            self.pub.publish(trajectory)
        print(f'Safety event {msg.data}; aborting single-foot test and holding measured joints.', flush=True)

    def _contact_cb(self, leg, msg):
        self.contacts[leg] = bool(msg.states)

    @staticmethod
    def _inside_margin(points, point):
        signs, margins = [], []
        for i, a in enumerate(points):
            b = points[(i + 1) % len(points)]
            edge = b - a
            length = float(np.linalg.norm(edge))
            if length < 1e-9:
                return False, -math.inf
            cross = float(edge[0] * (point[1] - a[1]) - edge[1] * (point[0] - a[0]))
            signs.append(cross)
            margins.append(abs(cross) / length)
        inside = all(v >= -1e-8 for v in signs) or all(v <= 1e-8 for v in signs)
        return inside, min(margins) if inside else -min(margins)

    def run_test(self):
        if self.lift_leg not in LEGS:
            raise ValueError(f'lift_leg must be one of {tuple(LEG_NAMES.values())}')
        if self.lift_m <= 0 or self.duration <= 0:
            raise ValueError('lift_m and duration_sec must be positive')
        deadline = time.monotonic() + self.timeout
        supporting = [leg for leg in LEGS if leg != self.lift_leg]
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.positions and self.controller and all(self.contacts[x] is not None for x in LEGS):
                break
        if not self.positions:
            raise RuntimeError('Timed out waiting for /joint_states.')
        if self.controller is None:
            raise RuntimeError('Timed out waiting for controller state.')
        if any(self.contacts[leg] is None for leg in LEGS):
            raise RuntimeError('Timed out waiting for foot contact sensor messages.')
        if not all(self.contacts[leg] for leg in supporting):
            raise RuntimeError(f'Support feet are not all on the ground: {self.contacts}')
        if not self.contacts[self.lift_leg]:
            raise RuntimeError(f'{self.lift_leg} is not in ground contact before the test.')

        home = {leg: [self.positions[name] for name in self.kinematics.chains[leg][1]] for leg in LEGS}
        feet = {leg: self.kinematics.fk(leg, home[leg])[:2] for leg in LEGS}
        polygon = [feet[leg] for leg in supporting]
        com = self.kinematics.com_xy
        inside, margin = self._inside_margin(polygon, com)
        print(f'COM={com.tolist()} m; support={[LEG_NAMES[x] for x in supporting]}; inside={inside}; edge_margin={margin:.5f} m', flush=True)
        if not inside or margin < self.margin_min:
            raise RuntimeError(f'Refusing foot lift: COM support margin {margin:.4f} m is below {self.margin_min:.4f} m.')

        target = self.kinematics.fk(self.lift_leg, home[self.lift_leg])
        target[2] += self.lift_m
        names = self.kinematics.chains[self.lift_leg][1]
        lifted = self.kinematics.ik(self.lift_leg, target, home[self.lift_leg])
        result = dict(self.positions)
        result.update(dict(zip(names, lifted)))
        ordered = [result[name] for name in JOINT_NAMES]
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = JOINT_NAMES
        point = JointTrajectoryPoint()
        point.positions = ordered
        sec = int(self.duration)
        point.time_from_start.sec = sec
        point.time_from_start.nanosec = int((self.duration - sec) * 1e9)
        msg.points = [point]
        self.status.publish(String(data=f'key=single_foot_test;lift_leg={self.lift_leg};lift_m={self.lift_m:.4f};vx=0;vy=0;yaw=0;cycle_duration_sec={self.duration:.3f}'))
        self.pub.publish(msg)
        print(f'Commanded {self.lift_leg_name} foot up {self.lift_m:.3f} m over {self.duration:.1f} simulated seconds; other joint targets held.', flush=True)
        finish = time.monotonic() + self.duration + 5.0
        while rclpy.ok() and time.monotonic() < finish and not self.safety_abort:
            rclpy.spin_once(self, timeout_sec=0.05)
        self.status.publish(String(data='key=single_foot_test_complete;lift_leg=none;vx=0;vy=0;yaw=0'))


def main(args=None):
    rclpy.init(args=args)
    node = SingleFootTest()
    try:
        node.run_test()
    except Exception as exc:
        print(f'Single-foot test aborted: {exc}', file=sys.stderr, flush=True)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
