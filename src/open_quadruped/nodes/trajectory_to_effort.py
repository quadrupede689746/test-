#!/usr/bin/env python3
"""Apply bounded PD effort commands for incoming joint trajectories."""

import math
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory


JOINT_ORDER = (
    'motor_front_left_hip', 'motor_front_left_upper_leg', 'motor_front_left_lower_leg',
    'motor_front_right_hip', 'motor_front_right_upper_leg', 'motor_front_right_lower_leg',
    'motor_back_left_hip', 'motor_back_left_upper_leg', 'motor_back_left_lower_leg',
    'motor_back_right_hip', 'motor_back_right_upper_leg', 'motor_back_right_lower_leg',
)


class TrajectoryToEffort(Node):
    def __init__(self):
        super().__init__('trajectory_to_effort')
        self.declare_parameter('trajectory_topic', '/quadruped_controller/joint_trajectory')
        self.declare_parameter('effort_topic', '/quadruped_controller/commands')
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('publish_rate_hz', 100.0)
        self.declare_parameter('kp_nm_per_rad', 15.0)
        self.declare_parameter('kd_nm_per_rad_sec', 0.5)
        self.declare_parameter('max_effort_nm', 4.0)
        self.declare_parameter('effort_slew_rate_nm_sec', 40.0)

        rate = float(self.get_parameter('publish_rate_hz').value)
        self.kp = float(self.get_parameter('kp_nm_per_rad').value)
        self.kd = float(self.get_parameter('kd_nm_per_rad_sec').value)
        self.max_effort = float(self.get_parameter('max_effort_nm').value)
        self.slew_rate = float(self.get_parameter('effort_slew_rate_nm_sec').value)
        for name, value in (
            ('publish_rate_hz', rate), ('kp_nm_per_rad', self.kp),
            ('kd_nm_per_rad_sec', self.kd), ('max_effort_nm', self.max_effort),
            ('effort_slew_rate_nm_sec', self.slew_rate),
        ):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f'{name} must be a positive finite number')

        self.position = None
        self.velocity = None
        self.target = None
        self.output = [0.0] * len(JOINT_ORDER)
        self.last_update = time.monotonic()
        self.warned_missing_feedback = False

        self.effort_pub = self.create_publisher(
            Float64MultiArray, str(self.get_parameter('effort_topic').value), 10
        )
        self.trajectory_sub = self.create_subscription(
            JointTrajectory, str(self.get_parameter('trajectory_topic').value),
            self._on_trajectory, 10,
        )
        self.joint_state_sub = self.create_subscription(
            JointState, str(self.get_parameter('joint_states_topic').value),
            self._on_joint_state, 10,
        )
        self.timer = self.create_timer(
            1.0 / rate, self._update, clock=Clock(clock_type=ClockType.STEADY_TIME)
        )
        self.get_logger().info(
            f'Bounded PD effort control: kp={self.kp:.2f}, kd={self.kd:.3f}, '
            f'torque limit=±{self.max_effort:.2f} Nm.'
        )

    def _on_trajectory(self, msg: JointTrajectory):
        if not msg.points or not msg.points[-1].positions:
            return
        names = list(msg.joint_names)
        values = list(msg.points[-1].positions)
        if len(names) != len(values):
            self.get_logger().error('Ignoring trajectory with mismatched joint_names and positions.')
            return
        mapping = dict(zip(names, values))
        if any(name not in mapping for name in JOINT_ORDER):
            self.get_logger().error('Ignoring trajectory; it must command the 12 named joints.')
            return
        target = [float(mapping[name]) for name in JOINT_ORDER]
        if not all(math.isfinite(value) for value in target):
            self.get_logger().error('Ignoring trajectory with NaN or infinite positions.')
            return
        self.target = target

    def _on_joint_state(self, msg: JointState):
        positions = dict(zip(msg.name, msg.position))
        velocities = dict(zip(msg.name, msg.velocity))
        if any(name not in positions or name not in velocities for name in JOINT_ORDER):
            return
        position = [float(positions[name]) for name in JOINT_ORDER]
        velocity = [float(velocities[name]) for name in JOINT_ORDER]
        if all(math.isfinite(v) for v in position + velocity):
            self.position = position
            self.velocity = velocity

    def _update(self):
        now = time.monotonic()
        dt = max(1e-4, min(0.05, now - self.last_update))
        self.last_update = now
        if self.position is None or self.velocity is None or self.target is None:
            return
        max_delta = self.slew_rate * dt
        efforts = []
        for index, (position, velocity, target) in enumerate(
            zip(self.position, self.velocity, self.target)
        ):
            requested = self.kp * (target - position) - self.kd * velocity
            requested = max(-self.max_effort, min(self.max_effort, requested))
            change = max(-max_delta, min(max_delta, requested - self.output[index]))
            self.output[index] += change
            efforts.append(self.output[index])
        self.effort_pub.publish(Float64MultiArray(data=efforts))


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryToEffort()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
