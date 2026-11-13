#!/usr/bin/env python3
"""Bridge the project JointAngles message to a ros2_control trajectory."""

import math

import rclpy
from rclpy.node import Node
from open_quadruped.msg import JointAngles
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


LEG_JOINTS = {
    'fl': ['motor_front_left_hip', 'motor_front_left_upper_leg', 'motor_front_left_lower_leg'],
    'fr': ['motor_front_right_hip', 'motor_front_right_upper_leg', 'motor_front_right_lower_leg'],
    'bl': ['motor_back_left_hip', 'motor_back_left_upper_leg', 'motor_back_left_lower_leg'],
    'br': ['motor_back_right_hip', 'motor_back_right_upper_leg', 'motor_back_right_lower_leg'],
}
JOINT_ORDER = [name for leg in ('fl', 'fr', 'bl', 'br') for name in LEG_JOINTS[leg]]


class JointAnglesToTrajectory(Node):
    def __init__(self) -> None:
        super().__init__('joint_angles_to_trajectory')
        self.declare_parameter('input_topic', 'joint_angles')
        self.declare_parameter('trajectory_topic', '/quadruped_controller/joint_trajectory')
        self.declare_parameter('trajectory_duration_sec', 0.10)

        self.duration = float(self.get_parameter('trajectory_duration_sec').value)
        if not math.isfinite(self.duration) or self.duration <= 0.0:
            raise ValueError('trajectory_duration_sec must be a positive finite number')

        input_topic = str(self.get_parameter('input_topic').value)
        trajectory_topic = str(self.get_parameter('trajectory_topic').value)
        self.publisher = self.create_publisher(JointTrajectory, trajectory_topic, 10)
        self.subscription = self.create_subscription(
            JointAngles, input_topic, self._callback, 10
        )
        self.get_logger().info(
            f'Bridging {input_topic} to {trajectory_topic} ({len(JOINT_ORDER)} joints).'
        )

    def _callback(self, msg: JointAngles) -> None:
        values = []
        for field in ('fl', 'fr', 'bl', 'br'):
            leg_values = list(getattr(msg, field))
            if len(leg_values) != 3:
                self.get_logger().error(
                    f'Ignoring JointAngles: field {field} has {len(leg_values)} values; expected 3.'
                )
                return
            values.extend(float(v) for v in leg_values)

        if not all(math.isfinite(value) for value in values):
            self.get_logger().error('Ignoring JointAngles message containing NaN or infinity.')
            return

        trajectory = JointTrajectory()
        trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory.joint_names = JOINT_ORDER
        point = JointTrajectoryPoint()
        point.positions = values
        whole_seconds = int(self.duration)
        point.time_from_start.sec = whole_seconds
        point.time_from_start.nanosec = int((self.duration - whole_seconds) * 1_000_000_000)
        trajectory.points = [point]
        self.publisher.publish(trajectory)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = JointAnglesToTrajectory()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
