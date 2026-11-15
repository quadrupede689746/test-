#!/usr/bin/env python3
"""Bridge the project JointAngles message to a ros2_control trajectory."""

import math

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from controller_manager_msgs.srv import ListControllers
from open_quadruped.msg import JointAngles
from std_srvs.srv import Empty
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
        self.declare_parameter('hold_initial_pose', True)
        self.declare_parameter('controller_manager', '/controller_manager')
        self.declare_parameter('initial_joint_positions', [0.0] * len(JOINT_ORDER))
        self.declare_parameter('startup_duration_sec', 1.0)
        self.declare_parameter('unpause_after_startup', True)

        self.duration = float(self.get_parameter('trajectory_duration_sec').value)
        if not math.isfinite(self.duration) or self.duration <= 0.0:
            raise ValueError('trajectory_duration_sec must be a positive finite number')
        self.startup_duration = float(self.get_parameter('startup_duration_sec').value)
        if not math.isfinite(self.startup_duration) or self.startup_duration <= 0.0:
            raise ValueError('startup_duration_sec must be a positive finite number')
        self.initial_positions = [
            float(value) for value in self.get_parameter('initial_joint_positions').value
        ]
        if len(self.initial_positions) != len(JOINT_ORDER) or not all(
            math.isfinite(value) for value in self.initial_positions
        ):
            raise ValueError('initial_joint_positions must contain 12 finite values in JOINT_ORDER')

        input_topic = str(self.get_parameter('input_topic').value)
        trajectory_topic = str(self.get_parameter('trajectory_topic').value)
        self.publisher = self.create_publisher(JointTrajectory, trajectory_topic, 10)
        self.subscription = self.create_subscription(
            JointAngles, input_topic, self._callback, 10
        )
        self.hold_initial_pose = bool(self.get_parameter('hold_initial_pose').value)
        controller_manager = str(self.get_parameter('controller_manager').value).rstrip('/')
        self.controller_client = self.create_client(
            ListControllers, f'{controller_manager}/list_controllers'
        )
        self.controller_request_pending = False
        self.controller_active = False
        self.unpause_after_startup = bool(self.get_parameter('unpause_after_startup').value)
        self.unpause_client = self.create_client(Empty, '/unpause_physics')
        self.unpause_request_pending = False
        self.unpaused = not self.unpause_after_startup
        self.startup_timer = self.create_timer(
            0.1,
            self._publish_startup_hold,
            clock=Clock(clock_type=ClockType.STEADY_TIME),
        )
        self.get_logger().info(
            f'Bridging {input_topic} to {trajectory_topic} ({len(JOINT_ORDER)} joints); '
            + ('commanding the configured initial pose after the controller becomes active.'
               if self.hold_initial_pose else 'startup pose hold is disabled.')
        )

    def _publish_startup_hold(self) -> None:
        if self.hold_initial_pose:
            if self.publisher.get_subscription_count() == 0:
                return
            if not self.controller_active:
                if self.controller_request_pending or not self.controller_client.service_is_ready():
                    return
                self.controller_request_pending = True
                future = self.controller_client.call_async(ListControllers.Request())
                future.add_done_callback(self._on_controllers)
                return
            self._publish_positions(self.initial_positions, self.startup_duration)
            self.hold_initial_pose = False
            self.get_logger().info(
                f'Commanded the configured initial joint pose over {self.startup_duration:.2f} s.'
            )
        if not self.unpaused:
            self._request_unpause()

    def _request_unpause(self) -> None:
        if self.unpause_request_pending or not self.unpause_client.service_is_ready():
            return
        self.unpause_request_pending = True
        future = self.unpause_client.call_async(Empty.Request())
        future.add_done_callback(self._on_unpaused)

    def _on_unpaused(self, future) -> None:
        self.unpause_request_pending = False
        try:
            future.result()
        except Exception as exc:  # noqa: BLE001 - retry on the next wall-timer tick
            self.get_logger().warning(f'Cannot unpause Gazebo yet: {exc}')
            return
        self.unpaused = True
        self.get_logger().info('Initial pose command is queued; Gazebo physics unpaused.')

    def _on_controllers(self, future) -> None:
        self.controller_request_pending = False
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001 - a failed service query is retried next tick
            self.get_logger().warning(f'Cannot query controller state yet: {exc}')
            return
        self.controller_active = any(
            controller.name == 'quadruped_controller'
            and controller.state == 'active'
            for controller in response.controller
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

        self.hold_initial_pose = False
        self._publish_positions(values)

    def _publish_positions(self, values, duration=None) -> None:
        trajectory = JointTrajectory()
        trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory.joint_names = JOINT_ORDER
        point = JointTrajectoryPoint()
        point.positions = values
        trajectory_duration = self.duration if duration is None else float(duration)
        whole_seconds = int(trajectory_duration)
        point.time_from_start.sec = whole_seconds
        point.time_from_start.nanosec = int(
            (trajectory_duration - whole_seconds) * 1_000_000_000
        )
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
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
