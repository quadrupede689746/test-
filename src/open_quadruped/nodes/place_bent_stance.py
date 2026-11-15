#!/usr/bin/env python3
"""Lower the body to stance height after startup joint commands have settled."""

import time

import rclpy
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import SetEntityState
from rclpy.node import Node
from sensor_msgs.msg import JointState

from keyboard_walk import UrdfLegKinematics


JOINTS = [
    f'motor_{leg}_{joint}'
    for leg in ('front_left', 'front_right', 'back_left', 'back_right')
    for joint in ('hip', 'upper_leg', 'lower_leg')
]


class PlaceBentStance(Node):
    def __init__(self):
        super().__init__('place_bent_stance')
        self.declare_parameter('model_name', 'open_quadruped')
        self.declare_parameter('joint_positions', [0.0] * len(JOINTS))
        self.declare_parameter('foot_radius_m', 0.02)
        self.declare_parameter('joint_tolerance_rad', 0.04)
        self.declare_parameter('stable_samples', 8)
        self.declare_parameter('wait_timeout_sec', 40.0)
        self.model_name = str(self.get_parameter('model_name').value)
        self.kinematics = UrdfLegKinematics()
        self.targets = [float(x) for x in self.get_parameter('joint_positions').value]
        if len(self.targets) != len(JOINTS):
            raise ValueError('joint_positions must contain 12 values in leg-major order.')
        self.foot_radius = float(self.get_parameter('foot_radius_m').value)
        self.tolerance = float(self.get_parameter('joint_tolerance_rad').value)
        self.stable_samples = int(self.get_parameter('stable_samples').value)
        self.timeout = float(self.get_parameter('wait_timeout_sec').value)
        self.start = time.monotonic()
        self.stable_count = 0
        self.pending = False
        self.finish_after_response = False
        self.done = False
        self.client = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        self.create_subscription(JointState, '/joint_states', self._joint_cb, 10)
        self.timer = self.create_timer(0.2, self._watchdog)
        self.get_logger().info(
            'Tracking measured foot positions and moving base height to keep the feet on the floor.'
        )

    def _watchdog(self):
        if not self.done and time.monotonic() - self.start > self.timeout:
            self.get_logger().error('Timed out waiting for the bent stance joint targets.')
            self.done = True

    def _joint_cb(self, msg):
        if self.done or self.pending:
            return
        positions = dict(zip(msg.name, msg.position))
        if not all(name in positions for name in JOINTS):
            return
        errors = [abs(float(positions[name]) - target)
                  for name, target in zip(JOINTS, self.targets)]
        self.stable_count = self.stable_count + 1 if max(errors) <= self.tolerance else 0
        if not self.client.service_is_ready():
            return
        leg_z = []
        for leg in self.kinematics.chains:
            names = self.kinematics.chains[leg][1]
            leg_z.append(float(self.kinematics.fk(
                leg, [float(positions[name]) for name in names]
            )[2]))
        base_height = self.foot_radius - min(leg_z)
        state = EntityState()
        state.name = self.model_name
        state.reference_frame = 'world'
        state.pose.position.z = base_height
        state.pose.orientation.w = 1.0
        request = SetEntityState.Request()
        request.state = state
        self.pending = True
        self.finish_after_response = self.stable_count >= self.stable_samples
        self.client.call_async(request).add_done_callback(self._on_placed)

    def _on_placed(self, future):
        self.pending = False
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(f'Gazebo stance placement failed: {exc}')
            self.done = True
            return
        if response.success:
            if self.finish_after_response:
                self.done = True
                self.get_logger().info(
                    'All joints settled; all four URDF foot centers are on the floor '
                    'and the bent stance is ready.'
                )
        else:
            self.get_logger().error('Gazebo rejected bent stance model placement.')
            self.done = True


def main(args=None):
    rclpy.init(args=args)
    node = PlaceBentStance()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
