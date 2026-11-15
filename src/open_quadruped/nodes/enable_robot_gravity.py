#!/usr/bin/env python3
"""Enable Gazebo gravity after the robot controller holds its start pose.

Gazebo Classic starts simulating before ros2_control has completed controller
activation.  This node gates gravity during that short startup window, then
restores it on every link reported by the spawned model.
"""

import time

import rclpy
from controller_manager_msgs.srv import ListControllers
from gazebo_msgs.srv import GetLinkProperties, GetModelProperties, SetLinkProperties
from rclpy.clock import Clock, ClockType
from rclpy.node import Node


class EnableRobotGravity(Node):
    def __init__(self):
        super().__init__('enable_robot_gravity')
        self.declare_parameter('model_name', 'open_quadruped')
        self.declare_parameter('controller_manager', '/controller_manager')
        self.declare_parameter('settle_delay_sec', 1.0)
        self.model_name = str(self.get_parameter('model_name').value)
        manager = str(self.get_parameter('controller_manager').value).rstrip('/')
        self.settle_delay = max(0.0, float(self.get_parameter('settle_delay_sec').value))

        self.controllers = self.create_client(ListControllers, f'{manager}/list_controllers')
        self.get_model = self.create_client(GetModelProperties, '/gazebo/get_model_properties')
        self.get_link = self.create_client(GetLinkProperties, '/gazebo/get_link_properties')
        self.set_link = self.create_client(SetLinkProperties, '/gazebo/set_link_properties')
        self.busy = False
        self.started_at = None
        self.links = []
        self.link_index = 0
        self.completed = 0
        self.timer = self.create_timer(
            0.1, self._tick, clock=Clock(clock_type=ClockType.STEADY_TIME)
        )
        self.get_logger().info('Waiting for active joint controller before enabling gravity.')

    def _tick(self):
        if self.busy:
            return
        if self.started_at is None:
            if not self.controllers.service_is_ready():
                return
            self.busy = True
            future = self.controllers.call_async(ListControllers.Request())
            future.add_done_callback(self._on_controllers)
            return
        if time.monotonic() - self.started_at < self.settle_delay:
            return
        if not self.links:
            if not self.get_model.service_is_ready():
                return
            self.busy = True
            request = GetModelProperties.Request()
            request.model_name = self.model_name
            self.get_model.call_async(request).add_done_callback(self._on_model)
            return
        if self.link_index < len(self.links):
            if not self.get_link.service_is_ready() or not self.set_link.service_is_ready():
                return
            self.busy = True
            request = GetLinkProperties.Request()
            request.link_name = self.links[self.link_index]
            self.get_link.call_async(request).add_done_callback(self._on_link)
            return
        self.timer.cancel()
        self.get_logger().info(
            f'Gravity enabled on {self.completed}/{len(self.links)} Gazebo links.'
        )

    def _on_controllers(self, future):
        self.busy = False
        try:
            response = future.result()
        except Exception as exc:  # service may still be starting
            self.get_logger().warning(f'Controller state query failed: {exc}')
            return
        if not any(c.name == 'quadruped_controller' and c.state == 'active'
                   for c in response.controller):
            return
        self.started_at = time.monotonic()
        self.get_logger().info(
            f'Joint controller active; letting the initial pose settle for '
            f'{self.settle_delay:.2f} s before restoring gravity.'
        )

    def _on_model(self, future):
        self.busy = False
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().warning(f'Gazebo model query failed: {exc}')
            return
        if not response.success:
            self.get_logger().warning(f'Gazebo model query failed: {response.status_message}')
            return
        self.links = list(response.body_names)
        if not self.links:
            self.get_logger().error('Gazebo reported no links for the robot model.')
            return
        self.get_logger().info(f'Restoring gravity on {len(self.links)} model links.')

    def _on_link(self, future):
        try:
            response = future.result()
        except Exception as exc:
            self.busy = False
            self.get_logger().warning(f'Gazebo link query failed: {exc}')
            return
        if not response.success:
            self.busy = False
            self.get_logger().warning(
                f"Cannot query {self.links[self.link_index]}: {response.status_message}"
            )
            self.link_index += 1
            return

        request = SetLinkProperties.Request()
        request.link_name = self.links[self.link_index]
        request.com = response.com
        request.gravity_mode = True
        request.mass = response.mass
        request.ixx = response.ixx
        request.ixy = response.ixy
        request.ixz = response.ixz
        request.iyy = response.iyy
        request.iyz = response.iyz
        request.izz = response.izz
        self.set_link.call_async(request).add_done_callback(self._on_set_link)

    def _on_set_link(self, future):
        self.busy = False
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().warning(f'Gazebo gravity update failed: {exc}')
        else:
            if response.success:
                self.completed += 1
            else:
                self.get_logger().warning(f'Gazebo gravity update failed: {response.status_message}')
        self.link_index += 1


def main(args=None):
    rclpy.init(args=args)
    node = EnableRobotGravity()
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
