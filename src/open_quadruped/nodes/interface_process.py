#!/usr/bin/env python3
"""Convert joystick body-pose commands into quadruped joint-angle targets.

The source ROS 1 package selected a gait mode with Joy button 7, but its gait
branch never computed a foot trajectory.  This ROS 2 port therefore fails
closed in gait mode rather than publishing undefined/stale joint angles.
Body-pose inverse kinematics is fully retained.
"""

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Joy

from control_library import body_ik, leg_ik
from open_quadruped.msg import JointAngles


class InterfaceProcess(Node):
    def __init__(self) -> None:
        super().__init__('interface_process')

        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('joy_topic', 'joy')
        self.declare_parameter('joint_angles_topic', 'joint_angles')
        self.declare_parameter('gait_mode_button', 7)
        self.declare_parameter('yaw_limit_deg', 15.0)
        self.declare_parameter('pitch_limit_deg', 15.0)
        self.declare_parameter('roll_limit_deg', 15.0)

        rate_hz = float(self.get_parameter('publish_rate_hz').value)
        if not math.isfinite(rate_hz) or rate_hz <= 0.0:
            raise ValueError('publish_rate_hz must be a positive finite number')

        self.gait_mode_button = int(self.get_parameter('gait_mode_button').value)
        self.yaw_limit = float(self.get_parameter('yaw_limit_deg').value)
        self.pitch_limit = float(self.get_parameter('pitch_limit_deg').value)
        self.roll_limit = float(self.get_parameter('roll_limit_deg').value)

        self.leg_model = leg_ik.LegIKModel(109.868, 144.580, 11.369, 63.763)
        self.body_model = body_ik.BodyIKModel(76.655, 229.3, 130.0)
        self.latest_joy: Optional[Joy] = None
        self._warned_gait = False

        joy_topic = str(self.get_parameter('joy_topic').value)
        joint_topic = str(self.get_parameter('joint_angles_topic').value)
        self.joy_sub = self.create_subscription(
            Joy, joy_topic, self._joy_callback, qos_profile_sensor_data
        )
        self.joint_pub = self.create_publisher(JointAngles, joint_topic, 10)
        self.timer = self.create_timer(1.0 / rate_hz, self._publish_joint_angles)

        self.get_logger().info(
            f'Listening on {joy_topic}; publishing body IK targets on {joint_topic}. '
            f'Hold Joy button {self.gait_mode_button} to select gait mode (not implemented).'
        )

    def _joy_callback(self, msg: Joy) -> None:
        self.latest_joy = msg

    def _publish_joint_angles(self) -> None:
        joy = self.latest_joy
        if joy is None:
            return

        if self.gait_mode_button < len(joy.buttons) and joy.buttons[self.gait_mode_button]:
            if not self._warned_gait:
                self.get_logger().warning(
                    'Gait mode is not implemented in the supplied upstream node; '
                    'no new joint command will be published while the button is held.'
                )
                self._warned_gait = True
            return
        self._warned_gait = False

        if len(joy.axes) <= 5:
            self.get_logger().warning('Joy message has fewer than 6 axes; waiting for a complete joystick message.')
            return

        yaw = float(joy.axes[2]) * self.yaw_limit
        pitch = float(joy.axes[5]) * self.pitch_limit
        roll = float(joy.axes[0]) * self.roll_limit

        try:
            self.body_model.reset_pose()
            self.body_model.transform(
                math.radians(yaw), math.radians(pitch), math.radians(roll)
            )
            htf_vectors = self.body_model.get_htf_vectors()
            joint_matrix = self.leg_model.ja_from_htf_vecs(htf_vectors)
            if len(joint_matrix) != 4 or any(len(leg) != 3 for leg in joint_matrix):
                raise ValueError('IK did not return exactly three angles for each of four legs')
            if not all(math.isfinite(float(value)) for leg in joint_matrix for value in leg):
                raise ValueError('IK returned a non-finite joint angle')
        except (ArithmeticError, ValueError) as exc:
            self.get_logger().warning(f'Cannot solve requested body pose: {exc}')
            return

        msg = JointAngles()
        msg.fl = [float(v) for v in joint_matrix[0]]
        msg.fr = [float(v) for v in joint_matrix[1]]
        msg.bl = [float(v) for v in joint_matrix[2]]
        msg.br = [float(v) for v in joint_matrix[3]]
        self.joint_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = InterfaceProcess()
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
