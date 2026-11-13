#!/usr/bin/env python3
"""Move one selected foot in X/Y while holding its final height constant."""

import math
import os
import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener, TransformException

from control_library import leg_ik
from open_quadruped.msg import JointAngles


LEGS = ('fl', 'fr', 'bl', 'br')
LEG_JOINTS = {
    'fl': ('motor_front_left_hip', 'motor_front_left_upper_leg', 'motor_front_left_lower_leg'),
    'fr': ('motor_front_right_hip', 'motor_front_right_upper_leg', 'motor_front_right_lower_leg'),
    'bl': ('motor_back_left_hip', 'motor_back_left_upper_leg', 'motor_back_left_lower_leg'),
    'br': ('motor_back_right_hip', 'motor_back_right_upper_leg', 'motor_back_right_lower_leg'),
}
FOOT_FRAMES = {
    'fl': 'front_left_foot', 'fr': 'front_right_foot',
    'bl': 'back_left_foot', 'br': 'back_right_foot',
}
# Joint origins in base_link, metres. These match the four hip joint origins
# in spot.urdf.xacro. The IK vector is hip origin minus foot target (mm),
# matching BodyIKModel.get_htf_vectors()'s body-to-foot convention.
HIP_ORIGINS = {
    'fl': (0.0915, 0.0394, 0.022),
    'fr': (0.0915, -0.0394, 0.022),
    'bl': (-0.1365, 0.0394, 0.022),
    'br': (-0.1365, -0.0394, 0.022),
}


class KeyboardFootControl(Node):
    def __init__(self):
        super().__init__('keyboard_foot_control')
        self.declare_parameter('gain_m', 0.01)
        self.declare_parameter('z_lift_m', 0.02)
        self.declare_parameter('move_duration_sec', 1.0)
        self.declare_parameter('publish_rate_hz', 30.0)
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('joint_angles_topic', '/joint_angles')
        self.declare_parameter('joint_states_topic', '/joint_states')

        self.gain = float(self.get_parameter('gain_m').value)
        self.lift = float(self.get_parameter('z_lift_m').value)
        self.duration = float(self.get_parameter('move_duration_sec').value)
        rate = float(self.get_parameter('publish_rate_hz').value)
        for name, value in (('gain_m', self.gain), ('z_lift_m', self.lift),
                            ('move_duration_sec', self.duration), ('publish_rate_hz', rate)):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f'{name} must be a positive finite number')

        self.base_frame = str(self.get_parameter('base_frame').value)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.ik = leg_ik.LegIKModel(109.868, 144.580, 11.369, 63.763)
        self.publisher = self.create_publisher(
            JointAngles, str(self.get_parameter('joint_angles_topic').value), 10
        )
        self.joint_state = None
        self.joint_state_sub = self.create_subscription(
            JointState, str(self.get_parameter('joint_states_topic').value),
            self._on_joint_state, 10
        )
        self.targets = {}
        self.home_targets = {}
        self.home_ik = {}
        self.home_joints = {}
        self.has_commands = False
        self.path = None
        self.selected = 0
        self.timer = self.create_timer(1.0 / rate, self._publish_pose)

    def _on_joint_state(self, msg):
        values = dict(zip(msg.name, msg.position))
        if all(name in values for names in LEG_JOINTS.values() for name in names):
            self.joint_state = {name: float(values[name]) for names in LEG_JOINTS.values() for name in names}

    def _read_feet(self):
        feet = {}
        for leg, frame in FOOT_FRAMES.items():
            transform = self.tf_buffer.lookup_transform(self.base_frame, frame, Time())
            point = transform.transform.translation
            feet[leg] = [float(point.x), float(point.y), float(point.z)]
        return feet

    def _sample_path(self):
        if self.path is None:
            return
        leg, start, end, start_time = self.path
        elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
        t = max(0.0, min(1.0, elapsed / self.duration))
        u = t * t * (3.0 - 2.0 * t)  # smooth start/stop
        point = [start[i] + (end[i] - start[i]) * u for i in range(3)]
        point[2] = start[2] + self.lift * math.sin(math.pi * u)
        self.targets[leg] = point
        if t >= 1.0:
            # Exact destination, including exactly the starting Z.
            self.targets[leg] = list(end)
            self.path = None

    def move_selected(self, axis, direction):
        try:
            # Use measured current feet for the beginning of each requested move.
            self.targets.update(self._read_feet())
        except TransformException as exc:
            self.get_logger().warning(f'Waiting for foot transforms: {exc}')
            return
        leg = LEGS[self.selected]
        start = list(self.targets[leg])
        end = list(start)
        end[axis] += direction * self.gain
        self.path = (leg, start, end, self.get_clock().now())
        self.has_commands = True
        axis_name = 'X' if axis == 0 else 'Y'
        self.get_logger().info(
            f'{leg.upper()} target {axis_name}{direction * self.gain:+.3f} m; '
            f'Z stays {start[2]:.3f} m at the destination.'
        )

    def _publish_pose(self):
        try:
            if len(self.targets) != len(LEGS) or not self.home_joints:
                if self.joint_state is None:
                    return
                feet = self._read_feet()
                home_ik = {}
                for leg in LEGS:
                    hx, hy, hz = HIP_ORIGINS[leg]
                    fx, fy, fz = feet[leg]
                    vector = ((hx - fx) * 1000.0, (hy - fy) * 1000.0, (hz - fz) * 1000.0)
                    home_ik[leg] = self.ik.ja_from_htf_vecs([vector])[0]
                self.targets = {leg: list(point) for leg, point in feet.items()}
                self.home_targets = {leg: list(point) for leg, point in feet.items()}
                self.home_ik = home_ik
                self.home_joints = {
                    leg: [self.joint_state[name] for name in LEG_JOINTS[leg]] for leg in LEGS
                }
                self.get_logger().info('Foot TF and joint feedback ready; no motion until a key is pressed.')
            was_moving = self.path is not None
            self._sample_path()
            if not self.has_commands:
                return
            joint_matrix = {}
            for leg in LEGS:
                hx, hy, hz = HIP_ORIGINS[leg]
                fx, fy, fz = self.targets[leg]
                vector_mm = ((hx - fx) * 1000.0, (hy - fy) * 1000.0, (hz - fz) * 1000.0)
                requested = self.ik.ja_from_htf_vecs([vector_mm])[0]
                delta = [requested[i] - self.home_ik[leg][i] for i in range(3)]
                joint_matrix[leg] = [self.home_joints[leg][i] + delta[i] for i in range(3)]
            # Check that the IK offset still respects each model joint's limits.
            limits = ((-1.04, 1.04), (-math.pi / 2, 2.59), (-2.9, math.pi / 2))
            for leg in LEGS:
                if any(value < lo or value > hi for value, (lo, hi) in zip(joint_matrix[leg], limits)):
                    raise ValueError(f'{leg.upper()} target would exceed a joint limit')
        except (TransformException, ArithmeticError, ValueError) as exc:
            self.get_logger().warning(f'No IK command published for this update: {exc}')
            return

        # Send during a path and once at its endpoint; the controller then holds
        # the last trajectory target until the next keyboard command.
        if not was_moving:
            return

        msg = JointAngles()
        msg.fl = [float(v) for v in joint_matrix['fl']]
        msg.fr = [float(v) for v in joint_matrix['fr']]
        msg.bl = [float(v) for v in joint_matrix['bl']]
        msg.br = [float(v) for v in joint_matrix['br']]
        self.publisher.publish(msg)


def read_key():
    ready, _, _ = select.select([sys.stdin], [], [], 0.02)
    if not ready:
        return None
    first = os.read(sys.stdin.fileno(), 1)
    if first != b'\x1b':
        return first.decode(errors='ignore')
    if not select.select([sys.stdin], [], [], 0.02)[0]:
        return 'escape'
    os.read(sys.stdin.fileno(), 1)  # '['
    tail = os.read(sys.stdin.fileno(), 1)
    return {b'A': 'up', b'B': 'down', b'C': 'right', b'D': 'left'}.get(tail, 'escape')


def main(args=None):
    if not sys.stdin.isatty():
        raise RuntimeError('Run this node from an interactive terminal.')
    rclpy.init(args=args)
    node = KeyboardFootControl()
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        print('\r\nSelect leg: 1=FL 2=FR 3=BL 4=BR | arrows or WASD move in X/Y | q quits\r')
        print(f'Gain: {node.gain:.3f} m per press; path lift: {node.lift:.3f} m\r')
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            key = read_key()
            if key == 'q':
                break
            if key in ('1', '2', '3', '4'):
                node.selected = int(key) - 1
                print(f'\rSelected leg: {LEGS[node.selected].upper()}          \r')
            elif key == 'right':
                node.move_selected(0, 1)
            elif key == 'left':
                node.move_selected(0, -1)
            elif key == 'up':
                node.move_selected(1, 1)
            elif key == 'down':
                node.move_selected(1, -1)
            elif key in ('d', 'D'):
                node.move_selected(0, 1)
            elif key in ('a', 'A'):
                node.move_selected(0, -1)
            elif key in ('w', 'W'):
                node.move_selected(1, 1)
            elif key in ('s', 'S'):
                node.move_selected(1, -1)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
