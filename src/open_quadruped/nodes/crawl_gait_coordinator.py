#!/usr/bin/env python3
"""Keyboard-started, one-leg-at-a-time crawl gait coordinator.

The scheduler produces a slow stepping-in-place crawl for validating gait
timing and IK. Physical forward travel also needs working foot-ground contact
and a body/stance controller, neither of which is configured in this model.
"""

import math
import os
import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener, TransformException

from control_library import leg_ik
from open_quadruped.msg import JointAngles


LEGS = ('fl', 'fr', 'bl', 'br')
CRAWL_ORDER = ('fl', 'br', 'fr', 'bl')
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
HIP_ORIGINS = {
    'fl': (0.0915, 0.0394, 0.022), 'fr': (0.0915, -0.0394, 0.022),
    'bl': (-0.1365, 0.0394, 0.022), 'br': (-0.1365, -0.0394, 0.022),
}
JOINT_LIMITS = ((-1.04, 1.04), (-math.pi / 2, 2.59), (-2.9, math.pi / 2))


class CrawlGaitCoordinator(Node):
    def __init__(self):
        super().__init__('crawl_gait_coordinator')
        self.declare_parameter('step_length_m', 0.04)
        self.declare_parameter('swing_height_m', 0.025)
        self.declare_parameter('swing_duration_sec', 1.0)
        self.declare_parameter('dwell_duration_sec', 0.20)
        self.declare_parameter('publish_rate_hz', 30.0)
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('joint_angles_topic', '/joint_angles')
        self.declare_parameter('joint_states_topic', '/joint_states')

        self.step_length = float(self.get_parameter('step_length_m').value)
        self.swing_height = float(self.get_parameter('swing_height_m').value)
        self.swing_duration = float(self.get_parameter('swing_duration_sec').value)
        self.dwell_duration = float(self.get_parameter('dwell_duration_sec').value)
        rate = float(self.get_parameter('publish_rate_hz').value)
        for name, value in (
            ('step_length_m', self.step_length), ('swing_height_m', self.swing_height),
            ('swing_duration_sec', self.swing_duration), ('publish_rate_hz', rate),
        ):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f'{name} must be a positive finite number')
        if not math.isfinite(self.dwell_duration) or self.dwell_duration < 0.0:
            raise ValueError('dwell_duration_sec must be finite and non-negative')
        if self.step_length > 0.10 or self.swing_height > 0.08:
            raise ValueError('step_length_m must be <= 0.10 and swing_height_m <= 0.08 for this first test')

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
            self._on_joint_state, 10,
        )
        self.home_feet = {}
        self.targets = {}
        self.home_ik = {}
        self.home_joints = {}
        self.ready = False

        self.running = False
        self.stop_after_swing = False
        self.exit_after_swing = False
        self.sequence_index = 0
        self.leg_side = {leg: -1 for leg in LEGS}
        self.active_leg = None
        self.swing_start = None
        self.swing_end = None
        self.swing_started_at = None
        self.next_swing_at = None

        self.timer = self.create_timer(1.0 / rate, self._tick)

    def _on_joint_state(self, msg):
        values = dict(zip(msg.name, msg.position))
        if all(name in values for names in LEG_JOINTS.values() for name in names):
            self.joint_state = {
                name: float(values[name])
                for names in LEG_JOINTS.values() for name in names
            }

    def _read_feet(self):
        result = {}
        for leg, frame in FOOT_FRAMES.items():
            transform = self.tf_buffer.lookup_transform(self.base_frame, frame, Time())
            p = transform.transform.translation
            result[leg] = [float(p.x), float(p.y), float(p.z)]
        return result

    def _initialize(self):
        if self.joint_state is None:
            return False
        try:
            feet = self._read_feet()
            home_ik = {}
            for leg in LEGS:
                hx, hy, hz = HIP_ORIGINS[leg]
                fx, fy, fz = feet[leg]
                vec = ((hx - fx) * 1000.0, (hy - fy) * 1000.0, (hz - fz) * 1000.0)
                home_ik[leg] = self.ik.ja_from_htf_vecs([vec])[0]
            self.home_feet = {leg: list(p) for leg, p in feet.items()}
            self.targets = {leg: list(p) for leg, p in feet.items()}
            self.home_ik = home_ik
            self.home_joints = {
                leg: [self.joint_state[name] for name in LEG_JOINTS[leg]] for leg in LEGS
            }
            self.ready = True
            self.get_logger().info(
                'Feedback ready. Press g to start the crawl; no motion is sent before then.'
            )
            return True
        except (TransformException, ArithmeticError, ValueError) as exc:
            self.get_logger().warning(f'Waiting for a complete foot transform/IK solution: {exc}')
            return False

    def toggle_gait(self):
        if not self.ready:
            self.get_logger().warning('Cannot start yet; waiting for TF and /joint_states.')
            return
        if self.running:
            if self.active_leg is None:
                self.running = False
                self.stop_after_swing = False
                self.get_logger().info('Crawl paused between steps.')
            else:
                self.stop_after_swing = True
                self.get_logger().info('Will pause after the current foot lands.')
        else:
            self.stop_after_swing = False
            self.running = True
            self.next_swing_at = self.get_clock().now()
            self.get_logger().info(
                f'Crawl started: order FL, BR, FR, BL; stride {self.step_length:.3f} m.'
            )

    def adjust_stride(self, delta):
        self.step_length = max(0.01, min(0.10, self.step_length + delta))
        self.get_logger().info(f'Step length set to {self.step_length:.3f} m.')

    def request_exit(self):
        if self.active_leg is None:
            self.running = False
            self.exit_after_swing = True
        else:
            self.running = False
            self.stop_after_swing = True
            self.exit_after_swing = True
            self.get_logger().info('Exiting after the active foot lands.')

    def _begin_next_swing(self, now):
        leg = CRAWL_ORDER[self.sequence_index]
        home = self.home_feet[leg]
        start = list(self.targets[leg])
        # Alternate between two X footholds around the measured home position.
        next_side = -self.leg_side[leg]
        end = [home[0] + next_side * self.step_length / 2.0, home[1], home[2]]
        self.active_leg = leg
        self.swing_start = start
        self.swing_end = end
        self.swing_started_at = now
        self.next_swing_at = None
        self.pending_side = next_side
        self.get_logger().info(
            f'Swing {leg.upper()} to X offset {next_side * self.step_length / 2.0:+.3f} m.'
        )

    def _advance_gait(self, now):
        if self.active_leg is not None:
            t = (now - self.swing_started_at).nanoseconds / 1e9 / self.swing_duration
            t = max(0.0, min(1.0, t))
            u = t * t * (3.0 - 2.0 * t)
            leg = self.active_leg
            p = [self.swing_start[i] + (self.swing_end[i] - self.swing_start[i]) * u for i in range(3)]
            p[2] = self.swing_start[2] + self.swing_height * math.sin(math.pi * u)
            self.targets[leg] = p
            if t >= 1.0:
                self.targets[leg] = list(self.swing_end)
                self.leg_side[leg] = self.pending_side
                self.active_leg = None
                self.sequence_index = (self.sequence_index + 1) % len(CRAWL_ORDER)
                self.next_swing_at = now + Duration(seconds=self.dwell_duration)
                self.get_logger().info(f'{leg.upper()} landed; next leg follows after the dwell.')
                if self.stop_after_swing:
                    self.running = False
                    self.stop_after_swing = False
                    self.get_logger().info('Crawl paused with the current foot on the ground.')
            return
        if self.running and self.next_swing_at is not None and now >= self.next_swing_at:
            self._begin_next_swing(now)

    def _publish_targets(self):
        joint_matrix = {}
        for leg in LEGS:
            hx, hy, hz = HIP_ORIGINS[leg]
            fx, fy, fz = self.targets[leg]
            vector = ((hx - fx) * 1000.0, (hy - fy) * 1000.0, (hz - fz) * 1000.0)
            requested = self.ik.ja_from_htf_vecs([vector])[0]
            joint_matrix[leg] = [
                self.home_joints[leg][i] + requested[i] - self.home_ik[leg][i]
                for i in range(3)
            ]
        for leg in LEGS:
            if any(v < lo or v > hi for v, (lo, hi) in zip(joint_matrix[leg], JOINT_LIMITS)):
                raise ValueError(f'{leg.upper()} foot target would exceed a joint limit')
        msg = JointAngles()
        msg.fl = [float(v) for v in joint_matrix['fl']]
        msg.fr = [float(v) for v in joint_matrix['fr']]
        msg.bl = [float(v) for v in joint_matrix['bl']]
        msg.br = [float(v) for v in joint_matrix['br']]
        self.publisher.publish(msg)

    def _tick(self):
        if not self.ready and not self._initialize():
            return
        was_swinging = self.active_leg is not None
        self._advance_gait(self.get_clock().now())
        if self.active_leg is not None or was_swinging:
            try:
                self._publish_targets()
            except (ArithmeticError, ValueError) as exc:
                self.running = False
                self.active_leg = None
                self.get_logger().error(f'Gait stopped; no valid joint command: {exc}')


def read_key():
    ready, _, _ = select.select([sys.stdin], [], [], 0.02)
    if not ready:
        return None
    byte = os.read(sys.stdin.fileno(), 1)
    if byte == b'\x1b':
        if not select.select([sys.stdin], [], [], 0.02)[0]:
            return 'escape'
        os.read(sys.stdin.fileno(), 1)
        tail = os.read(sys.stdin.fileno(), 1)
        return {b'A': 'up', b'B': 'down'}.get(tail, 'escape')
    return byte.decode(errors='ignore')


def main(args=None):
    if not sys.stdin.isatty():
        raise RuntimeError('Run the crawl coordinator from an interactive terminal.')
    rclpy.init(args=args)
    node = CrawlGaitCoordinator()
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    should_exit = False
    try:
        tty.setcbreak(fd)
        print('\r\nCrawl gait: g=start/pause after landing, +/-=step length, q=quit\r')
        print(f'Step length {node.step_length:.3f} m; lift {node.swing_height:.3f} m\r')
        while rclpy.ok() and not should_exit:
            rclpy.spin_once(node, timeout_sec=0.0)
            if node.exit_after_swing and node.active_leg is None:
                break
            key = read_key()
            if key in ('g', 'G'):
                node.toggle_gait()
            elif key in ('+', '='):
                node.adjust_stride(0.01)
            elif key == '-':
                node.adjust_stride(-0.01)
            elif key == 'q':
                node.request_exit()
                if node.exit_after_swing and node.active_leg is None:
                    should_exit = True
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
