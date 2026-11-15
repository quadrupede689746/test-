#!/usr/bin/env python3
"""Coordinate a continuous, slow crawl foot cycle from terminal controls."""

import math
import os
import select
import sys
import termios
import tty

import rclpy
from rclpy.duration import Duration
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
HIP_ORIGINS = {
    'fl': (0.0915, 0.0394, 0.022), 'fr': (0.0915, -0.0394, 0.022),
    'bl': (-0.1365, 0.0394, 0.022), 'br': (-0.1365, -0.0394, 0.022),
}
JOINT_LIMITS = ((-1.04, 1.04), (-math.pi / 2, 2.59), (-2.9, math.pi / 2))

# phase = (cycle_phase + offset) mod 1. With 1/8 swing and 1/4 phase
# spacing, only one foot swings at a time, in FL, BR, FR, BL order.
PHASE_OFFSETS = {'fl': 0.75, 'br': 0.50, 'fr': 0.25, 'bl': 0.0}


def smoothstep(value):
    value = max(0.0, min(1.0, value))
    return value * value * (3.0 - 2.0 * value)


class CrawlGaitCoordinator(Node):
    def __init__(self):
        super().__init__('crawl_gait_coordinator')
        self.declare_parameter('step_length_m', 0.04)
        self.declare_parameter('swing_height_m', 0.025)
        self.declare_parameter('cycle_duration_sec', 8.0)
        self.declare_parameter('swing_fraction', 0.125)
        self.declare_parameter('publish_rate_hz', 30.0)
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('joint_angles_topic', '/joint_angles')
        self.declare_parameter('joint_states_topic', '/joint_states')

        self.step_length = float(self.get_parameter('step_length_m').value)
        self.swing_height = float(self.get_parameter('swing_height_m').value)
        self.cycle_duration = float(self.get_parameter('cycle_duration_sec').value)
        self.swing_fraction = float(self.get_parameter('swing_fraction').value)
        rate = float(self.get_parameter('publish_rate_hz').value)
        for name, value in (
            ('step_length_m', self.step_length), ('swing_height_m', self.swing_height),
            ('cycle_duration_sec', self.cycle_duration), ('publish_rate_hz', rate),
        ):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f'{name} must be a positive finite number')
        if not 0.0 < self.swing_fraction < 0.25:
            raise ValueError('swing_fraction must be between 0 and 0.25 for non-overlapping swings')
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
        self.pause_requested = False
        self.exit_requested = False
        self.exit_after_pause = False
        self.elapsed_before_start = 0.0
        self.run_started_at = None
        self.last_logged_swing = None
        self.timer = self.create_timer(1.0 / rate, self._tick)

    def _on_joint_state(self, msg):
        values = dict(zip(msg.name, msg.position))
        if all(name in values for names in LEG_JOINTS.values() for name in names):
            self.joint_state = {
                name: float(values[name])
                for names in LEG_JOINTS.values() for name in names
            }

    def _read_feet(self):
        feet = {}
        for leg, frame in FOOT_FRAMES.items():
            transform = self.tf_buffer.lookup_transform(self.base_frame, frame, Time())
            p = transform.transform.translation
            feet[leg] = [float(p.x), float(p.y), float(p.z)]
        return feet

    def _initialize(self):
        if self.joint_state is None:
            return False
        try:
            feet = self._read_feet()
            home_ik = {}
            for leg in LEGS:
                hx, hy, hz = HIP_ORIGINS[leg]
                fx, fy, fz = feet[leg]
                vector = ((hx - fx) * 1000.0, (hy - fy) * 1000.0, (hz - fz) * 1000.0)
                home_ik[leg] = self.ik.ja_from_htf_vecs([vector])[0]
            self.home_feet = {leg: list(point) for leg, point in feet.items()}
            self.targets = {leg: list(point) for leg, point in feet.items()}
            self.home_ik = home_ik
            self.home_joints = {
                leg: [self.joint_state[name] for name in LEG_JOINTS[leg]] for leg in LEGS
            }
            self.ready = True
            self.get_logger().info('Feedback ready. Press g to start the continuous crawl; no motion before then.')
            return True
        except (TransformException, ArithmeticError, ValueError) as exc:
            self.get_logger().warning(f'Waiting for a complete foot transform/IK solution: {exc}')
            return False

    def toggle_gait(self):
        if not self.ready:
            self.get_logger().warning('Cannot start yet; waiting for TF and /joint_states.')
            return
        if self.running:
            self.pause_requested = True
            self.get_logger().info('Pause requested; holding after the current foot swing lands.')
        else:
            self.running = True
            self.pause_requested = False
            self.run_started_at = self.get_clock().now()
            self.get_logger().info(
                f'Continuous crawl started: step length {self.step_length:.3f} m, '
                f'cycle {self.cycle_duration:.1f} s.'
            )

    def adjust_stride(self, delta):
        self.step_length = max(0.01, min(0.10, self.step_length + delta))
        self.get_logger().info(f'Step length set to {self.step_length:.3f} m peak-to-peak.')

    def request_exit(self):
        if self.running:
            self.exit_requested = True
            self.pause_requested = True
            self.get_logger().info('Exiting after the current swing lands.')
        else:
            self.exit_after_pause = True

    def _elapsed(self, now):
        elapsed = self.elapsed_before_start
        if self.running and self.run_started_at is not None:
            elapsed += (now - self.run_started_at).nanoseconds / 1e9
        return elapsed

    def _leg_phase(self, cycle_phase, leg):
        return (cycle_phase + PHASE_OFFSETS[leg]) % 1.0

    def _swinging_legs(self, cycle_phase):
        swing_start = 1.0 - self.swing_fraction
        return [leg for leg in LEGS if self._leg_phase(cycle_phase, leg) >= swing_start]

    def _stance_x(self, phase):
        stance_fraction = 1.0 - self.swing_fraction
        return self.step_length / 2.0 - self.step_length * smoothstep(phase / stance_fraction)

    def _update_foot_targets(self, cycle_phase):
        stance_fraction = 1.0 - self.swing_fraction
        swing_start = stance_fraction
        for leg in LEGS:
            leg_phase = self._leg_phase(cycle_phase, leg)
            reference_phase = PHASE_OFFSETS[leg]
            reference_x = self._stance_x(reference_phase)
            if leg_phase < stance_fraction:
                x_offset = self._stance_x(leg_phase) - reference_x
                z_offset = 0.0
            else:
                swing_phase = (leg_phase - swing_start) / self.swing_fraction
                x_offset = (
                    -self.step_length / 2.0
                    + self.step_length * smoothstep(swing_phase)
                    - reference_x
                )
                z_offset = self.swing_height * math.sin(math.pi * swing_phase) ** 2
            self.targets[leg] = [
                self.home_feet[leg][0] + x_offset,
                self.home_feet[leg][1],
                self.home_feet[leg][2] + z_offset,
            ]

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
        now = self.get_clock().now()
        elapsed = self._elapsed(now)
        cycle_phase = (elapsed / self.cycle_duration) % 1.0
        self._update_foot_targets(cycle_phase)
        was_running = self.running

        swinging = self._swinging_legs(cycle_phase)
        if swinging != self.last_logged_swing:
            if swinging:
                self.get_logger().info(f'Swing phase: {swinging[0].upper()}')
            self.last_logged_swing = swinging

        if self.running and self.pause_requested and not swinging:
            self.elapsed_before_start = elapsed
            self.running = False
            self.run_started_at = None
            self.pause_requested = False
            self.get_logger().info('Crawl paused between foot swings.')
            if self.exit_requested:
                self.exit_after_pause = True

        if self.running or was_running:
            try:
                self._publish_targets()
            except (ArithmeticError, ValueError) as exc:
                self.running = False
                self.pause_requested = False
                self.get_logger().error(f'Gait stopped; no valid joint command: {exc}')


def read_key():
    ready, _, _ = select.select([sys.stdin], [], [], 0.02)
    if not ready:
        return None
    byte = os.read(sys.stdin.fileno(), 1)
    return byte.decode(errors='ignore')


def main(args=None):
    if not sys.stdin.isatty():
        raise RuntimeError('Run the crawl coordinator from an interactive terminal.')
    rclpy.init(args=args)
    node = CrawlGaitCoordinator()
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        print('\r\nContinuous crawl: g=start/pause safely, +/-=step length, q=quit after landing\r')
        print(f'Step {node.step_length:.3f} m peak-to-peak; lift {node.swing_height:.3f} m; cycle {node.cycle_duration:.1f} s\r')
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            if node.exit_after_pause:
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
                if node.exit_after_pause:
                    break
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
