#!/usr/bin/env python3
"""Slow keyboard crawl/trot using FK and IK parsed from the installed URDF."""

import math
import sys
import termios
import time
import tty

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.msg import ContactsState
from gazebo_msgs.msg import ModelStates, ModelState
from gazebo_msgs.srv import SetModelState
from kinematics import UrdfLegKinematics, FOOT_LINKS, LEGS
from keyboard_input import read_key
from safety_monitor import SafetyMonitor, SafetyState
from balance import convex_hull, stability_margin
from gait_generator import CRAWL_OFFSETS, TROT_OFFSETS, swing_legs
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


JOINT_NAMES = [
    f'motor_{side}_{joint}'
    for side in ('front_left', 'front_right', 'back_left', 'back_right')
    for joint in ('hip', 'upper_leg', 'lower_leg')
]
BENT_START_POSE = [
    0.0, -0.68162889, 1.26606748,
    0.0, -0.68162889, 1.26606748,
    0.0, -0.68162889, 1.26606748,
    0.0, -0.68162889, 1.26606748,
]
CRAWL_OFFSETS = {'fl': 0.75, 'br': 0.50, 'fr': 0.25, 'bl': 0.0}
TROT_OFFSETS = {'fl': 0.0, 'br': 0.0, 'fr': 0.5, 'bl': 0.5}
# Small-step swing clearance arcs from the measured 5 cm single-foot tests.
# The offsets are zero at takeoff/touchdown and peak with the foot lift.
SWING_CLEARANCE_XY = {
    'fl': (0.013, 0.028), 'fr': (0.013, -0.028),
    'bl': (-0.013, 0.052), 'br': (-0.013, -0.052),
}


class KeyboardWalk(Node):
    def __init__(self):
        super().__init__('keyboard_walk')
        # The node publishes into a Gazebo simulation when run separately with
        # ros2 run, so its timers and trajectory stamps must use /clock.
        result = self.set_parameters([
            Parameter('use_sim_time', Parameter.Type.BOOL, True)
        ])[0]
        if not result.successful:
            raise RuntimeError(f'Cannot enable simulated time: {result.reason}')
        self.declare_parameter('trajectory_topic', '/quadruped_controller/joint_trajectory')
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('cycle_duration_sec', 24.0)
        self.declare_parameter('step_length_m', 0.005)
        self.declare_parameter('swing_height_m', 0.03)
        self.declare_parameter('swing_fraction', 0.125)
        # Transfer toward a 25 mm margin, while requiring at least 22 mm from
        # the measured feet and COM before the next foot leaves the floor.
        self.declare_parameter('support_margin_m', 0.025)
        self.declare_parameter('minimum_support_margin_m', 0.022)
        self.declare_parameter('start_pose_tolerance_rad', 0.08)
        self.declare_parameter('publish_rate_hz', 30.0)
        self.cycle_duration = float(self.get_parameter('cycle_duration_sec').value)
        self.step_length = float(self.get_parameter('step_length_m').value)
        self.swing_height = float(self.get_parameter('swing_height_m').value)
        self.swing_fraction = float(self.get_parameter('swing_fraction').value)
        self.support_margin = float(self.get_parameter('support_margin_m').value)
        self.minimum_support_margin = float(
            self.get_parameter('minimum_support_margin_m').value
        )
        self.start_pose_tolerance = float(
            self.get_parameter('start_pose_tolerance_rad').value
        )
        if (self.cycle_duration <= 0 or self.step_length <= 0
                or self.swing_height < 0 or self.support_margin <= 0
                or self.minimum_support_margin <= 0):
            raise ValueError('Cycle and step length must be positive; swing height cannot be negative')
        if not 0.0 < self.swing_fraction < 0.25:
            raise ValueError('swing_fraction must be in (0, 0.25) for a crawl gait')

        self.kinematics = UrdfLegKinematics()
        self.publisher = self.create_publisher(
            JointTrajectory, str(self.get_parameter('trajectory_topic').value), 10
        )
        self.status_publisher = self.create_publisher(String, '/keyboard_walk/status', 10)
        self.create_subscription(String, '/robot_logger/safety_stop', self._safety_stop, 10)
        self.contacts = {leg: None for leg in LEGS}
        self.safety = SafetyMonitor()
        self.last_safety_state = SafetyState.OK
        self.stable_since = None
        self.last_pitch = None
        self.last_imu_pitch = None
        self.last_base_z = None
        self.stable_reference = None
        self.stable_ready = False
        self.base_xyz = None
        self.base_rotation = None
        self.base_yaw = None
        self.start_feet_world = None
        self.stride_world = None
        self.start_world_xyz = None
        self.start_world_rotation = None
        self.contact_subs = [self.create_subscription(
            ContactsState, f'/foot_contacts/{FOOT_LINKS[leg]}/bumper_states',
            lambda msg, leg=leg: self._contact_cb(leg, msg), 10) for leg in LEGS]
        self.joint_state = None
        self.create_subscription(
            JointState, str(self.get_parameter('joint_states_topic').value),
            self._on_joint_state, 10,
        )
        self.create_subscription(ModelStates, '/gazebo/model_states',
                                 self._on_model_states, 10)
        self.create_subscription(Imu, '/imu', self._on_imu, 10)
        self.reset_client = self.create_client(SetModelState, '/gazebo/set_model_state')
        self.home = None
        self.home_feet = None
        self.last_output = None
        self.start_pose_commanded = False
        self.vx = self.vy = self.yaw = 0.0
        self.speed_scale = 1.0
        self.gait = 'crawl'
        self.crouched = False
        self.motion_started = None
        self.timer = self.create_timer(
            1.0 / float(self.get_parameter('publish_rate_hz').value), self._tick
        )
        self.get_logger().info('URDF parsed; waiting for complete /joint_states feedback.')

    def _publish_status(self, key):
        status = (f'key={key};vx={self.vx * self.speed_scale:.4f};'
                  f'vy={self.vy * self.speed_scale:.4f};'
                  f'yaw={self.yaw * self.speed_scale:.4f};gait={self.gait};'
                  f'step_length_m={self.step_length:.4f};'
                  f'swing_height_m={self.swing_height:.4f};'
                  f'cycle_duration_sec={self.cycle_duration:.3f}')
        self.status_publisher.publish(String(data=status))

    def _safety_stop(self, msg):
        self.vx = self.vy = self.yaw = 0.0
        self.motion_started = None
        self.get_logger().error(f'Safety stop from robot_logger: {msg.data}; commanding stand pose.')
        if self.home is not None:
            self._publish(self._home_output(), 1.0)
        self._publish_status(f'safety_stop:{msg.data}')

    def _contact_cb(self, leg, msg):
        self.contacts[leg] = bool(msg.states)

    def _on_model_states(self, msg):
        try:
            index = msg.name.index('open_quadruped')
        except ValueError:
            return
        pose = msg.pose[index]
        self.base_xyz = np.array([pose.position.x, pose.position.y, pose.position.z])
        q = pose.orientation
        x, y, z, w = q.x, q.y, q.z, q.w
        self.base_rotation = np.array([
            [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
            [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
            [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)],
        ])
        self.last_base_z = float(pose.position.z)
        self.last_pitch = math.asin(float(np.clip(-self.base_rotation[2, 0], -1, 1)))
        self.base_yaw = math.atan2(self.base_rotation[1, 0],
                                   self.base_rotation[0, 0])
        self._initialize_home_if_ready()


    def _on_imu(self, msg):
        q = msg.orientation
        sinp = 2.0 * (q.w*q.y - q.z*q.x)
        pitch = math.asin(float(np.clip(sinp, -1.0, 1.0)))
        sinr = 2.0 * (q.w*q.x + q.y*q.z)
        cosr = 1.0 - 2.0 * (q.x*q.x + q.y*q.y)
        roll = math.atan2(sinr, cosr)
        contacts = ({leg: value is True for leg, value in self.contacts.items()}
                    if self.home is not None and all(value is not None for value in self.contacts.values())
                    else None)
        if contacts is not None and self.motion_started is not None:
            gait_elapsed = (self.get_clock().now() - self.motion_started).nanoseconds / 1e9
            for expected_swing in swing_legs(gait_elapsed, self.cycle_duration, self.gait, self.swing_fraction):
                contacts[expected_swing] = True
        self.last_imu_pitch = pitch
        state = self.safety.evaluate(roll=roll, pitch=pitch,
            angular_speed=float(np.linalg.norm([msg.angular_velocity.x, msg.angular_velocity.y])),
            contacts=contacts)
        changed = state != self.last_safety_state
        self.last_safety_state = state
        if state is SafetyState.FALL:
            self.vx = self.vy = self.yaw = 0.0
            self.motion_started = None
            if changed:
                self.get_logger().fatal(f'FALL detected by /imu (roll={roll:.3f}, pitch={pitch:.3f}); movement locked.')
        elif state is SafetyState.WARNING:
            self.vx = self.vy = self.yaw = 0.0
            self.motion_started = None
            if changed:
                self.get_logger().error('WARNING from safety monitor; stopping gait and returning to stand pose.')
            if self.home is not None:
                self._publish(self._home_output(), 1.0)

    def _is_stable(self):
        ready = (self.home is not None and self.last_imu_pitch is not None
                 and self.last_base_z is not None and abs(self.last_imu_pitch) < .12
                 and all(v is True for v in self.contacts.values()))
        now = time.monotonic()
        if ready:
            if self.stable_since is None:
                self.stable_since = now
                self.stable_reference = (self.last_base_z, self.last_imu_pitch)
            z_ref, pitch_ref = self.stable_reference
            stable_window = (abs(self.last_base_z - z_ref) < .005
                             and abs(self.last_imu_pitch - pitch_ref) < .02)
            if stable_window:
                self.stable_ready = now - self.stable_since >= 2.0
            else:
                self.stable_since = now
                self.stable_reference = (self.last_base_z, self.last_imu_pitch)
                self.stable_ready = False
        else:
            self.stable_since = None
            self.stable_reference = None
            self.stable_ready = False
        return self.stable_ready

    def _on_joint_state(self, msg):
        positions = dict(zip(msg.name, msg.position))
        if all(name in positions for name in JOINT_NAMES):
            self.joint_state = {name: float(positions[name]) for name in JOINT_NAMES}
            self._initialize_home_if_ready()

    def _initialize_home_if_ready(self):
        if self.home is not None or self.joint_state is None or self.base_xyz is None:
            return
        measured = [self.joint_state[name] for name in JOINT_NAMES]
        pose_error = max(abs(a - b) for a, b in zip(measured, BENT_START_POSE))
        if pose_error > self.start_pose_tolerance and not self.start_pose_commanded:
            self.get_logger().info(
                f'Moving to validated bent start pose (max initial joint error '
                f'{pose_error:.3f} rad) before enabling keyboard walking.'
            )
            self.start_pose_commanded = True
            self._publish(BENT_START_POSE, 10.0)
            return
        if pose_error > self.start_pose_tolerance:
            return
        self.home = {leg: [self.joint_state[name] for name in self.kinematics.chains[leg][1]]
                     for leg in LEGS}
        self.home_feet = {
            leg: self.kinematics.fk(leg, self.home[leg]) for leg in LEGS
        }
        self.home_feet_world = {
            leg: self.base_xyz + self.base_rotation @ self.home_feet[leg]
            for leg in LEGS
        }
        self.last_output = {name: self.joint_state[name] for name in JOINT_NAMES}
        self.get_logger().info(
            'Validated bent start pose reached; world-frame foot anchoring is ready.'
        )

    def _home_output(self):
        return [value for leg in LEGS for value in self.home[leg]]

    @staticmethod
    def _smooth(value):
        value = max(0.0, min(1.0, value))
        return value * value * (3.0 - 2.0 * value)

    def _leg_offset(self, leg, elapsed):
        phases = CRAWL_OFFSETS if self.gait == 'crawl' else TROT_OFFSETS
        phase = (elapsed / self.cycle_duration + phases[leg]) % 1.0
        stance = 1.0 - self.swing_fraction
        swinging = phase >= stance
        if swinging:
            progress = (phase - stance) / self.swing_fraction
            blend = self._smooth(progress)
            wave = -0.5 + blend
            lift = self.swing_height * math.sin(math.pi * progress) ** 2
        else:
            progress = phase / stance
            wave = 0.5 - progress
            lift = 0.0

        side = 1.0 if leg in ('fl', 'bl') else -1.0
        ramp = self._smooth(elapsed / 0.5)
        forward_drive = self.speed_scale * (self.vx - self.yaw * side * 0.5)
        lateral_drive = self.speed_scale * self.vy
        dx = math.copysign(1.0, forward_drive) * wave * self.step_length * abs(forward_drive) if forward_drive else 0.0
        dy = math.copysign(1.0, lateral_drive) * wave * self.step_length * abs(lateral_drive) if lateral_drive else 0.0
        return ramp * dx, ramp * dy, ramp * lift

    @staticmethod
    def _convex_hull(points):
        return convex_hull(points)

    def _support_translation(self, support_points, margin_m=0.005, com_xy=None):
        hull = self._convex_hull(support_points)
        if len(hull) < 3:
            return np.zeros(2)
        com = (self.kinematics.com_xy_at(self.joint_state)
               if com_xy is None else np.asarray(com_xy, dtype=float))
        edges = []
        distances = []
        for index, a_tuple in enumerate(hull):
            a = np.asarray(a_tuple)
            b = np.asarray(hull[(index + 1) % len(hull)])
            edge = b - a
            length = float(np.linalg.norm(edge))
            normal = np.array([-edge[1], edge[0]]) / length  # hull is counterclockwise
            distance = float(np.dot(com - a, normal))
            edges.append((a, edge, normal, length))
            distances.append(distance)
        if min(distances) >= 0.0:
            index = int(np.argmin(distances))
            _, _, normal, _ = edges[index]
            # Foot targets move opposite to the desired body/COM translation:
            # with planted feet, moving each target inward shifts the body outward.
            return -normal * max(0.0, margin_m - distances[index])

        nearest_point = None
        nearest_distance = float('inf')
        nearest_normal = None
        for a, edge, normal, length in edges:
            fraction = float(np.clip(np.dot(com - a, edge) / (length * length), 0.0, 1.0))
            point = a + fraction * edge
            distance = float(np.linalg.norm(com - point))
            if distance < nearest_distance:
                nearest_point = point
                nearest_distance = distance
                nearest_normal = normal
        return com - nearest_point - margin_m * nearest_normal

    def _support_margin(self, support_points):
        return stability_margin(support_points, self.kinematics.com_xy_at(self.joint_state))

    def _measured_foot_xy(self, leg):
        names = self.kinematics.chains[leg][1]
        q = [self.joint_state[name] for name in names]
        return self.kinematics.fk(leg, q)[:2]

    def _balance_shift(self, cycle_phase, elapsed):
        stance = 1.0 - self.swing_fraction
        # Transfer weight well before liftoff and release it after landing.
        # The first command itself is ramped in so it cannot step the body.
        pre_swing = 0.12
        post_land = 0.04
        offsets = CRAWL_OFFSETS if self.gait == 'crawl' else TROT_OFFSETS
        candidates = []
        for leg in LEGS:
            phase = (cycle_phase + offsets[leg]) % 1.0
            if stance - pre_swing <= phase < stance:
                blend = self._smooth((phase - (stance - pre_swing)) / pre_swing)
            elif stance <= phase < 1.0:
                blend = 1.0
            elif phase < post_land:
                blend = 1.0 - self._smooth(phase / post_land)
            else:
                continue
            support_points = []
            for other in LEGS:
                if other == leg:
                    continue
                support_points.append(self._measured_foot_xy(other))
            candidates.append((blend, self._support_translation(
                support_points, margin_m=self.support_margin,
                com_xy=self.kinematics.com_xy_at(self.joint_state)
            )))
        if not candidates:
            return np.zeros(3)
        blend, shift = max(candidates, key=lambda item: item[0])
        blend *= self._smooth(elapsed / 3.0)
        # Lower the body slightly while translating the legs toward the
        # three-foot support polygon, leaving reach margin before liftoff.
        return np.array([blend * shift[0], blend * shift[1], blend * 0.005])

    def _publish(self, positions, duration=0.12):
        msg = JointTrajectory()
        # Interpret each streamed IK target as starting on receipt; the node's
        # compute time must not make a short trajectory start in the past.
        msg.joint_names = JOINT_NAMES
        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in positions]
        sec = int(duration)
        point.time_from_start.sec = sec
        point.time_from_start.nanosec = int((duration - sec) * 1e9)
        msg.points = [point]
        self.publisher.publish(msg)
        if len(positions) == len(JOINT_NAMES):
            self.last_output = dict(zip(JOINT_NAMES, map(float, positions)))

    def _tick(self):
        self._is_stable()
        if (self.home is None or self.home_feet is None
                or self.base_xyz is None or self.base_rotation is None):
            return
        if self.vx == 0.0 and self.vy == 0.0 and self.yaw == 0.0:
            # The trajectory bridge already holds the start pose. Avoid
            # streaming identical trajectories while idle; JTC can treat each
            # new message as a replacement trajectory.
            self.motion_started = None
            return

        if self.motion_started is None:
            self.motion_started = self.get_clock().now()
            self.start_feet_world = {
                leg: self.base_xyz + self.base_rotation @ self.kinematics.fk(
                    leg, [self.joint_state[name]
                          for name in self.kinematics.chains[leg][1]])
                for leg in LEGS
            }
            self.start_world_xyz = self.base_xyz.copy()
            self.start_world_rotation = self.base_rotation.copy()
            self.stride_world = {}
            for leg in LEGS:
                side = 1.0 if leg in ('fl', 'bl') else -1.0
                forward_drive = self.speed_scale * (self.vx - self.yaw * side * 0.5)
                lateral_drive = self.speed_scale * self.vy
                local_stride = np.array([
                    math.copysign(self.step_length * abs(forward_drive), forward_drive)
                    if forward_drive else 0.0,
                    math.copysign(self.step_length * abs(lateral_drive), lateral_drive)
                    if lateral_drive else 0.0,
                    0.0,
                ])
                self.stride_world[leg] = self.start_world_rotation @ local_stride
        elapsed = (self.get_clock().now() - self.motion_started).nanoseconds / 1e9
        cycle_phase = (elapsed / self.cycle_duration) % 1.0
        balance_shift = self._balance_shift(cycle_phase, elapsed)
        offsets = CRAWL_OFFSETS if self.gait == 'crawl' else TROT_OFFSETS
        stance = 1.0 - self.swing_fraction
        swinging = list(swing_legs(elapsed, self.cycle_duration, self.gait, self.swing_fraction))
        if swinging:
            leg = swinging[0]
            supports = [other for other in LEGS if other != leg]
            if not all(self.contacts[other] is True for other in supports):
                support_states = {support: self.contacts[support] for support in supports}
                self.vx = self.vy = self.yaw = 0.0
                self.motion_started = None
                self.get_logger().error(
                    f'Crawl stopped before {leg} liftoff: support contact missing '
                    f'({support_states}).'
                )
                self._publish(self._home_output(), 0.5)
                return
            # Gate liftoff from measured support feet and the measured-joint
            # URDF COM. Planned targets alone were optimistic in the first
            # crawl trial when the joints lagged their IK commands.
            support_points = [self._measured_foot_xy(other) for other in supports]
            measured_margin = self._support_margin(support_points)
            if measured_margin < self.minimum_support_margin:
                self.vx = self.vy = self.yaw = 0.0
                self.motion_started = None
                self.get_logger().error(
                    f'Crawl stopped before {leg} liftoff: predicted support margin '
                    f'{measured_margin:.4f} m is below safety minimum '
                    f'{self.minimum_support_margin:.4f} m.'
                )
                self._publish(self._home_output(), 0.5)
                return
        output = None
        last_ik_error = None
        for transfer_scale in (1.0, 0.75, 0.5, 0.25, 0.0):
            candidate = []
            try:
                for leg in LEGS:
                    leg_offset = offsets[leg]
                    leg_cycle = math.floor(elapsed / self.cycle_duration + leg_offset)
                    phase = (cycle_phase + leg_offset) % 1.0
                    stride = self.stride_world[leg]
                    target_world = (self.start_feet_world[leg] + leg_cycle * stride)
                    lift = 0.0
                    if phase >= 1.0 - self.swing_fraction:
                        progress = (phase - (1.0 - self.swing_fraction)) / self.swing_fraction
                        blend = self._smooth(progress)
                        target_world = target_world + blend * stride
                        swing_wave = math.sin(math.pi * progress) ** 2
                        lift = self.swing_height * swing_wave
                        clearance_local = np.array([
                            SWING_CLEARANCE_XY[leg][0],
                            SWING_CLEARANCE_XY[leg][1], 0.0,
                        ]) * swing_wave * min(1.0, self.swing_height / 0.05)
                        target_world = target_world + self.start_world_rotation @ clearance_local
                    target_world = target_world + self.base_rotation @ (
                        transfer_scale * balance_shift
                    )
                    target_world[2] += lift
                    target = self.base_rotation.T @ (target_world - self.base_xyz)
                    names = self.kinematics.chains[leg][1]
                    seed = [self.last_output[name] for name in names]
                    candidate.extend(self.kinematics.ik(leg, target, seed))
                output = candidate
                if transfer_scale < 1.0:
                    self.get_logger().warning(
                        f'Reduced support-transfer command to {transfer_scale:.2f} '
                        'to keep all feet within IK limits.'
                    )
                break
            except (ValueError, np.linalg.LinAlgError) as exc:
                last_ik_error = exc
        if output is None:
            self.vx = self.vy = self.yaw = 0.0
            self.motion_started = None
            self.get_logger().error(
                f'Gait stopped; no reachable IK target after transfer reduction: '
                f'{last_ik_error}'
            )
            self._publish(self._home_output(), 0.5)
            return
        self._publish(output)

    def handle_key(self, key):
        if key in ('w', 's', 'a', 'd', 'q', 'e') and not self._is_stable():
            self.get_logger().warning(
                f'Keyboard movement refused until 4 contacts and stable base pose are maintained for 2 seconds.'
            )
            return
        if key == 'w': self.vx = 1.0; self.crouched = False
        elif key == 's': self.vx = -1.0; self.crouched = False
        elif key == 'a': self.yaw = 1.0; self.crouched = False
        elif key == 'd': self.yaw = -1.0; self.crouched = False
        elif key == 'q': self.vy = 1.0; self.crouched = False
        elif key == 'e': self.vy = -1.0; self.crouched = False
        elif key == ' ':
            self.vx = self.vy = self.yaw = 0.0
            self.crouched = False
            self.get_logger().info('Stopped; returning to the measured stand pose.')
            if self.home is not None:
                self._publish(self._home_output(), 0.5)
        elif key in ('r', 'R'):
            self.vx = self.vy = self.yaw = 0.0
            self.motion_started = None
            if not self.reset_client.wait_for_service(timeout_sec=0.2):
                self.get_logger().error('Reset unavailable: /gazebo/set_model_state is not active.')
            else:
                state = ModelState()
                state.model_name = 'open_quadruped'
                state.pose.position.z = 0.22
                state.pose.orientation.w = 1.0
                request = SetModelState.Request(); request.model_state = state
                self.reset_client.call_async(request)
                self._publish(BENT_START_POSE, 1.0)
                self.stable_since = None; self.stable_reference = None; self.stable_ready = False
                self.get_logger().warning('Gazebo base reset requested; waiting for stable stand before movement.')
        elif key in ('x', 'X'):
            self.vx = self.vy = self.yaw = 0.0
            self.get_logger().warning(
                'Crouch is disabled: the URDF start pose has no validated stable '
                'crouch target, and the previous all-leg target lost ground support.'
            )
        elif key in ('+', '='):
            self.speed_scale = min(2.0, self.speed_scale + 0.1)
            self.step_length = min(0.05, self.step_length + 0.002)
            self.get_logger().info(f'Speed scale {self.speed_scale:.1f}; step {self.step_length:.3f} m.')
        elif key == '-':
            self.speed_scale = max(0.1, self.speed_scale - 0.1)
            self.step_length = max(0.004, self.step_length - 0.002)
            self.get_logger().info(f'Speed scale {self.speed_scale:.1f}; step {self.step_length:.3f} m.')
        elif key in ('g', 'G'):
            self.gait = 'trot' if self.gait == 'crawl' else 'crawl'
            self.get_logger().info(f'Gait set to {self.gait}. Crawl is the safer default.')
        self._publish_status(key if key != ' ' else 'space')


def main(args=None):
    if not sys.stdin.isatty():
        raise RuntimeError('Run keyboard_walk.py in an interactive terminal.')
    rclpy.init(args=args)
    node = KeyboardWalk()
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    print('\nKeyboard walk: w/s forward/back, a/d turn, q/e sidestep, SPACE stand/stop, r reset after fall, x crouch (disabled), +/- stride, g crawl/trot, Ctrl+C quit.')
    try:
        tty.setcbreak(fd)
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            key = read_key()
            if key is not None:
                node.handle_key(key)
    except KeyboardInterrupt:
        pass
    finally:
        if node.home is not None:
            node.vx = node.vy = node.yaw = 0.0
            try:
                node._publish(node._home_output(), 0.5)
                time.sleep(0.1)
            except Exception:  # ROS context may already be shutting down on Ctrl+C.
                pass
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
