#!/usr/bin/env python3
"""Slow keyboard crawl/trot using FK and IK parsed from the installed URDF."""

import math
import os
import select
import subprocess
import sys
import termios
import time
import tty
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.msg import ContactsState
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


LEGS = ('fl', 'fr', 'bl', 'br')
FOOT_LINKS = {'fl': 'front_left_foot', 'fr': 'front_right_foot',
              'bl': 'back_left_foot', 'br': 'back_right_foot'}
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


def _origin(element):
    if element is None:
        return np.eye(4)
    xyz = np.fromstring(element.get('xyz', '0 0 0'), sep=' ')
    rpy = np.fromstring(element.get('rpy', '0 0 0'), sep=' ')
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    transform = np.eye(4)
    transform[:3, :3] = rz @ ry @ rx
    transform[:3, 3] = xyz
    return transform


def _axis_rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    norm = np.linalg.norm(axis)
    if norm < 1e-12:
        raise ValueError('URDF revolute joint has a zero axis')
    x, y, z = axis / norm
    c, s, t = math.cos(angle), math.sin(angle), 1.0 - math.cos(angle)
    transform = np.eye(4)
    transform[:3, :3] = [
        [t*x*x+c, t*x*y-s*z, t*x*z+s*y],
        [t*x*y+s*z, t*y*y+c, t*y*z-s*x],
        [t*x*z-s*y, t*y*z+s*x, t*z*z+c],
    ]
    return transform


class UrdfLegKinematics:
    """FK/IK chain for each foot, extracted from the package's Xacro URDF."""
    def __init__(self):
        package = get_package_share_directory('open_quadruped')
        xacro_path = os.path.join(package, 'urdf', 'spot.urdf.xacro')
        urdf = subprocess.run(
            ['xacro', xacro_path], check=True, capture_output=True, text=True
        ).stdout
        root = ET.fromstring(urdf)
        parent_joint = {}
        for joint in root.findall('joint'):
            parent = joint.find('parent').get('link')
            child = joint.find('child').get('link')
            parent_joint[child] = joint

        self.urdf_joints = root.findall('joint')
        self.link_inertials = []
        self.total_mass = 0.0
        for link in root.findall('link'):
            inertial = link.find('inertial')
            if inertial is None:
                continue
            mass = float(inertial.find('mass').get('value'))
            self.link_inertials.append((link.get('name'), mass,
                                        _origin(inertial.find('origin'))))
            self.total_mass += mass
        if self.total_mass <= 0.0:
            raise ValueError('URDF has no positive link mass for support calculation')
        self.com_xyz = self.com_xyz_at({})
        self.com_xy = self.com_xyz[:2]

        self.chains = {}
        self.limits = {}
        for leg, foot in FOOT_LINKS.items():
            chain = []
            link = foot
            while link != 'base_link':
                joint = parent_joint.get(link)
                if joint is None:
                    raise ValueError(f'No base_link-to-{foot} chain in the URDF')
                chain.append(joint)
                link = joint.find('parent').get('link')
            chain.reverse()
            actuated = []
            for joint in chain:
                if joint.get('type') == 'revolute':
                    name = joint.get('name')
                    limit = joint.find('limit')
                    self.limits[name] = (
                        float(limit.get('lower')), float(limit.get('upper'))
                    )
                    actuated.append(name)
            if len(actuated) != 3:
                raise ValueError(f'{foot} must have exactly three revolute joints')
            self.chains[leg] = (chain, actuated)

    def fk(self, leg, positions):
        chain, names = self.chains[leg]
        values = dict(zip(names, positions))
        transform = np.eye(4)
        for joint in chain:
            transform = transform @ _origin(joint.find('origin'))
            if joint.get('type') == 'revolute':
                axis_element = joint.find('axis')
                axis = np.fromstring(axis_element.get('xyz', '1 0 0'), sep=' ')
                transform = transform @ _axis_rotation(axis, values[joint.get('name')])
        return transform[:3, 3].copy()

    def com_xyz_at(self, positions):
        """Compute the full URDF mass center at the measured joint positions."""
        transforms = {'base_link': np.eye(4)}
        while True:
            progress = False
            for joint in self.urdf_joints:
                parent = joint.find('parent').get('link')
                child = joint.find('child').get('link')
                if parent not in transforms or child in transforms:
                    continue
                transform = transforms[parent] @ _origin(joint.find('origin'))
                if joint.get('type') == 'revolute':
                    axis = np.fromstring(joint.find('axis').get('xyz', '1 0 0'), sep=' ')
                    transform = transform @ _axis_rotation(
                        axis, float(positions.get(joint.get('name'), 0.0))
                    )
                transforms[child] = transform
                progress = True
            if not progress:
                break
        weighted_com = np.zeros(3)
        included_mass = 0.0
        for name, mass, inertial_origin in self.link_inertials:
            if name not in transforms:
                continue
            weighted_com += mass * (transforms[name] @ inertial_origin)[:3, 3]
            included_mass += mass
        if included_mass <= 0.0:
            raise ValueError('Could not resolve URDF link transforms for COM')
        return weighted_com / included_mass

    def com_xy_at(self, positions):
        return self.com_xyz_at(positions)[:2]

    def ik(self, leg, target, seed):
        """Damped least-squares IK; all geometry/limits come from the URDF."""
        chain, names = self.chains[leg]
        target = np.asarray(target, dtype=float)
        q = np.asarray(seed, dtype=float).copy()
        bounds = np.asarray([self.limits[name] for name in names], dtype=float)
        for _ in range(50):
            error = target - self.fk(leg, q)
            if np.linalg.norm(error) < 2e-4:
                return q.tolist()
            jacobian = np.zeros((3, 3))
            epsilon = 1e-5
            for index in range(3):
                shifted = q.copy()
                shifted[index] += epsilon
                jacobian[:, index] = (self.fk(leg, shifted) - self.fk(leg, q)) / epsilon
            damping = 0.003
            delta = np.linalg.solve(
                jacobian.T @ jacobian + damping * damping * np.eye(3),
                jacobian.T @ error,
            )
            largest = float(np.max(np.abs(delta)))
            if largest > 0.12:
                delta *= 0.12 / largest
            q = np.clip(q + delta, bounds[:, 0], bounds[:, 1])
        residual = float(np.linalg.norm(target - self.fk(leg, q)))
        if residual > 0.002:
            raise ValueError(f'{leg.upper()} target is unreachable (error {residual:.4f} m)')
        return q.tolist()


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
        self.declare_parameter('swing_height_m', 0.05)
        self.declare_parameter('swing_fraction', 0.125)
        # The previous crawl trial's command-frame margin overestimated the
        # margin from measured foot/COM positions by ~15 mm. Keep extra planned
        # margin and gate liftoff using the measured support contacts below.
        self.declare_parameter('support_margin_m', 0.04)
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
        self.contact_subs = [self.create_subscription(
            ContactsState, f'/foot_contacts/{FOOT_LINKS[leg]}/bumper_states',
            lambda msg, leg=leg: self._contact_cb(leg, msg), 10) for leg in LEGS]
        self.joint_state = None
        self.create_subscription(
            JointState, str(self.get_parameter('joint_states_topic').value),
            self._on_joint_state, 10,
        )
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

    def _on_joint_state(self, msg):
        positions = dict(zip(msg.name, msg.position))
        if all(name in positions for name in JOINT_NAMES):
            self.joint_state = {name: float(positions[name]) for name in JOINT_NAMES}
            if self.home is None:
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
                self.last_output = {
                    name: self.joint_state[name] for name in JOINT_NAMES
                }
                self.get_logger().info(
                    'Validated bent start pose reached; keyboard walking is ready.'
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
        points = sorted({(float(point[0]), float(point[1])) for point in points})
        if len(points) <= 2:
            return points

        def cross(a, b, c):
            return ((b[0] - a[0]) * (c[1] - a[1])
                    - (b[1] - a[1]) * (c[0] - a[0]))

        lower = []
        for point in points:
            while len(lower) >= 2 and cross(lower[-2], lower[-1], point) <= 0.0:
                lower.pop()
            lower.append(point)
        upper = []
        for point in reversed(points):
            while len(upper) >= 2 and cross(upper[-2], upper[-1], point) <= 0.0:
                upper.pop()
            upper.append(point)
        return lower[:-1] + upper[:-1]

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
        hull = self._convex_hull(support_points)
        if len(hull) < 3:
            return -math.inf
        com = self.kinematics.com_xy_at(self.joint_state)
        distances = []
        for i, a_tuple in enumerate(hull):
            a = np.asarray(a_tuple)
            edge = np.asarray(hull[(i + 1) % len(hull)]) - a
            normal = np.array([-edge[1], edge[0]]) / np.linalg.norm(edge)
            distances.append(float(np.dot(com - a, normal)))
        return min(distances) if min(distances) >= 0.0 else -min(abs(d) for d in distances if d < 0.0)

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
                dx, dy, _ = self._leg_offset(other, elapsed)
                support_points.append(self.home_feet[other][:2] + np.array([dx, dy]))
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
        if self.home is None or self.home_feet is None:
            return
        if self.vx == 0.0 and self.vy == 0.0 and self.yaw == 0.0:
            # The trajectory bridge already holds the start pose. Avoid
            # streaming identical trajectories while idle; JTC can treat each
            # new message as a replacement trajectory.
            self.motion_started = None
            return

        if self.motion_started is None:
            self.motion_started = self.get_clock().now()
        elapsed = (self.get_clock().now() - self.motion_started).nanoseconds / 1e9
        cycle_phase = (elapsed / self.cycle_duration) % 1.0
        balance_shift = self._balance_shift(cycle_phase, elapsed)
        offsets = CRAWL_OFFSETS if self.gait == 'crawl' else TROT_OFFSETS
        stance = 1.0 - self.swing_fraction
        swinging = [leg for leg in LEGS
                    if (cycle_phase + offsets[leg]) % 1.0 >= stance]
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
        output = []
        try:
            for leg in LEGS:
                dx, dy, lift = self._leg_offset(leg, elapsed)
                phase = (cycle_phase + (CRAWL_OFFSETS if self.gait == 'crawl'
                                        else TROT_OFFSETS)[leg]) % 1.0
                if phase >= 1.0 - self.swing_fraction:
                    progress = (phase - (1.0 - self.swing_fraction)) / self.swing_fraction
                    blend = math.sin(math.pi * progress) ** 2
                    dx += SWING_CLEARANCE_XY[leg][0] * blend
                    dy += SWING_CLEARANCE_XY[leg][1] * blend
                target = self.home_feet[leg] + np.array([
                    dx + balance_shift[0], dy + balance_shift[1],
                    lift + balance_shift[2]
                ])
                names = self.kinematics.chains[leg][1]
                seed = [self.last_output[name] for name in names]
                output.extend(self.kinematics.ik(leg, target, seed))
        except (ValueError, np.linalg.LinAlgError) as exc:
            self.vx = self.vy = self.yaw = 0.0
            self.get_logger().error(f'Gait stopped; refusing invalid IK target: {exc}')
            return
        self._publish(output)

    def handle_key(self, key):
        if key in ('w', 's', 'a', 'd', 'q', 'e') and not all(
            self.contacts[leg] is True for leg in LEGS
        ):
            self.get_logger().warning(
                f'Keyboard movement refused until all four feet contact the ground: {self.contacts}'
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


def read_key():
    ready, _, _ = select.select([sys.stdin], [], [], 0.02)
    if not ready:
        return None
    return os.read(sys.stdin.fileno(), 1).decode(errors='ignore')


def main(args=None):
    if not sys.stdin.isatty():
        raise RuntimeError('Run keyboard_walk.py in an interactive terminal.')
    rclpy.init(args=args)
    node = KeyboardWalk()
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    print('\nKeyboard walk: w/s forward/back, a/d turn, q/e sidestep, SPACE stand/stop, x crouch (disabled), +/- stride, g crawl/trot, Ctrl+C quit.')
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
