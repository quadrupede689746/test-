#!/usr/bin/env python3
"""Transfer load, lift one foot, measure world height/contact, then restore stance."""
import csv
import math
import re
import sys
import time
from datetime import datetime
from pathlib import Path

import numpy as np
import rclpy
from control_msgs.msg import JointTrajectoryControllerState
from gazebo_msgs.msg import ContactsState, ModelStates
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from keyboard_walk import FOOT_LINKS, JOINT_NAMES, LEGS, UrdfLegKinematics

LEG_NAMES = {'fl': 'front_left', 'fr': 'front_right', 'bl': 'back_left', 'br': 'back_right'}
NAME_LEGS = {value: key for key, value in LEG_NAMES.items()}


def _rotate(q, vector):
    x, y, z, w = q.x, q.y, q.z, q.w
    rotation = np.array([
        [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
        [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
        [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)],
    ])
    return rotation @ np.asarray(vector, dtype=float)


def _rpy(q):
    sinr = 2 * (q.w*q.x + q.y*q.z)
    cosr = 1 - 2 * (q.x*q.x + q.y*q.y)
    roll = math.atan2(sinr, cosr)
    sinp = 2 * (q.w*q.y - q.z*q.x)
    pitch = math.asin(max(-1.0, min(1.0, sinp)))
    siny = 2 * (q.w*q.z + q.x*q.y)
    cosy = 1 - 2 * (q.y*q.y + q.z*q.z)
    return roll, pitch, math.atan2(siny, cosy)


class SingleFootTest(Node):
    def __init__(self):
        super().__init__('single_foot_test')
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.declare_parameter('lift_leg', 'back_left')
        self.declare_parameter('lift_m', 0.03)
        self.declare_parameter('duration_sec', 4.0)
        self.declare_parameter('preload_duration_sec', 3.0)
        self.declare_parameter('support_margin_min_m', 0.02)
        self.declare_parameter('support_margin_target_m', 0.035)
        self.declare_parameter('minimum_world_lift_m', 0.03)
        self.declare_parameter('output_dir', str(Path.home() / 'robodog_ws' / 'logs'))
        self.declare_parameter('model_name', 'open_quadruped')
        self.declare_parameter('wait_timeout_sec', 30.0)

        requested_leg = str(self.get_parameter('lift_leg').value)
        self.leg = NAME_LEGS.get(requested_leg, requested_leg)
        if self.leg not in LEGS:
            raise ValueError('lift_leg must be front_left/front_right/back_left/back_right')
        self.leg_name = LEG_NAMES[self.leg]
        self.lift_m = float(self.get_parameter('lift_m').value)
        self.duration = float(self.get_parameter('duration_sec').value)
        self.preload_duration = float(self.get_parameter('preload_duration_sec').value)
        self.margin_min = float(self.get_parameter('support_margin_min_m').value)
        self.margin_target = max(self.margin_min, float(
            self.get_parameter('support_margin_target_m').value
        ))
        self.minimum_world_lift = float(self.get_parameter('minimum_world_lift_m').value)
        self.timeout = float(self.get_parameter('wait_timeout_sec').value)
        if self.lift_m <= 0 or self.duration <= 0 or self.preload_duration <= 0:
            raise ValueError('lift_m and phase durations must be positive')

        self.kinematics = UrdfLegKinematics()
        self.model_name = str(self.get_parameter('model_name').value)
        self.positions = None
        self.model_pose = None
        self.controller = None
        self.contacts = {leg: None for leg in LEGS}
        self.all_contact_since = None
        self.collision_names = {leg: '' for leg in LEGS}
        self.phase = 'wait'
        self.test_start_foot_z = None
        self.swing_dx = 0.0
        self.swing_dy = 0.0
        self.max_lift_world_m = 0.0
        self.contact_lost_during_lift = False
        self.abort_reason = ''
        self.rows = 0

        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        safe_leg = re.sub(r'[^A-Za-z0-9_]+', '_', self.leg_name)
        out_dir = Path(str(self.get_parameter('output_dir').value)).expanduser()
        out_dir.mkdir(parents=True, exist_ok=True)
        self.csv_path = out_dir / f'{stamp}_foot_lift_{safe_leg}_{self.lift_m:.3f}m.csv'
        self.file = self.csv_path.open('w', newline='', buffering=1)
        self.writer = csv.DictWriter(self.file, fieldnames=self._columns())
        self.writer.writeheader()

        self.publisher = self.create_publisher(
            JointTrajectory, '/quadruped_controller/joint_trajectory', 10
        )
        self.status = self.create_publisher(String, '/keyboard_walk/status', 10)
        self.create_subscription(JointState, '/joint_states', self._joint_cb, 10)
        self.create_subscription(JointTrajectoryControllerState,
                                 '/quadruped_controller/controller_state', self._controller_cb, 10)
        self.create_subscription(ModelStates, '/gazebo/model_states', self._model_cb, 10)
        self.contact_subs = [self.create_subscription(
            ContactsState, f'/foot_contacts/{FOOT_LINKS[leg]}/bumper_states',
            lambda msg, leg=leg: self._contact_cb(leg, msg), 10) for leg in LEGS]
        self.create_subscription(String, '/robot_logger/safety_stop', self._safety_cb, 10)
        self.timer = self.create_timer(
            0.05, self._sample, clock=Clock(clock_type=ClockType.STEADY_TIME)
        )

    @staticmethod
    def _columns():
        cols = ['wall_sec', 'sim_sec', 'phase', 'lift_leg', 'target_lift_m',
                'target_dx_m', 'target_dy_m',
                'foot_world_z_m', 'foot_world_delta_z_m', 'base_x_m', 'base_y_m', 'base_z_m',
                'roll_rad', 'pitch_rad', 'support_margin_m', 'support_inside',
                'max_tracking_error_rad']
        for leg in LEGS:
            cols += [f'{LEG_NAMES[leg]}_contact', f'{LEG_NAMES[leg]}_collision_pair']
        for name in JOINT_NAMES:
            cols += [f'{name}_position_rad', f'{name}_error_rad']
        return cols

    def _joint_cb(self, msg):
        values = dict(zip(msg.name, msg.position))
        if all(name in values for name in JOINT_NAMES):
            self.positions = {name: float(values[name]) for name in JOINT_NAMES}

    def _controller_cb(self, msg):
        names = list(msg.joint_names)
        commands = dict(zip(names, msg.reference.positions))
        measured = dict(zip(names, msg.feedback.positions))
        self.controller = {n: commands[n] - measured[n] for n in names
                           if n in commands and n in measured}

    def _model_cb(self, msg):
        if self.model_name not in msg.name:
            return
        self.model_pose = msg.pose[msg.name.index(self.model_name)]

    def _contact_cb(self, leg, msg):
        self.contacts[leg] = bool(msg.states)
        pairs = sorted({f'{s.collision1_name}|{s.collision2_name}' for s in msg.states})
        self.collision_names[leg] = ';'.join(pairs)
        if all(value is True for value in self.contacts.values()):
            if self.all_contact_since is None:
                self.all_contact_since = time.monotonic()
        else:
            self.all_contact_since = None

    def _safety_cb(self, msg):
        self.abort_reason = f'logger safety stop: {msg.data}'

    def _q(self, leg, positions=None):
        positions = positions or self.positions
        return [positions[name] for name in self.kinematics.chains[leg][1]]

    def _foot_local(self, leg, positions=None):
        return self.kinematics.fk(leg, self._q(leg, positions))

    def _foot_world(self, leg):
        if self.model_pose is None or self.positions is None:
            return None
        p = self.model_pose.position
        return np.array([p.x, p.y, p.z]) + _rotate(
            self.model_pose.orientation, self._foot_local(leg)
        )

    def _com_world(self):
        if self.model_pose is None:
            return None
        p = self.model_pose.position
        return np.array([p.x, p.y, p.z]) + _rotate(
            self.model_pose.orientation, self.kinematics.com_xyz_at(self.positions)
        )

    @staticmethod
    def _hull(points):
        pts = sorted({(float(p[0]), float(p[1])) for p in points})
        if len(pts) <= 2:
            return pts
        def cross(a, b, c):
            return (b[0]-a[0])*(c[1]-a[1]) - (b[1]-a[1])*(c[0]-a[0])
        lower, upper = [], []
        for p in pts:
            while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
                lower.pop()
            lower.append(p)
        for p in reversed(pts):
            while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
                upper.pop()
            upper.append(p)
        return lower[:-1] + upper[:-1]

    @classmethod
    def _margin(cls, points, com):
        hull = cls._hull(points)
        if len(hull) < 3:
            return False, -math.inf
        signed = []
        for i, a in enumerate(hull):
            b = hull[(i + 1) % len(hull)]
            edge = np.asarray(b) - np.asarray(a)
            length = float(np.linalg.norm(edge))
            cross = edge[0] * (com[1] - a[1]) - edge[1] * (com[0] - a[0])
            signed.append(float(cross / length))
        inside = min(signed) >= -1e-8
        if inside:
            return True, min(signed)
        return False, -min(abs(value) for value in signed if value < 0.0)

    def _current_support(self):
        com = self._com_world()
        points = [self._foot_world(leg)[:2] for leg in LEGS if leg != self.leg]
        if com is None or any(p is None for p in points):
            return False, -math.inf
        return self._margin(points, com[:2])

    def _support_target_offset(self, local_support, com_xy):
        hull = self._hull(local_support)
        inside, _ = self._margin(hull, com_xy)
        if len(hull) != 3:
            raise RuntimeError('The three supporting feet do not form a triangle.')
        edge_data = []
        for i, a_tuple in enumerate(hull):
            a = np.asarray(a_tuple)
            b = np.asarray(hull[(i + 1) % len(hull)])
            edge = b - a
            normal = np.array([-edge[1], edge[0]]) / np.linalg.norm(edge)
            distance = float(np.dot(com_xy - a, normal))
            edge_data.append((distance, normal, a, edge))
        distance, inward_normal, _, _ = min(edge_data, key=lambda row: row[0])
        if inside:
            # Shared foot targets move opposite to desired body/COM motion.
            return -inward_normal * max(0.0, self.margin_target - distance)
        nearest, nearest_distance = None, math.inf
        for _, normal, a, edge in edge_data:
            frac = float(np.clip(np.dot(com_xy - a, edge) / np.dot(edge, edge), 0, 1))
            point = a + frac * edge
            d = float(np.linalg.norm(com_xy - point))
            if d < nearest_distance:
                nearest, nearest_distance, inward_normal = point, d, normal
        return com_xy - nearest - self.margin_target * inward_normal

    def _publish(self, positions, duration, key):
        msg = JointTrajectory()
        # A long IK search can span several simulated seconds on this host.
        # A zero stamp makes the controller start the new segment on receipt
        # instead of rejecting a timestamp that has already elapsed.
        msg.joint_names = JOINT_NAMES
        point = JointTrajectoryPoint()
        point.positions = [float(positions[name]) for name in JOINT_NAMES]
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
        msg.points = [point]
        self.status.publish(String(data=f'key={key};vx=0;vy=0;yaw=0'))
        self.publisher.publish(msg)

    def _sample(self):
        if self.positions is None or self.model_pose is None:
            return
        pose = self.model_pose
        p = pose.position
        roll, pitch, _ = _rpy(pose.orientation)
        foot = self._foot_world(self.leg)
        inside, margin = self._current_support()
        delta_z = (float(foot[2]) - self.test_start_foot_z
                   if foot is not None and self.test_start_foot_z is not None else '')
        max_error = max((abs(v) for v in (self.controller or {}).values()), default=float('nan'))
        row = {'wall_sec': time.monotonic(), 'sim_sec': self.get_clock().now().nanoseconds / 1e9,
               'phase': self.phase, 'lift_leg': self.leg_name, 'target_lift_m': self.lift_m,
               'target_dx_m': self.swing_dx, 'target_dy_m': self.swing_dy,
               'foot_world_z_m': float(foot[2]) if foot is not None else '',
               'foot_world_delta_z_m': delta_z, 'base_x_m': p.x, 'base_y_m': p.y, 'base_z_m': p.z,
               'roll_rad': roll, 'pitch_rad': pitch, 'support_margin_m': margin,
               'support_inside': inside, 'max_tracking_error_rad': max_error}
        for leg in LEGS:
            row[f'{LEG_NAMES[leg]}_contact'] = self.contacts[leg]
            row[f'{LEG_NAMES[leg]}_collision_pair'] = self.collision_names[leg]
        for name in JOINT_NAMES:
            row[f'{name}_position_rad'] = self.positions.get(name, '')
            row[f'{name}_error_rad'] = (self.controller or {}).get(name, '')
        self.writer.writerow(row)
        self.rows += 1
        if self.phase == 'lift' and delta_z != '':
            self.max_lift_world_m = max(self.max_lift_world_m, float(delta_z))
            if self.contacts[self.leg] is False:
                self.contact_lost_during_lift = True

    def _spin_for(self, duration):
        deadline = time.monotonic() + duration
        while rclpy.ok() and time.monotonic() < deadline and not self.abort_reason:
            rclpy.spin_once(self, timeout_sec=0.02)

    def _wait_ready(self):
        deadline = time.monotonic() + self.timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if (self.positions and self.model_pose and self.controller
                    and self.all_contact_since is not None
                    and time.monotonic() - self.all_contact_since >= 0.5):
                return
        raise RuntimeError('Timed out waiting for joint/model/controller/contact feedback.')

    def _make_joint_target(self, feet_targets):
        result = dict(self.positions)
        for leg in LEGS:
            names = self.kinematics.chains[leg][1]
            seed = [self.positions[name] for name in names]
            q = self.kinematics.ik(leg, feet_targets[leg], seed)
            result.update(dict(zip(names, q)))
        return result

    def _reachable_lift_target(self, start_foot, seed):
        """Find the smallest horizontal foot adjustment that makes the lift reachable."""
        names = self.kinematics.chains[self.leg][1]
        for radius in np.arange(0.0, 0.081, 0.005):
            candidates = [(0.0, 0.0)] if radius == 0 else [
                (radius * math.cos(angle), radius * math.sin(angle))
                for angle in np.arange(0.0, 2.0 * math.pi, math.pi / 12.0)
            ]
            reachable = []
            for dx, dy in candidates:
                target = start_foot + np.array([dx, dy, self.lift_m])
                try:
                    q = np.asarray(self.kinematics.ik(self.leg, target, seed), dtype=float)
                except (ValueError, np.linalg.LinAlgError):
                    continue
                limit_margin = min(
                    min(q[i] - self.kinematics.limits[name][0],
                        self.kinematics.limits[name][1] - q[i])
                    for i, name in enumerate(names)
                )
                if limit_margin >= 0.05:
                    reachable.append((limit_margin, dx, dy, q))
            if reachable:
                reachable.sort(key=lambda item: (-item[0], item[1]**2 + item[2]**2))
                _, self.swing_dx, self.swing_dy, q = reachable[0]
                return start_foot + np.array([self.swing_dx, self.swing_dy, self.lift_m]), q
        raise RuntimeError('No reachable lift target found within 8 cm horizontal adjustment '
                           'while keeping at least 0.05 rad joint-limit margin.')

    def _restore(self, baseline):
        supporting_contacts = all(self.contacts[leg] for leg in LEGS if leg != self.leg)
        safe_pose = self.model_pose is not None and self.model_pose.position.z > 0.14
        if safe_pose:
            roll, pitch, _ = _rpy(self.model_pose.orientation)
            safe_pose = abs(roll) < 0.35 and abs(pitch) < 0.35
        if not supporting_contacts or not safe_pose:
            self.phase = 'aborted_hold'
            print('Skipping baseline restore: support contact or body attitude is unsafe; '
                  'leaving the last measured joint pose commanded.', flush=True)
            hold = dict(self.positions)
            self._publish(hold, 1.0, 'unsafe_hold')
            self._spin_for(1.2)
            return
        self.phase = 'restore'
        self._publish(baseline, 3.0, 'foot_lift_restore')
        self._spin_for(4.0)
        self.phase = 'done'

    def run_test(self):
        self._wait_ready()
        baseline = dict(self.positions)
        if not all(self.contacts.values()):
            raise RuntimeError(f'All four feet must start in contact: {self.contacts}')
        initial_com = self._com_world()
        initial_inside, initial_margin = self._current_support()
        self.test_start_foot_z = float(self._foot_world(self.leg)[2])
        print(f'Initial support triangle without {self.leg_name}: inside={initial_inside}, '
              f'margin={initial_margin:.4f} m; COM-world={initial_com[:2].tolist()} m.', flush=True)

        inside, margin = initial_inside, initial_margin
        for transfer_attempt in range(1, 4):
            if inside and margin >= self.margin_min:
                break
            local_support = [self._foot_local(leg)[:2] for leg in LEGS if leg != self.leg]
            com_xy = self.kinematics.com_xy_at(self.positions)
            offset = self._support_target_offset(local_support, com_xy.copy())
            predicted = self._margin([p + offset for p in local_support], com_xy)
            print(f'Weight-transfer pass {transfer_attempt}: relative foot target shift '
                  f'[{offset[0]:.4f}, {offset[1]:.4f}] m; predicted margin={predicted[1]:.4f} m.',
                  flush=True)
            targets = {leg: self._foot_local(leg) + np.array([offset[0], offset[1], 0.0])
                       for leg in LEGS}
            self.phase = 'weight_transfer'
            self._publish(self._make_joint_target(targets), self.preload_duration, 'weight_transfer')
            self._spin_for(self.preload_duration + 1.0)
            inside, margin = self._current_support()
            print(f'Measured after transfer {transfer_attempt}: inside={inside}, '
                  f'margin={margin:.4f} m; contacts={self.contacts}.', flush=True)
            if not all(self.contacts.values()):
                break
        if self.abort_reason:
            self._restore(baseline)
            raise RuntimeError(self.abort_reason)
        if not inside or margin < self.margin_min:
            self._restore(baseline)
            raise RuntimeError(f'Measured support margin {margin:.4f} m is below '
                               f'{self.margin_min:.4f} m; lift refused.')
        if not all(self.contacts.values()):
            self._restore(baseline)
            raise RuntimeError(f'A foot lost contact during weight transfer: {self.contacts}')

        self.test_start_foot_z = float(self._foot_world(self.leg)[2])
        lift_targets = dict(self.positions)
        start_foot = self._foot_local(self.leg)
        leg_names = self.kinematics.chains[self.leg][1]
        foot_target, q_lift = self._reachable_lift_target(
            start_foot, [self.positions[name] for name in leg_names]
        )
        print(f'Reachable swing target includes horizontal shift '
              f'[{self.swing_dx:.3f}, {self.swing_dy:.3f}] m.', flush=True)
        print('Measured and commanded lifted-leg joints: '
              f'{np.round([self.positions[name] for name in leg_names], 4).tolist()} -> '
              f'{np.round(q_lift, 4).tolist()}; target FK={np.round(foot_target, 4).tolist()}.',
              flush=True)
        lift_targets.update(dict(zip(leg_names, q_lift)))
        self.phase = 'lift'
        self._publish(lift_targets, self.duration, 'single_foot_test')
        self._spin_for(self.duration + 1.5)
        if self.abort_reason:
            self._restore(baseline)
            raise RuntimeError(self.abort_reason)

        lift_ok = (self.max_lift_world_m >= self.minimum_world_lift
                   and self.contact_lost_during_lift)
        print(f'Lift measurement: target={self.lift_m:.3f} m, minimum actual clearance='
              f'{self.minimum_world_lift:.3f} m, max world rise='
              f'{self.max_lift_world_m:.4f} m, contact cleared={self.contact_lost_during_lift}, '
              f'contacts={self.contacts}. Result={"PASS" if lift_ok else "FAIL"}.', flush=True)
        self._restore(baseline)
        self.file.close()
        print(f'Detailed measurements: {self.csv_path} ({self.rows} rows).', flush=True)
        if not lift_ok:
            raise RuntimeError('Foot did not achieve the requested world lift with contact cleared.')


def main(args=None):
    rclpy.init(args=args)
    node = SingleFootTest()
    try:
        node.run_test()
    except Exception as exc:
        print(f'Single-foot test failed: {exc}', file=sys.stderr, flush=True)
        try:
            node.file.close()
        except Exception:
            pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
