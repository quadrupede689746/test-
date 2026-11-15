#!/usr/bin/env python3
"""Sample Gazebo pose, ros2_control state, foot contacts and keyboard commands to CSV."""

import csv
import math
import os
import re
import subprocess
import xml.etree.ElementTree as ET
import time
from datetime import datetime
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from control_msgs.msg import JointTrajectoryControllerState
from gazebo_msgs.msg import ContactsState, ModelStates
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.clock import Clock, ClockType
from sensor_msgs.msg import JointState
from std_msgs.msg import String

JOINTS = [f'motor_{leg}_{joint}' for leg in (
    'front_left', 'front_right', 'back_left', 'back_right')
    for joint in ('hip', 'upper_leg', 'lower_leg')]
LEGS = ('front_left', 'front_right', 'back_left', 'back_right')


def _q_to_rpy(q):
    sinr = 2.0 * (q.w * q.x + q.y * q.z)
    cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr, cosr)
    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    pitch = math.asin(max(-1.0, min(1.0, sinp)))
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return roll, pitch, math.atan2(siny, cosy)


def _finite_or_empty(value):
    return value if value is not None and math.isfinite(value) else ''


class RobotLogger(Node):
    def __init__(self):
        super().__init__('robot_logger')
        result = self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])[0]
        if not result.successful:
            raise RuntimeError(f'Cannot enable simulated time: {result.reason}')
        self.declare_parameter('trial_name', 'trial')
        self.declare_parameter('output_dir', str(Path.home() / 'robodog_ws' / 'logs'))
        self.declare_parameter('sample_rate_hz', 50.0)
        self.declare_parameter('model_name', 'open_quadruped')
        self.declare_parameter('max_joint_effort_nm', 4.0)
        self.declare_parameter('fall_z_m', 0.1)
        self.declare_parameter('fall_angle_rad', 0.8)
        self.declare_parameter('max_joint_speed_rad_s', 20.0)
        self.declare_parameter('max_tracking_error_rad', 0.3)
        self.declare_parameter('step_length_m', 0.01)
        self.declare_parameter('swing_height_m', 0.005)
        self.declare_parameter('cycle_duration_sec', 16.0)
        self.declare_parameter('gait', 'crawl')
        self.declare_parameter('keyboard_topic', '/keyboard_walk/status')
        self.declare_parameter('controller_state_topic', '/quadruped_controller/controller_state')
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('model_states_topic', '/gazebo/model_states')
        self.declare_parameter('contact_topic_prefix', '/foot_contacts')

        self.rate = min(100.0, max(1.0, float(self.get_parameter('sample_rate_hz').value)))
        self.model_name = str(self.get_parameter('model_name').value)
        self.effort_limit = float(self.get_parameter('max_joint_effort_nm').value)
        self.output_dir = Path(str(self.get_parameter('output_dir').value)).expanduser()
        self.output_dir.mkdir(parents=True, exist_ok=True)
        label = re.sub(r'[^A-Za-z0-9_.-]+', '_', str(self.get_parameter('trial_name').value)).strip('._') or 'trial'
        self.stem = datetime.now().strftime('%Y%m%d_%H%M%S') + '_' + label
        self.csv_path = self.output_dir / f'{self.stem}.csv'
        self.summary_path = self.output_dir / f'{self.stem}_summary.txt'
        self.params_path = self.output_dir / f'{self.stem}_params.txt'

        self.model = None
        self.joint_measured = {}
        self.joint_velocities = {}
        self.joint_efforts = {}
        self.controller = None
        self.contacts = {leg: None for leg in LEGS}
        self.contact_forces = {leg: None for leg in LEGS}
        self.ever_contact = {leg: False for leg in LEGS}
        self.command = 'unknown'
        self.first_events = {}
        self.first_anomaly = None
        self.last_sample = None
        self.samples = 0
        self.safety_latched = False
        self.wall_start = time.monotonic()

        self.create_subscription(ModelStates, str(self.get_parameter('model_states_topic').value), self._model_cb, 10)
        self.create_subscription(JointState, str(self.get_parameter('joint_states_topic').value), self._joint_cb, 10)
        self.create_subscription(JointTrajectoryControllerState, str(self.get_parameter('controller_state_topic').value), self._controller_cb, 10)
        self.create_subscription(String, str(self.get_parameter('keyboard_topic').value), self._command_cb, 10)
        prefix = str(self.get_parameter('contact_topic_prefix').value).rstrip('/')
        self.contact_subs = [self.create_subscription(
            ContactsState, f'{prefix}/{leg}_foot/bumper_states',
            lambda msg, leg=leg: self._contact_cb(leg, msg), 10)
            for leg in LEGS]
        self.safety_pub = self.create_publisher(String, '/robot_logger/safety_stop', 10)

        self.file = self.csv_path.open('w', newline='', buffering=1)
        self.writer = csv.DictWriter(self.file, fieldnames=self._columns())
        self.writer.writeheader()
        self._write_params()
        # Sample on wall time so Gazebo slowing under contact-sensor load does
        # not silently reduce the requested CSV rate. Each row still carries
        # the exact simulation timestamp for alignment with ROS data.
        self.timer = self.create_timer(1.0 / self.rate, self._sample,
                                      clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info(f'Logging at {self.rate:.1f} Hz to {self.csv_path}')

    @staticmethod
    def _columns():
        columns = ['wall_time_sec', 'sim_time_sec', 'x_m', 'y_m', 'z_m', 'roll_rad', 'pitch_rad', 'yaw_rad',
                   'linear_vx_m_s', 'linear_vy_m_s', 'linear_vz_m_s',
                   'angular_wx_rad_s', 'angular_wy_rad_s', 'angular_wz_rad_s',
                   'keyboard_command', 'requested_vx', 'requested_vy', 'requested_yaw']
        for name in JOINTS:
            columns += [f'{name}_command_rad', f'{name}_position_rad',
                        f'{name}_error_rad', f'{name}_velocity_rad_s', f'{name}_effort_nm']
        for leg in LEGS:
            columns += [f'{leg}_contact', f'{leg}_contact_force_n']
        return columns

    def _model_cb(self, msg):
        if self.model_name not in msg.name:
            return
        i = msg.name.index(self.model_name)
        self.model = (msg.pose[i], msg.twist[i])

    def _joint_cb(self, msg):
        self.joint_measured = {name: msg.position[i] for i, name in enumerate(msg.name) if i < len(msg.position)}
        self.joint_velocities = {name: msg.velocity[i] for i, name in enumerate(msg.name) if i < len(msg.velocity)}
        self.joint_efforts = {name: msg.effort[i] for i, name in enumerate(msg.name) if i < len(msg.effort)}

    def _controller_cb(self, msg):
        self.controller = msg

    def _command_cb(self, msg):
        self.command = msg.data

    def _contact_cb(self, leg, msg):
        self.contacts[leg] = bool(msg.states)
        magnitudes = []
        for state in msg.states:
            wrench = state.total_wrench
            magnitudes.append(math.sqrt(wrench.force.x ** 2 + wrench.force.y ** 2 + wrench.force.z ** 2))
        # ContactState contains one entry per physics-step contact within the
        # bumper update window; summing them would multiply the same load.
        self.contact_forces[leg] = max(magnitudes, default=0.0)
        if msg.states:
            self.ever_contact[leg] = True

    def _controller_maps(self):
        if self.controller is None:
            return {}, {}, {}, {}
        msg = self.controller
        names = list(msg.joint_names)
        command = dict(zip(names, msg.reference.positions))
        feedback = dict(zip(names, msg.feedback.positions))
        velocity = dict(zip(names, msg.feedback.velocities))
        for name, value in self.joint_velocities.items():
            velocity.setdefault(name, value)
        effort = dict(zip(names, msg.output.effort))
        return command, feedback, velocity, effort

    def _event(self, event, sim_time, joint='', detail=''):
        if event in self.first_events:
            return
        item = {'time': sim_time, 'joint': joint, 'detail': detail}
        self.first_events[event] = item
        if self.first_anomaly is None:
            self.first_anomaly = (event, item)
        self.get_logger().warning(f'First {event} at t={sim_time:.3f}s; joint={joint or "none"}; {detail}')

    def _sample(self):
        now = self.get_clock().now()
        sim_time = now.nanoseconds / 1e9
        if self.model is None:
            return
        pose, twist = self.model
        roll, pitch, yaw = _q_to_rpy(pose.orientation)
        command, measured, velocity, output_effort = self._controller_maps()
        fall_z = float(self.get_parameter('fall_z_m').value)
        fall_angle = float(self.get_parameter('fall_angle_rad').value)
        speed_limit = float(self.get_parameter('max_joint_speed_rad_s').value)
        error_limit = float(self.get_parameter('max_tracking_error_rad').value)
        if pose.position.z < fall_z or abs(roll) > fall_angle or abs(pitch) > fall_angle:
            self._event('fall', sim_time, detail=f'z={pose.position.z:.4f}, roll={roll:.4f}, pitch={pitch:.4f}')
            self._publish_safety('fall')

        row = {
            'wall_time_sec': time.monotonic() - self.wall_start,
            'sim_time_sec': sim_time, 'x_m': pose.position.x, 'y_m': pose.position.y, 'z_m': pose.position.z,
            'roll_rad': roll, 'pitch_rad': pitch, 'yaw_rad': yaw,
            'linear_vx_m_s': twist.linear.x, 'linear_vy_m_s': twist.linear.y, 'linear_vz_m_s': twist.linear.z,
            'angular_wx_rad_s': twist.angular.x, 'angular_wy_rad_s': twist.angular.y, 'angular_wz_rad_s': twist.angular.z,
            'keyboard_command': self.command,
        }
        vx = vy = wz = ''
        match = re.search(r'(?:^|;)vx=([-+0-9.eE]+);vy=([-+0-9.eE]+);yaw=([-+0-9.eE]+)', self.command)
        if match:
            vx, vy, wz = map(float, match.groups())
        row.update({'requested_vx': vx, 'requested_vy': vy, 'requested_yaw': wz})
        for name in JOINTS:
            cmd = command.get(name)
            pos = measured.get(name, self.joint_measured.get(name))
            vel = velocity.get(name, None)
            if vel is None:
                vel = self.joint_velocities.get(name)
            err = (cmd - pos) if cmd is not None and pos is not None else None
            effort = output_effort.get(name, self.joint_efforts.get(name))
            row[f'{name}_command_rad'] = _finite_or_empty(cmd)
            row[f'{name}_position_rad'] = _finite_or_empty(pos)
            row[f'{name}_error_rad'] = _finite_or_empty(err)
            row[f'{name}_velocity_rad_s'] = _finite_or_empty(vel)
            row[f'{name}_effort_nm'] = _finite_or_empty(effort)
            if vel is not None and math.isfinite(vel) and abs(vel) > speed_limit:
                self._event('joint_speed', sim_time, name, f'velocity={vel:.4f} rad/s')
                self._publish_safety(f'joint_speed:{name}')
            if err is not None and abs(err) > error_limit:
                self._event('tracking_error', sim_time, name, f'command={cmd:.4f}, measured={pos:.4f}, error={err:.4f} rad')
            if effort is not None and math.isfinite(effort) and abs(effort) >= self.effort_limit:
                self._event('effort_saturation', sim_time, name, f'effort={effort:.4f} Nm, limit={self.effort_limit:.4f} Nm')
        for leg in LEGS:
            contact = self.contacts[leg]
            row[f'{leg}_contact'] = '' if contact is None else int(contact)
            row[f'{leg}_contact_force_n'] = _finite_or_empty(self.contact_forces[leg])
            if contact is False and self.ever_contact[leg]:
                self._event('foot_contact_loss', sim_time, detail=f'{leg} contact lost')

        self.writer.writerow(row)
        self.samples += 1
        self.last_sample = row

    def _publish_safety(self, reason):
        if not self.safety_latched and (reason.startswith('fall') or reason.startswith('joint_speed')):
            self.safety_latched = True
            self.safety_pub.publish(String(data=reason))

    def _write_params(self):
        share = Path(get_package_share_directory('open_quadruped'))
        controller_path = share / 'config' / 'ros2_controllers.yaml'
        world_path = share / 'worlds' / 'crawl.world'
        urdf_path = share / 'urdf' / 'spot.urdf.xacro'
        p = self.params_path.open('w')
        p.write(f'trial={self.stem}\nsample_rate_hz={self.rate}\nmodel_name={self.model_name}\n')
        p.write(f'effort_limit_nm={self.effort_limit}\n')
        for key in ('fall_z_m', 'fall_angle_rad', 'max_joint_speed_rad_s', 'max_tracking_error_rad',
                    'keyboard_topic', 'controller_state_topic', 'contact_topic_prefix',
                    'step_length_m', 'swing_height_m', 'cycle_duration_sec', 'gait'):
            p.write(f'{key}={self.get_parameter(key).value}\n')
        for label, path in [('controller_config', controller_path), ('world', world_path)]:
            p.write(f'\n[{label}] {path}\n{path.read_text()}\n')
        p.write(f'\n[URDF mass, effort, velocity, dimensions] {urdf_path}\n')
        source = urdf_path.read_text()
        for line_no, line in enumerate(source.splitlines(), 1):
            if any(token in line for token in ('mass=', 'effort=', 'velocity=', 'length=', 'width=', 'height=', 'radius=')):
                p.write(f'{line_no}: {line.strip()}\n')
        try:
            generated = subprocess.run(['xacro', str(urdf_path)], check=True,
                                       capture_output=True, text=True, timeout=10).stdout
            robot = ET.fromstring(generated)
            p.write('\n[Expanded per-link mass and inertia]\n')
            for link in robot.findall('link'):
                inertial = link.find('inertial')
                if inertial is None:
                    continue
                mass = inertial.find('mass').get('value')
                inertia = inertial.find('inertia')
                origin = inertial.find('origin')
                p.write(f'{link.get("name")}: mass_kg={mass}; inertia_kg_m2={inertia.attrib}; '
                        f'inertial_origin_xyz={origin.get("xyz") if origin is not None else "0 0 0"}; '
                        f'inertial_origin_rpy={origin.get("rpy") if origin is not None else "0 0 0"}\n')
            p.write('\n[Expanded joint effort and velocity limits]\n')
            for joint in robot.findall('joint'):
                limit = joint.find('limit')
                if limit is not None and joint.get('type') == 'revolute':
                    p.write(f'{joint.get("name")}: {limit.attrib}\n')
        except Exception as exc:
            p.write(f'\nexpanded_urdf_parse_error={exc}\n')
        p.close()

    def _finish(self):
        try:
            self.file.flush()
            self.file.close()
        except Exception:
            pass
        with self.summary_path.open('w') as f:
            f.write(f'trial={self.stem}\nsamples={self.samples}\n')
            if self.first_anomaly:
                event, item = self.first_anomaly
                probable = {
                    'fall': 'body lost support or unstable contact/actuation',
                    'joint_speed': 'joint command tracking or physics instability',
                    'tracking_error': 'controller cannot track the requested trajectory',
                    'effort_saturation': 'measured actuator effort reached configured limit',
                    'foot_contact_loss': 'foot unloaded or lost ground contact',
                }.get(event, 'inspect synchronized CSV signals')
                f.write(f'first_anomaly={event}; t={item["time"]:.6f}; joint={item["joint"] or "none"}; detail={item["detail"]}; probable={probable}\n')
            else:
                f.write('first_anomaly=none detected\n')
            for event, item in self.first_events.items():
                f.write(f'first_{event}=t={item["time"]:.6f}; joint={item["joint"] or "none"}; {item["detail"]}\n')
        if self.first_anomaly:
            event, item = self.first_anomaly
            probable = {
                'fall': 'perte de support ou instabilité du contact/de l’actionnement',
                'joint_speed': 'commande articulaire non suivie ou instabilité physique',
                'tracking_error': 'le contrôleur ne suit pas la consigne de trajectoire',
                'effort_saturation': 'limite d’effort configurée atteinte',
                'foot_contact_loss': 'le pied s’est déchargé ou a perdu le contact au sol',
            }.get(event, 'inspecter les signaux synchronisés du CSV')
            print(f'Première anomalie à t={item["time"]:.3f}s, joint={item["joint"] or "aucun"}, cause probable={probable}', flush=True)
        else:
            print('Première anomalie : aucune détectée pendant cet essai.', flush=True)
        print(f'CSV: {self.csv_path}; résumé: {self.summary_path}; paramètres: {self.params_path}', flush=True)


def main(args=None):
    rclpy.init(args=args)
    node = RobotLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError as exc:
        # Humble may raise during a SIGINT while DDS is taking a queued sensor
        # message. Preserve and close the already sampled log cleanly.
        print(f'ROS executor stopped during shutdown: {exc}', flush=True)
    finally:
        node._finish()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
