"""URDF-derived quadruped leg kinematics and mass properties."""
import math
import os
import subprocess
import xml.etree.ElementTree as ET
import numpy as np
from ament_index_python.packages import get_package_share_directory

LEGS = ('fl', 'fr', 'bl', 'br')
FOOT_LINKS = {'fl': 'front_left_foot', 'fr': 'front_right_foot', 'bl': 'back_left_foot', 'br': 'back_right_foot'}

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

