"""
DogController - runs a trained policy on a REAL or SIMULATED robot (Gazebo, hardware, ...). No ROS, no MuJoCo.
It rebuilds exactly the observation the policy saw during training (see dog_env.py) and turns the policy output
into 12 joint position targets.  Joint order everywhere: FL, FR, BL, BR  x  (hip, upper_leg, lower_leg).
Keep the constants below in sync with dog_env.py.
"""
import os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from export import NpzPolicy

STAND = np.array([0, -0.68162889, 1.26606748] * 4)
SIT = np.array([[0, -0.2, 0.5]] * 2 + [[0, -1.57, 1.57]] * 2).ravel()
LIE = np.array([[0.9, -1.0, 1.55], [-0.9, -1.0, 1.55], [0.9, -1.0, 1.55], [-0.9, -1.0, 1.55]]).ravel()
POSES = [STAND, SIT, LIE]
ACTION_SCALE, ACTION_FILTER, REF_RATE = 0.5, 0.6, 0.5
GAIT_PERIOD = 0.4
LO = np.array([-1.04, -1.5708, -2.9] * 4); HI = np.array([1.04, 2.59, 1.5708] * 4)
CMD_MAX = np.array([0.25, 0.12, 0.8]); CMD_MIN = np.array([-0.15, -0.12, -0.8])   # ranges seen in training
MODES = {"walk": 0, "stand": 0, "sit": 1, "lie": 2}


class DogController:
    def __init__(self, policy_path, legacy_gyro=True):
        self.policy = NpzPolicy(policy_path); self.legacy_gyro = legacy_gyro
        self.reset()

    def reset(self):
        self.mode = 0; self.cmd = np.zeros(3); self.phase = 0.0
        self.a_f = np.zeros(12); self.last_action = np.zeros(12)
        self.ref = None; self.engaged = False

    def set_command(self, vx=0.0, vy=0.0, wz=0.0, mode="walk"):
        self.cmd = np.clip([vx, vy, wz], CMD_MIN, CMD_MAX); self.mode = MODES[mode]

    def moving(self):
        return self.mode == 0 and (np.linalg.norm(self.cmd[:2]) > 0.02 or abs(self.cmd[2]) > 0.05)

    @staticmethod
    def rot(quat_wxyz):
        w, x, y, z = quat_wxyz / np.linalg.norm(quat_wxyz)
        return np.array([[1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
                         [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
                         [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)]])

    def observe(self, q, qd, quat_wxyz, gyro_body):
        R = self.rot(np.asarray(quat_wxyz, float))
        gravity = R.T @ np.array([0, 0, -1.0])
        gyro = (R.T @ gyro_body) if self.legacy_gyro else np.asarray(gyro_body, float)
        clock = [np.sin(2*np.pi*self.phase), np.cos(2*np.pi*self.phase)] if self.moving() else [0.0, 0.0]
        return np.concatenate([gravity, gyro * 0.25, q - STAND, qd * 0.05, self.last_action,
                               self.cmd * np.array([2.0, 2.0, 1.0]), np.eye(3)[self.mode], clock]).astype(np.float32)

    def upright(self, quat_wxyz):
        return float(self.rot(np.asarray(quat_wxyz, float))[2, 2])

    def update(self, q, qd, quat_wxyz, gyro_body, dt=0.02):
        """Call at 50 Hz. Returns the 12 joint position targets (radians)."""
        q = np.asarray(q, float); qd = np.asarray(qd, float)
        goal = POSES[self.mode]
        if self.ref is None:
            self.ref = np.clip(q, LO, HI).copy()
        # stand-up phase: glide from wherever the legs are to the reference pose, policy NOT in control yet
        if not self.engaged:
            self.ref = self.ref + np.clip(goal - self.ref, -REF_RATE * dt, REF_RATE * dt)
            if np.max(np.abs(self.ref - goal)) < 1e-3 and np.max(np.abs(q - goal)) < 0.15:
                self.engaged = True
            return np.clip(self.ref, LO, HI)
        a = np.clip(self.policy(self.observe(q, qd, quat_wxyz, gyro_body)), -1, 1)
        self.a_f = self.a_f + ACTION_FILTER * (a - self.a_f)
        self.ref = self.ref + np.clip(goal - self.ref, -REF_RATE * dt, REF_RATE * dt)
        target = np.clip(self.ref + ACTION_SCALE * self.a_f, LO, HI)
        self.last_action = a
        self.phase = (self.phase + dt / GAIT_PERIOD) % 1.0
        return target
