"""
DogEnv - MuJoCo environment for the robodog. The policy gets a COMMAND and must obey it.

  command = (mode, vx, vy, wz)
     mode 0 = walk/stand : follow body-frame velocity vx (m/s, +forward), vy (m/s, +left), wz (rad/s, +turn left)
     mode 1 = sit
     mode 2 = lie down

>>> TO CHOOSE WHAT THE DOG TRAINS ON, EDIT `TASKS` BELOW (weight 0 = never trained). <<<
"""
import os
import numpy as np
import mujoco
import gymnasium as gym
from gymnasium import spaces

HERE = os.path.dirname(os.path.abspath(__file__))
LEGS = ['front_left', 'front_right', 'back_left', 'back_right']
PARTS = ('hip', 'upper_leg', 'lower_leg')

# --------------------------------------------------------------------------------------
# 1) WHAT THE DOG TRAINS ON.   name: (weight, mode, vx, vy, wz)      speeds = maximum value
#    weight = how often this task is picked (0 = disabled). Edit freely.
# --------------------------------------------------------------------------------------
TASKS = {
    "stand":      (2.5, 0,  0.00, 0.0,  0.0),
    "forward":    (3.0, 0,  0.25, 0.0,  0.0),
    "backward":   (2.0, 0, -0.15, 0.0,  0.0),
    "turn_left":  (3.0, 0,  0.00, 0.0,  0.8),
    "turn_right": (3.0, 0,  0.00, 0.0, -0.8),
    "fwd_left":   (1.0, 0,  0.20, 0.0,  0.5),
    "fwd_right":  (1.0, 0,  0.20, 0.0, -0.5),
    "strafe_left":  (1.5, 0, 0.00,  0.12, 0.0),    # v6: enabled (it was weight 0 -> the dog never learned to go sideways)
    "strafe_right": (1.5, 0, 0.00, -0.12, 0.0),
    "sit":        (0.0, 1,  0.00, 0.0,  0.0),   # EXPERIMENTAL: disabled, in my tests the dog tips backwards when it folds its hind legs
    "lie_down":   (1.0, 2,  0.00, 0.0,  0.0),
}

# 2) reward weights (positive = reward, negative = penalty)
REWARD = dict(
    track_lin=4.0, track_yaw=2.0, alive=0.2,   # v6: track_yaw 0.5 -> 2.0
    pose=1.5,                      # sit / lie: match the reference pose
    lin_vel_z=-2.0, ang_vel_xy=-0.05, orientation=-2.0, height=-20.0,
    torque=-0.002, joint_acc=-2e-7,
    still_pose=-1.0, still_joint_vel=-0.05, still_feet_down=-1.0, still_action=-0.1,   # when told to stand still
    # --- anti-skating / anti-vibration -------------------------------------------------
    gait_contact=1.0,              # feet must touch/leave the ground in a diagonal trot pattern (moving only)
    swing_height=-100.0,           # a foot that is in the air must be lifted ~3.5 cm (moving only)
    foot_slip=-2.0,                # penalty for a foot sliding on the ground while it is in contact
    action_rate=-0.05, action_accel=-0.02, joint_vel=-0.0005,   # v6: less smoothing penalty -> bigger steps   # smooth, non-vibrating leg motion
)
GAIT_PERIOD = 0.4             # s per step cycle (2.5 Hz)  - the dog gets a clock (sin, cos) in its observation
STANCE_DUTY = 0.6             # fraction of the cycle a foot is on the ground
SWING_HEIGHT = 0.035          # m
FOOT_RADIUS = 0.02
YAW_SIGMA = 0.25              # v6: was 0.1 -> for a 0.8 rad/s command the reward was exp(-6.4)=0.002 at wz=0: no gradient, the dog never learned to turn
RANDOM_CMD_P = 0.4            # v6: probability of a continuous random command (vx, vy, wz mixed) instead of a fixed task
ACTION_FILTER = 0.6           # low-pass on the actions (1 = none): removes high-frequency buzzing
PHASE_OFFSET = np.array([0.0, 0.5, 0.5, 0.0])   # LEGS order: FL, FR, BL, BR  -> diagonal pairs move together (trot)

# reference poses per leg (hip, upper, lower); order of legs = LEGS. Found by settling tests in MuJoCo.
STAND = np.array([0, -0.68162889, 1.26606748] * 4)
SIT = np.array([[0, -0.2, 0.5]] * 2 + [[0, -1.57, 1.57]] * 2).ravel()
LIE = np.array([[0.9, -1.0, 1.55], [-0.9, -1.0, 1.55], [0.9, -1.0, 1.55], [-0.9, -1.0, 1.55]]).ravel()
POSES = [STAND, SIT, LIE]
LEGACY_GYRO_FRAME = False     # v1-v3 rotated the (already body-frame) angular velocity by R.T by mistake.
                              # Set True ONLY to run a model that was trained with v1-v3.
# Robustness against the sim-to-sim gap (Gazebo vs MuJoCo): randomised physics, sensor noise, 1-step latency.
RANDOMIZE = dict(kp=(0.75, 1.25), kv=(0.5, 1.5), friction=(0.6, 1.4), mass=(0.85, 1.2), latency_p=0.5,
                 noise_gravity=0.03, noise_gyro=0.3, noise_q=0.01, noise_qd=0.8)
TARGET_HEIGHT = 0.195
ACTION_SCALE = 0.5            # rad around the reference pose
CTRL_DT = 0.02                # 50 Hz policy
REF_RATE = 0.5               # rad/s: how fast the reference pose moves between stand/sit/lie
COMMAND_PERIOD = 4.0          # s: a new command every 4 s
EPISODE_S = 20.0


class DogEnv(gym.Env):
    metadata = {"render_modes": ["rgb_array"]}

    def __init__(self, xml=os.path.join(HERE, "dog.xml"), tasks=None, seed=None, push=True, randomize=True):
        self.m = mujoco.MjModel.from_xml_path(xml)
        self.d = mujoco.MjData(self.m)
        m = self.m
        self.tasks = {k: v for k, v in (tasks or TASKS).items() if v[0] > 0}
        self.task_names = list(self.tasks)
        w = np.array([self.tasks[k][0] for k in self.task_names]); self.task_p = w / w.sum()
        self.push = push
        self.qadr = np.array([m.joint(f"motor_{l}_{p}").qposadr[0] for l in LEGS for p in PARTS])
        self.vadr = np.array([m.joint(f"motor_{l}_{p}").dofadr[0] for l in LEGS for p in PARTS])
        self.aid = np.array([m.actuator(f"motor_{l}_{p}").id for l in LEGS for p in PARTS])
        self.base = m.body("base_link").id
        self.foot_geom = [m.geom(f"{l}_foot_geom").id for l in LEGS]
        self.floor = m.geom("floor").id
        self.lo = m.actuator_ctrlrange[self.aid, 0]; self.hi = m.actuator_ctrlrange[self.aid, 1]
        self.n_sub = int(round(CTRL_DT / m.opt.timestep))
        self.max_steps = int(EPISODE_S / CTRL_DT); self.cmd_steps = int(COMMAND_PERIOD / CTRL_DT)
        self.rng = np.random.default_rng(seed)
        self.push = push; self.randomize = randomize
        self.kp0 = m.actuator_gainprm[self.aid, 0].copy(); self.kv0 = -m.actuator_biasprm[self.aid, 2].copy()
        self.fric0_foot = m.geom_friction[self.foot_geom, 0].copy(); self.fric0_floor = m.geom_friction[self.floor, 0]
        self.mass0 = m.body_mass.copy(); self.inertia0 = m.body_inertia.copy(); self.delay = 0   # (the whole robot is scaled)
        self.observation_space = spaces.Box(-np.inf, np.inf, (50,), np.float32)
        self.action_space = spaces.Box(-1.0, 1.0, (12,), np.float32)
        self.mode = 0; self.cmd = np.zeros(3); self.fixed_command = None
        self._vel = np.zeros(6); self.phase = 0.0

    # ---------------------------------------------------------------- helpers
    def set_command(self, mode, vx=0.0, vy=0.0, wz=0.0):
        """Used when running a trained policy: force a command (stops random resampling)."""
        self.fixed_command = (mode, vx, vy, wz); self.mode = mode; self.cmd = np.array([vx, vy, wz], float)

    def _sample_command(self):
        if self.fixed_command is not None:
            return
        if self.rng.random() < RANDOM_CMD_P:       # v6: any mix inside the ranges of dog_controller.py (CMD_MIN / CMD_MAX)
            self.mode = 0
            self.cmd = np.array([self.rng.uniform(-0.15, 0.25), self.rng.uniform(-0.12, 0.12), self.rng.uniform(-0.8, 0.8)])
            if self.rng.random() < 0.3: self.cmd[1] = 0.0
            return
        name = self.rng.choice(self.task_names, p=self.task_p)
        _, mode, vx, vy, wz = self.tasks[name]
        s = self.rng.uniform(0.5, 1.0)
        self.mode = mode; self.cmd = np.array([vx, vy, wz]) * s

    def _randomize_physics(self):
        m, R, rng = self.m, RANDOMIZE, self.rng
        if not self.randomize:
            kp, kv, fr, ms, self.delay = 1.0, 1.0, 1.0, 1.0, 0
            ff = 1.0
        else:
            kp = rng.uniform(*R["kp"]); kv = rng.uniform(*R["kv"]); fr = rng.uniform(*R["friction"])
            ff = rng.uniform(*R["friction"]); ms = rng.uniform(*R["mass"])
            self.delay = int(rng.random() < R["latency_p"])          # 0 or 1 control step (20 ms) of latency
        m.actuator_gainprm[self.aid, 0] = self.kp0 * kp; m.actuator_biasprm[self.aid, 1] = -self.kp0 * kp
        m.actuator_biasprm[self.aid, 2] = -self.kv0 * kv
        m.geom_friction[self.foot_geom, 0] = self.fric0_foot * fr; m.geom_friction[self.floor, 0] = self.fric0_floor * ff
        m.body_mass[:] = self.mass0 * ms; m.body_inertia[:] = self.inertia0 * ms

    def _moving(self):
        """True when the current command asks the dog to walk or turn (False = stand / sit / lie)."""
        return self.mode == 0 and (np.linalg.norm(self.cmd[:2]) > 0.02 or abs(self.cmd[2]) > 0.05)

    def _R(self):
        return self.d.xmat[self.base].reshape(3, 3)

    def _contacts(self):
        c = np.zeros(4, bool)
        for i in range(self.d.ncon):
            con = self.d.contact[i]
            for k, g in enumerate(self.foot_geom):
                if (con.geom1 == g and con.geom2 == self.floor) or (con.geom2 == g and con.geom1 == self.floor):
                    c[k] = True
        return c

    def _obs(self):
        d = self.d; R = self._R()
        gravity = R.T @ np.array([0, 0, -1.0])
        gyro = (R.T @ d.qvel[3:6]) if LEGACY_GYRO_FRAME else d.qvel[3:6].copy()   # qvel[3:6] is body-frame already
        q = d.qpos[self.qadr]; qd = d.qvel[self.vadr]
        if self.randomize:
            N = RANDOMIZE; rg = self.rng
            gravity = gravity + rg.normal(0, N["noise_gravity"], 3); gyro = gyro + rg.normal(0, N["noise_gyro"], 3)
            q = q + rg.normal(0, N["noise_q"], 12); qd = qd + rg.normal(0, N["noise_qd"], 12)
        onehot = np.eye(3)[self.mode]
        o = np.concatenate([gravity, gyro * 0.25, q - STAND, qd * 0.05, self.last_action,
                            self.cmd * np.array([2.0, 2.0, 1.0]), onehot,
                            [np.sin(2 * np.pi * self.phase), np.cos(2 * np.pi * self.phase)] if self._moving() else [0.0, 0.0]])
        return o.astype(np.float32)

    # ---------------------------------------------------------------- gym API
    def reset(self, seed=None, options=None):
        if seed is not None:
            self.rng = np.random.default_rng(seed)
        m, d = self.m, self.d
        mujoco.mj_resetData(m, d)
        self._randomize_physics()
        self._sample_command()
        ref = POSES[0]
        d.qpos[self.qadr] = ref + self.rng.uniform(-0.05, 0.05, 12); d.ctrl[self.aid] = ref
        yaw = self.rng.uniform(-np.pi, np.pi)
        d.qpos[3:7] = [np.cos(yaw / 2), 0, 0, np.sin(yaw / 2)]
        d.qpos[2] = 0.3; mujoco.mj_forward(m, d)
        feet = [d.xpos[m.body(f"{l}_foot").id][2] for l in LEGS]
        d.qpos[2] = 0.3 - min(feet) + 0.021; mujoco.mj_forward(m, d)
        self.steps = 0; self.phase = self.rng.random(); self.a_f = np.zeros(12); self.last_action2 = np.zeros(12); self.last_action = np.zeros(12); self.prev_qd = d.qvel[self.vadr].copy()
        self.air = np.zeros(4); self.last_contact = np.ones(4, bool); self.ref = ref.copy()
        return self._obs(), {}

    def step(self, action):
        a = np.clip(np.asarray(action, float), -1, 1)
        a_eff = self.last_action if self.delay else a                  # optional 1-step latency (training only)
        self.a_f = self.a_f + ACTION_FILTER * (a_eff - self.a_f)      # low-pass filtered action (no buzzing)
        goal = POSES[self.mode]                       # reference pose glides to the new mode
        self.ref = self.ref + np.clip(goal - self.ref, -REF_RATE * CTRL_DT, REF_RATE * CTRL_DT)
        target = np.clip(self.ref + ACTION_SCALE * self.a_f, self.lo, self.hi)
        self.d.ctrl[self.aid] = target
        if self.push and self.steps % 150 == 149 and self.rng.random() < 0.5:      # small random shove
            self.d.qvel[0:2] += self.rng.uniform(-0.15, 0.15, 2)
        mujoco.mj_step(self.m, self.d, nstep=self.n_sub)
        self.steps += 1
        self.phase = (self.phase + CTRL_DT / GAIT_PERIOD) % 1.0
        d = self.d; R = self._R()
        v_body = R.T @ d.qvel[0:3]; w_body = (R.T @ d.qvel[3:6]) if LEGACY_GYRO_FRAME else d.qvel[3:6].copy()
        gravity = R.T @ np.array([0, 0, -1.0])
        q = d.qpos[self.qadr]; qd = d.qvel[self.vadr]
        tau = d.actuator_force[self.aid]
        contact = self._contacts()

        # foot height above the floor and speed of the contact point (for the anti-skating terms)
        foot_h = d.geom_xpos[self.foot_geom][:, 2] - FOOT_RADIUS
        foot_v = np.zeros(4)
        for k, g in enumerate(self.foot_geom):
            mujoco.mj_objectVelocity(self.m, d, mujoco.mjtObj.mjOBJ_GEOM, g, self._vel, 0)
            vc = self._vel[3:6] + np.cross(self._vel[0:3], [0.0, 0.0, -FOOT_RADIUS])
            foot_v[k] = np.linalg.norm(vc[:2])

        r = 0.0; W = REWARD; terms = {}
        moving = self._moving()
        if self.mode == 0:
            lin_err = np.sum((self.cmd[:2] - v_body[:2]) ** 2)
            yaw_err = (self.cmd[2] - w_body[2]) ** 2
            terms["track_lin"] = W["track_lin"] * np.exp(-lin_err / 0.03)
            terms["track_yaw"] = W["track_yaw"] * np.exp(-yaw_err / YAW_SIGMA)
            terms["height"] = W["height"] * (d.qpos[2] - TARGET_HEIGHT) ** 2
            terms["orientation"] = W["orientation"] * np.sum(gravity[:2] ** 2)
            if not moving:      # STAND STILL: no stepping, no shuffling, all four feet down, joints quiet
                terms["still_pose"] = W["still_pose"] * np.sum((q - STAND) ** 2)
                terms["still_joint_vel"] = W["still_joint_vel"] * np.sum(qd ** 2)
                terms["still_feet_down"] = W["still_feet_down"] * float(np.sum(~contact))
                terms["still_action"] = W["still_action"] * np.sum(a ** 2)
            else:   # a REAL step pattern: diagonal pairs alternate, swing feet are lifted
                stance_des = ((self.phase + PHASE_OFFSET) % 1.0) < STANCE_DUTY
                terms["gait_contact"] = W["gait_contact"] * np.mean(contact == stance_des)
                terms["swing_height"] = W["swing_height"] * np.sum((~stance_des) * np.minimum(foot_h - SWING_HEIGHT, 0.0) ** 2)
        else:
            terms["pose"] = W["pose"] * np.exp(-np.sum((q - POSES[self.mode]) ** 2) / 0.5)
            terms["track_lin"] = W["track_lin"] * np.exp(-np.sum(v_body[:2] ** 2) / 0.01)
        terms["foot_slip"] = W["foot_slip"] * np.sum(foot_v * contact)
        terms["alive"] = W["alive"]
        terms["lin_vel_z"] = W["lin_vel_z"] * v_body[2] ** 2
        terms["ang_vel_xy"] = W["ang_vel_xy"] * np.sum(w_body[:2] ** 2)
        terms["torque"] = W["torque"] * np.sum(tau ** 2)
        terms["action_rate"] = W["action_rate"] * np.sum((a - self.last_action) ** 2)
        terms["action_accel"] = W["action_accel"] * np.sum((a - 2 * self.last_action + self.last_action2) ** 2)
        terms["joint_vel"] = W["joint_vel"] * np.sum(qd ** 2)
        terms["joint_acc"] = W["joint_acc"] * np.sum(((qd - self.prev_qd) / CTRL_DT) ** 2)
        r = float(sum(terms.values()))
        self.last_action2 = self.last_action; self.last_action = a; self.prev_qd = qd.copy()

        up = R[2, 2]
        fell = (up < 0.5 and self.mode != 1) or (up < 0.3) or d.qpos[2] < 0.03 or (self.mode == 0 and d.qpos[2] < 0.1)
        truncated = self.steps >= self.max_steps
        terminated = bool(fell)
        if terminated:
            r -= 5.0
        if self.steps % self.cmd_steps == 0 and not truncated:
            self._sample_command()
        info = {"vx": v_body[0], "vy": v_body[1], "wz": w_body[2], "mode": self.mode, "terms": terms,
                "slip": float(np.sum(foot_v * contact)), "contact": contact.copy()}
        return self._obs(), r, terminated, truncated, info
