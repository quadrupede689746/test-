"""Checks that DogController reproduces the training environment EXACTLY (observations and joint targets)."""
import sys, numpy as np
import dog_env as D
from dog_env import DogEnv
from dog_controller import DogController
path = sys.argv[1]
for legacy in (False, True):
    D.LEGACY_GYRO_FRAME = legacy
    env = DogEnv(seed=5, push=False, randomize=False); env.set_command(0, 0.2, 0.0, 0.3); o, _ = env.reset()
    c = DogController(path, legacy_gyro=legacy); c.set_command(0.2, 0.0, 0.3); c.engaged = True
    c.ref = D.POSES[0].copy(); c.phase = env.phase
    d = env.d; worst_obs = worst_tgt = 0.0
    for i in range(400):
        q = d.qpos[env.qadr].copy(); qd = d.qvel[env.vadr].copy(); quat = d.qpos[3:7].copy(); gyro = d.qvel[3:6].copy()
        obs_c = c.observe(q, qd, quat, gyro)
        worst_obs = max(worst_obs, float(np.abs(obs_c - o).max()))
        tgt = c.update(q, qd, quat, gyro)
        o, r, te, tr, info = env.step(c.last_action)     # env applies the same action the controller chose
        worst_tgt = max(worst_tgt, float(np.abs(d.ctrl[env.aid] - tgt).max()))
        if te: break
    print(f"legacy_gyro={legacy}: steps={i+1}  max |obs difference| = {worst_obs:.2e}   max |target difference| = {worst_tgt:.2e}   x travelled = {d.qpos[0]:.2f} m")
