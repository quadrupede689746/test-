"""Ouvre le chien dans le viewer MuJoCo et le fait trotter.  Usage:  python view.py"""
import time, mujoco, mujoco.viewer, numpy as np
import trot
m, d = trot.m, trot.d
STEP, PERIOD, LIFT, DUTY = 0.05, 0.4, 0.035, 0.6   # <- parametres a modifier
mujoco.mj_resetData(m, d)
for l in trot.legs:
    d.qpos[trot.qadr[l]] = trot.BENT; d.ctrl[trot.aid[l]] = trot.BENT
d.qpos[2] = trot.base[2]; d.qpos[3] = 1; mujoco.mj_forward(m, d)
off = {'front_left':0,'back_right':0,'front_right':.5,'back_left':.5}
qprev = {l: trot.BENT.copy() for l in trot.legs}
with mujoco.viewer.launch_passive(m, d) as v:
    i = 0
    while v.is_running():
        t0 = time.time(); t = d.time
        if i % 10 == 0:
            ramp = min(1, t)
            for l in trot.legs:
                ph = (t/PERIOD + off[l]) % 1
                if ph < DUTY: u = ph/DUTY; dx, dz = STEP*ramp*(.5-u), 0
                else: u = (ph-DUTY)/(1-DUTY); dx, dz = STEP*ramp*(-.5+u), LIFT*ramp*np.sin(np.pi*u)
                q = trot.ik(l, trot.nom[l]+np.array([dx,0,dz]), qprev[l]); qprev[l] = q
                d.ctrl[trot.aid[l]] = q
        mujoco.mj_step(m, d); i += 1
        if i % 16 == 0: v.sync()
        time.sleep(max(0, m.opt.timestep - (time.time()-t0)))
