import mujoco, numpy as np
m=mujoco.MjModel.from_xml_path('dog.xml'); d=mujoco.MjData(m)
BENT=np.array([0,-0.68162889,1.26606748]*4)   # pose from the Gazebo tests (per leg: hip, upper, lower)
def jid(n): return m.joint(n).qposadr[0]
legs=['front_left','front_right','back_left','back_right']
names=[f'motor_{l}_{p}' for l in legs for p in ('hip','upper_leg','lower_leg')]
for n,v in zip(names,BENT): d.qpos[jid(n)]=v
d.ctrl[:]=BENT
mujoco.mj_forward(m,d)
foot_z=[d.xpos[m.body(f'{l}_foot').id][2] for l in legs]
print('foot z (base at 0.25):',np.round(foot_z,3))
d.qpos[2]=0.25-min(foot_z)+0.02+0.002   # sphere radius 0.02
mujoco.mj_forward(m,d)
print('base height',d.qpos[2])
for i in range(3000): mujoco.mj_step(m,d)
q=d.qpos[3:7]
print('after 3 s: z=%.3f  quat=%s  xy=%s'%(d.qpos[2],np.round(q,3),np.round(d.qpos[:2],3)))
print('max |torque| =',np.abs(d.actuator_force).max().round(2),'Nm  (limit 4)')
