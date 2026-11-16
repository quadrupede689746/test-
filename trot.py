import mujoco, numpy as np, sys
m=mujoco.MjModel.from_xml_path('dog.xml'); d=mujoco.MjData(m)
legs=['front_left','front_right','back_left','back_right']
P=('hip','upper_leg','lower_leg')
BENT=np.array([0,-0.68162889,1.26606748])
qadr={l:[m.joint(f'motor_{l}_{p}').qposadr[0] for p in P] for l in legs}
vadr={l:[m.joint(f'motor_{l}_{p}').dofadr[0] for p in P] for l in legs}
aid={l:[m.actuator(f'motor_{l}_{p}').id for p in P] for l in legs}
fb={l:m.body(f'{l}_foot').id for l in legs}
lo=np.array([-1.04,-1.5708,-2.9]); hi=np.array([1.04,2.59,1.5708])

# initial pose
for l in legs: d.qpos[qadr[l]]=BENT; d.ctrl[aid[l]]=BENT
mujoco.mj_forward(m,d)
z0=min(d.xpos[fb[l]][2] for l in legs); d.qpos[2]=0.25-z0+0.022
mujoco.mj_forward(m,d)
base=d.xpos[m.body('base_link').id].copy()
# nominal foot positions in the base frame
def foot_in_base(l):
    R=d.xmat[m.body('base_link').id].reshape(3,3); return R.T@(d.xpos[fb[l]]-d.xpos[m.body('base_link').id])
nom={l:foot_in_base(l) for l in legs}

dd=mujoco.MjData(m)   # scratch data for IK
def ik(l,target_base,q0,iters=40):
    """damped least squares on the 3 joints of leg l, target in base frame (base kept at identity)"""
    q=q0.copy(); dd.qpos[:]=0; dd.qpos[3]=1
    jp=np.zeros((3,m.nv))
    for _ in range(iters):
        dd.qpos[qadr[l]]=q; mujoco.mj_kinematics(m,dd); mujoco.mj_comPos(m,dd)
        pos=dd.xpos[fb[l]]-dd.xpos[m.body('base_link').id]
        err=target_base-pos
        if np.linalg.norm(err)<1e-4: break
        mujoco.mj_jacBody(m,dd,jp,None,fb[l])
        J=jp[:,vadr[l]]
        q+=J.T@np.linalg.solve(J@J.T+1e-4*np.eye(3),err)
        q=np.clip(q,lo,hi)
    return q

def run(step=0.05,lift=0.035,period=0.5,T=8.0,duty=0.6,vid=None):
    # reset
    mujoco.mj_resetData(m,d)
    for l in legs: d.qpos[qadr[l]]=BENT; d.ctrl[aid[l]]=BENT
    d.qpos[2]=base[2]; d.qpos[3]=1; mujoco.mj_forward(m,d)
    phase_off={'front_left':0.0,'back_right':0.0,'front_right':0.5,'back_left':0.5}  # trot diagonal pairs
    qprev={l:BENT.copy() for l in legs}
    x0=d.qpos[0]; n=int(T/m.opt.timestep); ctrl_dt=0.01
    fall=None; maxtau=0; frames=[]
    if vid: r=mujoco.Renderer(m,480,640); cam=mujoco.MjvCamera(); cam.distance=0.9;cam.elevation=-15;cam.azimuth=140
    for i in range(n):
        t=i*m.opt.timestep
        if i%int(ctrl_dt/m.opt.timestep)==0:
            ramp=min(1,t/1.0)
            for l in legs:
                ph=((t/period)+phase_off[l])%1.0
                s=step*ramp
                if ph<duty:   # stance: foot moves backward relative to body
                    u=ph/duty; dx=s/2-s*u; dz=0
                else:         # swing: foot moves forward with a lift
                    u=(ph-duty)/(1-duty); dx=-s/2+s*u; dz=lift*ramp*np.sin(np.pi*u)
                tgt=nom[l]+np.array([dx,0,dz])
                q=ik(l,tgt,qprev[l]); qprev[l]=q; d.ctrl[aid[l]]=q
        mujoco.mj_step(m,d)
        maxtau=max(maxtau,np.abs(d.actuator_force).max())
        up=d.xmat[m.body('base_link').id].reshape(3,3)[2,2]
        if d.qpos[2]<0.08 or up<0.5: fall=t; break
        if vid and i%33==0:
            cam.lookat[:]=d.qpos[:3]; r.update_scene(d,cam); frames.append(r.render().copy())
    dist=d.qpos[0]-x0
    return dict(fall=fall,dist=dist,y=d.qpos[1],z=d.qpos[2],up=d.xmat[m.body('base_link').id].reshape(3,3)[2,2],maxtau=maxtau,frames=frames)

if __name__=='__main__':
    for step in (0.03,0.05,0.08):
        for period in (0.4,0.6):
            r=run(step=step,period=period,T=8)
            print(f"step={step:.2f} period={period:.1f} -> fall={r['fall']} dist={r['dist']:.3f} m  y={r['y']:.3f} z={r['z']:.3f} up={r['up']:.2f} maxtau={r['maxtau']:.2f}")
