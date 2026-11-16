"""
Run a trained policy.
  python run_policy.py models/dog_v1/policy.npz            # window + type commands in THIS terminal
  python run_policy.py models/dog_v1/final.zip             # also works (needs stable-baselines3)
  python run_policy.py models/dog_v1/policy.npz --eval     # no window: measures how well each task is followed
  python run_policy.py models/dog_v1/policy.npz --demo     # window, automatic sequence of commands
  add --legacy-gyro for models trained with v1-v3 (before the gyro fix)

Commands typed in the terminal (then Enter):
  f forward | b backward | l turn left | r turn right | fl forward+left | fr forward+right
  sl strafe left | sr strafe right | s stand/stop | lie lie down | sit sit (experimental)
  or numbers:  v 0.2 0 0.3   (vx m/s, vy m/s, wz rad/s)        q quit
"""
import sys, time, threading
import numpy as np
import mujoco
from dog_env import DogEnv, TASKS, CTRL_DT
from export import load_policy

PRESETS = {"f": (0, .22, 0, 0), "b": (0, -.13, 0, 0), "l": (0, 0, 0, .7), "r": (0, 0, 0, -.7),
           "fl": (0, .18, 0, .45), "fr": (0, .18, 0, -.45), "sl": (0, 0, .1, 0), "sr": (0, 0, -.1, 0),
           "s": (0, 0, 0, 0), "lie": (2, 0, 0, 0), "sit": (1, 0, 0, 0)}

def evaluate(policy, seconds=10.0):
    print(f"{'task':14s} {'cmd (vx,vy,wz)':22s} {'measured (vx,vy,wz)':24s} {'foot slip':9s} {'feet lifted':11s} {'leg motion':10s} fell")
    for name, (w, mode, vx, vy, wz) in TASKS.items():
        if w <= 0:
            continue
        env = DogEnv(seed=0, push=False, randomize=False); env.set_command(mode, vx, vy, wz); o, _ = env.reset()
        m = []; sl = []; air = []; lm = []; fell = False
        for i in range(int(seconds / CTRL_DT)):
            o, r, te, tr, info = env.step(policy(o))
            if i > 150: m.append([info["vx"], info["vy"], info["wz"]]); sl.append(info["slip"]); air.append(float((~info["contact"]).any())); lm.append(float(np.abs(env.d.qvel[env.vadr]).mean()))
            if te: fell = True; break
        mm = np.mean(m, 0) if m else np.zeros(3)
        print(f"{name:14s} ({vx:5.2f},{vy:5.2f},{wz:5.2f})      ({mm[0]:5.2f},{mm[1]:5.2f},{mm[2]:5.2f})   {np.mean(sl) if sl else 0:6.3f}    {100*np.mean(air) if air else 0:5.0f}%       {np.mean(lm) if lm else 0:6.2f}     {'YES' if fell else 'no'}")
    print("foot slip = speed of feet sliding on the floor (m/s, ~0 = no skating). feet lifted = % of time a foot is in the air (walking: high, standing: ~0).  leg motion = mean joint speed rad/s (standing: ~0).")

def run_viewer(policy, demo=False):
    import mujoco.viewer
    env = DogEnv(seed=0, push=False, randomize=False); env.set_command(*PRESETS["s"]); o, _ = env.reset()
    cmd = {"v": PRESETS["s"], "quit": False}
    def reader():
        for line in sys.stdin:
            t = line.split()
            if not t: continue
            if t[0] == "q": cmd["quit"] = True; return
            if t[0] in PRESETS: cmd["v"] = PRESETS[t[0]]
            elif t[0] == "v" and len(t) == 4: cmd["v"] = (0, *map(float, t[1:]))
            else: print("unknown command")
    if not demo:
        threading.Thread(target=reader, daemon=True).start(); print(__doc__.split("Commands")[1])
    seq = [("s", 3), ("f", 6), ("l", 5), ("r", 5), ("b", 5), ("fl", 6), ("fr", 6), ("lie", 5), ("s", 5)]
    with mujoco.viewer.launch_passive(env.m, env.d) as v:
        v.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING; v.cam.trackbodyid = env.base
        v.cam.distance = 1.2; v.cam.elevation = -20
        t0 = time.time()
        while v.is_running() and not cmd["quit"]:
            t = time.time()
            if demo:
                T = sum(s for _, s in seq); x = (t - t0) % T; acc = 0
                for k, s in seq:
                    acc += s
                    if x < acc: cmd["v"] = PRESETS[k]; break
            if env.fixed_command != cmd["v"]:
                env.set_command(*cmd["v"])
            o, r, te, tr, info = env.step(policy(o))
            if te or tr: o, _ = env.reset()
            v.sync()
            time.sleep(max(0, CTRL_DT - (time.time() - t)))

if __name__ == "__main__":
    path = [a for a in sys.argv[1:] if not a.startswith("--")]
    if not path: sys.exit(__doc__)
    if "--legacy-gyro" in sys.argv:          # model trained with v1-v3 (see README_GAZEBO.md)
        import dog_env; dog_env.LEGACY_GYRO_FRAME = True
    pol = load_policy(path[0])
    evaluate(pol) if "--eval" in sys.argv else run_viewer(pol, demo="--demo" in sys.argv)
