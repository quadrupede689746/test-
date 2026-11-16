# Robodog - train a command-following dog (MuJoCo + PPO), v2

## 1. Install (Linux, once)
    python3 -m venv ~/dogenv && source ~/dogenv/bin/activate
    pip install torch --index-url https://download.pytorch.org/whl/cpu
    pip install mujoco numpy gymnasium stable-baselines3
(later, in a new terminal:  source ~/dogenv/bin/activate)

## 2. Choose WHAT the dog trains on
Edit the table `TASKS` at the top of `dog_env.py`:  "forward": (weight, mode, vx, vy, wz)
 weight 0 = never trained. vx forward/back m/s, vy sideways m/s, wz turn rad/s (+ = left).
 mode 0 walk/stand, 1 sit (experimental, off), 2 lie down. Floor = infinite flat plane.

## 3. Train
    python train.py --name dog_v1 --steps 5000000 --envs 8
 Ctrl+C saves. Output in models/<name>/ : final.zip, vecnormalize.pkl, policy.npz, ckpt_*.zip
 Watch ep_len_mean (toward 1000 = not falling).

## 4. Check and watch
    python run_policy.py models/dog_v1/policy.npz --eval    # speeds, foot slip, feet lifted, falls
    python run_policy.py models/dog_v1/policy.npz           # window; type f b l r s lie + Enter in the terminal
    python run_policy.py models/dog_v1/policy.npz --demo    # automatic sequence
 Works with policy.npz or final.zip (any existing model of THIS version).

## 5. Export / continue
 policy.npz = the exported model (numpy only). Continue training:
    python train.py --name dog_v2 --resume models/dog_v1/final.zip --steps 5000000

## 6. Against "skating" / leg vibration (v2 reward)
 The dog gets a gait clock (sin/cos, 2.5 Hz) and is rewarded for a real trot: diagonal feet touch and leave the
 ground in turn, swinging feet are lifted ~3.5 cm. Sliding feet, jerky/vibrating leg commands and fast joint
 motion are penalised, and actions are low-pass filtered. Tune: REWARD, GAIT_PERIOD, STANCE_DUTY, SWING_HEIGHT.
 --eval prints "foot slip" (should be ~0) and "feet lifted" (should be high when walking, ~0 standing).
 NOTE: models trained with v1 (48 inputs) do not work with v2 (50 inputs) - train from scratch.

## 7. Add a new gesture
 Add a reference pose to POSES (test that it stands still in MuJoCo first), a row in TASKS with a new mode,
 grow the mode one-hot in the observation, and give it a reward in DogEnv.step.

## 8. Standing still (v3)
 The gait clock is only given to the dog while a walk/turn command is active, and a stand command is rewarded for
 quiet legs (no joint motion, all four feet down). `--eval` shows "leg motion" and "feet lifted": both ~0 for `stand`.
 Models from v2 can be continued with --resume (same 50 inputs).

## 9. v4: gyro bug fix + Gazebo
 Fixed: the body rotation rate was rotated twice in v1-v3 (LEGACY_GYRO_FRAME in dog_env.py). New training uses the
 correct frame. To run an older model in MuJoCo add --legacy-gyro. Gazebo: see README_GAZEBO.md.

## 10. v5: robustness against shaking in Gazebo
 Training now randomises (each episode): motor gains (kp 0.75-1.25x, kv 0.5-1.5x), friction (0.6-1.4x), total mass
 (0.85-1.2x), 1-step action latency (50% of episodes) and adds sensor noise (RANDOMIZE in dog_env.py).
 --eval / run_policy.py / verify_controller.py use the clean (noise-free) environment.
 Continue an older model (v3 or v4) with the new environment:
    python3 train.py --name v5 --resume /path/to/models/v3/final.zip --steps 5000000 --envs 8
 (vecnormalize.pkl must be in the same folder as final.zip). Then in Gazebo use  -p legacy_gyro:=false
 (the v5 environment uses the corrected gyro frame, even when the weights started from a v3 model).
