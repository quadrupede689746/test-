"""
Train the dog with PPO (Stable-Baselines3).
  python train.py                       # new run, name dog_v1, 5M steps, 8 parallel envs
  python train.py --name myrun --steps 10000000 --envs 12
  python train.py --name myrun --resume models/dog_v1/final.zip   # continue training an existing model
What the dog learns = the TASKS table at the top of dog_env.py (edit it, then re-run).
Results are written to models/<name>/ : checkpoints, final.zip, vecnormalize.pkl, policy.npz
"""
import argparse, os
import torch
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import CheckpointCallback
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv, VecNormalize
from dog_env import DogEnv
from export import export_npz

def make(seed):
    return lambda: Monitor(DogEnv(seed=seed))      # Monitor -> logs ep_rew_mean / ep_len_mean

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--name", default="dog_v1"); ap.add_argument("--steps", type=int, default=5_000_000)
    ap.add_argument("--envs", type=int, default=8); ap.add_argument("--resume", default=None)
    a = ap.parse_args()
    out = os.path.join("models", a.name); os.makedirs(out, exist_ok=True)
    torch.set_num_threads(1)
    venv = (SubprocVecEnv if a.envs > 1 else DummyVecEnv)([make(i) for i in range(a.envs)])
    if a.resume:
        vn = os.path.join(os.path.dirname(a.resume), "vecnormalize.pkl")
        venv = VecNormalize.load(vn, venv); venv.training = True
        model = PPO.load(a.resume, env=venv, device="cpu")
    else:
        venv = VecNormalize(venv, norm_obs=True, norm_reward=True, clip_obs=10.0, gamma=0.99)
        n_steps = 256
        model = PPO("MlpPolicy", venv, n_steps=n_steps, batch_size=a.envs * n_steps // 4, n_epochs=5,
                    learning_rate=3e-4, gamma=0.99, gae_lambda=0.95, clip_range=0.2, ent_coef=0.0,
                    policy_kwargs=dict(net_arch=dict(pi=[128, 128], vf=[128, 128]), activation_fn=torch.nn.Tanh,
                                       log_std_init=-1.5),   # small exploration noise at the start (std ~0.22)
                    device="cpu", verbose=1)
    ckpt = CheckpointCallback(save_freq=max(500_000 // a.envs, 1), save_path=out, name_prefix="ckpt", save_vecnormalize=True)
    try:
        model.learn(total_timesteps=a.steps, callback=ckpt, reset_num_timesteps=not a.resume)
    except KeyboardInterrupt:
        print("Interrupted - saving what we have")
    model.save(os.path.join(out, "final")); venv.save(os.path.join(out, "vecnormalize.pkl"))
    export_npz(os.path.join(out, "final.zip"))
    print("DONE. Try it:  python run_policy.py", os.path.join(out, "policy.npz"))
