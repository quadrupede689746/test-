"""Turn a trained Stable-Baselines3 PPO model into a small portable file (policy.npz, numpy only)."""
import os, pickle, sys
import numpy as np

def export_npz(zip_path, out_path=None, vecnorm_path=None):
    import torch
    from stable_baselines3 import PPO
    model = PPO.load(zip_path, device="cpu")
    folder = os.path.dirname(os.path.abspath(zip_path))
    vecnorm_path = vecnorm_path or os.path.join(folder, "vecnormalize.pkl")
    out_path = out_path or os.path.join(folder, "policy.npz")
    pol = model.policy
    lins = [m for m in pol.mlp_extractor.policy_net if isinstance(m, torch.nn.Linear)]
    arrs = {}
    for i, l in enumerate(lins):
        arrs[f"W{i}"] = l.weight.detach().numpy(); arrs[f"b{i}"] = l.bias.detach().numpy()
    arrs["Wa"] = pol.action_net.weight.detach().numpy(); arrs["ba"] = pol.action_net.bias.detach().numpy()
    arrs["n_hidden"] = np.array(len(lins))
    act = [m for m in pol.mlp_extractor.policy_net if not isinstance(m, torch.nn.Linear)]
    arrs["activation"] = np.array(type(act[0]).__name__ if act else "Tanh")
    if os.path.exists(vecnorm_path):
        with open(vecnorm_path, "rb") as f:
            vn = pickle.load(f)
        arrs["obs_mean"] = vn.obs_rms.mean; arrs["obs_var"] = vn.obs_rms.var
        arrs["eps"] = np.array(vn.epsilon); arrs["clip_obs"] = np.array(vn.clip_obs)
    else:
        print("WARNING: no vecnormalize.pkl found next to the model; observations will not be normalised")
    np.savez(out_path, **arrs)
    print("exported ->", out_path)
    return out_path

class NpzPolicy:
    """Pure numpy inference: runs anywhere (laptop, Raspberry Pi, ...)."""
    def __init__(self, path):
        z = np.load(path, allow_pickle=False)
        self.n = int(z["n_hidden"]); self.W = [z[f"W{i}"] for i in range(self.n)]; self.b = [z[f"b{i}"] for i in range(self.n)]
        self.Wa, self.ba = z["Wa"], z["ba"]
        name = str(z["activation"]); self.act = {"Tanh": np.tanh, "ReLU": lambda x: np.maximum(x, 0),
                                                 "ELU": lambda x: np.where(x > 0, x, np.expm1(np.minimum(x, 0)))}.get(name, np.tanh)
        self.has_norm = "obs_mean" in z.files
        if self.has_norm:
            self.mean, self.var, self.eps, self.clip = z["obs_mean"], z["obs_var"], float(z["eps"]), float(z["clip_obs"])
    def act_(self, obs):
        x = np.asarray(obs, np.float64)
        if self.has_norm:
            x = np.clip((x - self.mean) / np.sqrt(self.var + self.eps), -self.clip, self.clip)
        for W, b in zip(self.W, self.b):
            x = self.act(W @ x + b)
        return np.clip(self.Wa @ x + self.ba, -1, 1)
    __call__ = act_

def load_policy(path):
    if path.endswith(".zip"):
        path = export_npz(path)
    return NpzPolicy(path)

if __name__ == "__main__":
    export_npz(sys.argv[1])
