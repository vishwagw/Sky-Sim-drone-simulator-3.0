"""CPU-only PPO (stable-baselines3) on SkySimCoreHoverEnv, matched to train_es.py.

Same task, same 12-32-32-4 tanh policy size, same 20 validation and 100 held-out
test seeds, same success criterion. Budget is wall-clock (PPO_WALL_S, default
1100 s = the ES run), on all available CPUs, torch forced to CPU.

Usage: SKYSIM_CORE_LIB=/path/libskysim_core.so python train_ppo.py OUTDIR
"""
import json
import os
import sys
import time

import numpy as np
import torch

sys.path.insert(0, os.path.dirname(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "python"))
from stable_baselines3 import PPO  # noqa: E402
from stable_baselines3.common.callbacks import BaseCallback  # noqa: E402
from stable_baselines3.common.vec_env import SubprocVecEnv, VecNormalize  # noqa: E402
from skysim.core_env import SkySimCoreHoverEnv  # noqa: E402

WALL = float(os.environ.get("PPO_WALL_S", 1100))
N_ENVS = int(os.environ.get("PPO_ENVS", 8))
SEED = int(os.environ.get("PPO_SEED", 0))


def make(i):
    def _f():
        e = SkySimCoreHoverEnv()
        e.reset(seed=SEED * 1000 + i)
        return e
    return _f


def rollout(model, vn, env, seed):
    o, _ = env.reset(seed=int(seed))
    ret, errs, crashed = 0.0, [], False
    while True:
        a, _ = model.predict(vn.normalize_obs(o[None]), deterministic=True)
        o, r, term, trunc, info = env.step(a[0])
        ret += r
        errs.append(info["pos_err"])
        crashed |= term
        if term or trunc:
            break
    tail = np.mean(errs[-100:]) if len(errs) >= 100 else float("inf")
    return ret, tail, crashed


def evaluate(model, vn, env, seeds):
    res = [rollout(model, vn, env, s) for s in seeds]
    ret = np.array([r[0] for r in res]); tail = np.array([r[1] for r in res])
    crash = np.array([r[2] for r in res]); ok = (~crash) & (tail < 0.25)
    return dict(return_mean=float(ret.mean()),
                tail_err_median=float(np.median(tail[~crash])) if (~crash).any() else None,
                tail_err_mean=float(tail[~crash].mean()) if (~crash).any() else None,
                crash_rate=float(crash.mean()), success_rate=float(ok.mean()))


class Budget(BaseCallback):
    def __init__(self, vn, t0, log, every=100_000):
        super().__init__(); self.vn, self.t0, self.log, self.every = vn, t0, log, every
        self.next = 0; self.env = SkySimCoreHoverEnv(); self.val = list(range(10_000, 10_020))
        self.eval_s = 0.0

    def _on_step(self):
        if self.num_timesteps >= self.next:
            self.next += self.every
            te = time.perf_counter()
            ev = [rollout(self.model, self.vn, self.env, s) for s in self.val]
            self.eval_s += time.perf_counter() - te
            el = time.perf_counter() - self.t0 - self.eval_s   # training time only
            vr = float(np.mean([e[0] for e in ev])); vt = float(np.median([e[1] for e in ev]))
            self.log.append(dict(steps=int(self.num_timesteps), wall_s=el, val_return=vr, val_tail_err_median=vt))
            print(f"steps {self.num_timesteps:9d}  train_wall {el:7.1f}s  val_return {vr:8.4f}  tail {vt:.3f} m", flush=True)
        return (time.perf_counter() - self.t0 - self.eval_s) < WALL


def main(out):
    os.makedirs(out, exist_ok=True)
    torch.set_num_threads(1)
    venv = VecNormalize(SubprocVecEnv([make(i) for i in range(N_ENVS)]), norm_obs=True, norm_reward=True,
                        gamma=0.99, clip_obs=10.0)
    model = PPO("MlpPolicy", venv, n_steps=512, batch_size=512, n_epochs=10, learning_rate=3e-4,
                gamma=0.99, gae_lambda=0.95, clip_range=0.2, ent_coef=0.0, max_grad_norm=0.5,
                policy_kwargs=dict(net_arch=dict(pi=[32, 32], vf=[64, 64]), activation_fn=torch.nn.Tanh,
                                   log_std_init=-1.0),
                device="cpu", seed=SEED, verbose=0)
    log = []
    t0 = time.perf_counter()
    cb = Budget(venv, t0, log)
    model.learn(total_timesteps=10**9, callback=cb)
    wall = time.perf_counter() - t0 - cb.eval_s
    venv.training = False
    test_env = SkySimCoreHoverEnv()
    n_pi = sum(p.numel() for n, p in model.policy.named_parameters()
               if n.startswith("mlp_extractor.policy_net") or n.startswith("action_net") or n == "log_std")
    res = dict(algo="PPO (stable-baselines3)", n_envs=N_ENVS, seed=SEED, policy_params=int(n_pi),
               env_steps=int(model.num_timesteps), phys_ticks=int(model.num_timesteps) * test_env.n_sub,
               wall_s=wall, cpus=os.cpu_count(), torch=torch.__version__,
               ppo=evaluate(model, venv, test_env, list(range(20_000, 20_100))), log=log)
    model.save(os.path.join(out, "ppo_model")); venv.save(os.path.join(out, "ppo_vecnorm.pkl"))
    json.dump(res, open(os.path.join(out, "ppo_results.json"), "w"), indent=1)
    print(json.dumps({k: res[k] for k in ("wall_s", "env_steps", "ppo")}, indent=1))


if __name__ == "__main__":
    main(sys.argv[1])
