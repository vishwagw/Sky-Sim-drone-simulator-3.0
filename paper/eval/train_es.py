"""CPU-only policy learning on SkySimCoreHoverEnv with OpenAI-style evolution strategies.

No GPU and no deep-learning framework: NumPy MLP (12-32-32-4, tanh), antithetic
Gaussian perturbations, centred-rank fitness shaping, Adam. Compares the learned
policy with a hand-tuned cascaded PID baseline on 100 held-out seeds.

Usage: SKYSIM_CORE_LIB=/path/libskysim_core.so python train_es.py OUTDIR
"""
import json
import os
import sys
import time
from multiprocessing import Pool

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "python"))
from skysim.core_env import SkySimCoreHoverEnv  # noqa: E402

SIZES = [12, 32, 32, 4]
N_PARAMS = sum(a * b + b for a, b in zip(SIZES[:-1], SIZES[1:]))
OBS_SCALE = np.array([2, 2, 2, 2, 2, 2, 0.5, 0.5, 1, 4, 4, 4], dtype=np.float64)


def policy(theta, o):
    x = np.clip(o / OBS_SCALE, -5, 5)
    i = 0
    for k, (a, b) in enumerate(zip(SIZES[:-1], SIZES[1:])):
        W = theta[i:i + a * b].reshape(a, b); i += a * b
        bb = theta[i:i + b]; i += b
        x = x @ W + bb
        x = np.tanh(x)
    return x


def pid_policy(o, st):
    ex, ey, ez = o[0:3]
    vx, vy, vz = o[3:6]
    st["iy"] = np.clip(st.get("iy", 0.0) + ey * 0.02, -3, 3)
    roll = np.clip(-0.35 * ex - 0.45 * vx, -0.4, 0.4) / 0.4
    pitch = np.clip(-0.35 * ez - 0.45 * vz, -0.4, 0.4) / 0.4
    thr = np.clip(-0.10 * ey - 0.08 * vy - 0.02 * st["iy"], -0.3, 0.3) / 0.3
    yaw = np.clip(-1.0 * o[8], -1, 1)
    return np.array([roll, pitch, yaw, thr])


_env = None


def _get_env():
    global _env
    if _env is None:
        _env = SkySimCoreHoverEnv()
    return _env


def rollout(theta, seed, pid=False):
    env = _get_env()
    o, _ = env.reset(seed=int(seed))
    ret, st, errs, crashed = 0.0, {}, [], False
    while True:
        a = pid_policy(o, st) if pid else policy(theta, o)
        o, r, term, trunc, info = env.step(a)
        ret += r
        errs.append(info["pos_err"])
        if term:
            crashed = True
        if term or trunc:
            break
    tail = np.mean(errs[-100:]) if len(errs) >= 100 else float("inf")
    return ret, tail, crashed


def _eval_pair(args):
    theta, eps, sigma, seeds = args
    rp = np.mean([rollout(theta + sigma * eps, s)[0] for s in seeds])
    rn = np.mean([rollout(theta - sigma * eps, s)[0] for s in seeds])
    return rp, rn


def centred_ranks(x):
    r = np.empty_like(x); r[np.argsort(x)] = np.arange(len(x))
    return r / (len(x) - 1) - 0.5


def evaluate(theta, seeds, pid=False):
    res = [rollout(theta, s, pid) for s in seeds]
    ret = np.array([r[0] for r in res]); tail = np.array([r[1] for r in res])
    crash = np.array([r[2] for r in res])
    ok = (~crash) & (tail < 0.25)
    return dict(return_mean=float(ret.mean()), tail_err_median=float(np.median(tail[~crash])) if (~crash).any() else None,
                tail_err_mean=float(tail[~crash].mean()) if (~crash).any() else None,
                crash_rate=float(crash.mean()), success_rate=float(ok.mean()))


def main(out):
    os.makedirs(out, exist_ok=True)
    rng = np.random.default_rng(0)
    theta = rng.normal(0, 0.1, N_PARAMS)
    theta[-(SIZES[-2] * SIZES[-1] + SIZES[-1]):] = 0.0   # zero output layer: start at hover
    pairs, sigma, lr, gens, eps_per = 24, 0.04, 0.02, int(os.environ.get("ES_GENS", 300)), 3
    m = np.zeros(N_PARAMS); v = np.zeros(N_PARAMS); b1, b2 = 0.9, 0.999
    log = []
    t0 = time.perf_counter()
    phys_steps = 0
    env_tmp = SkySimCoreHoverEnv()
    steps_per_ep = env_tmp.max_steps * env_tmp.n_sub
    env_tmp.close()
    val_seeds = list(range(10_000, 10_020))
    with Pool(os.cpu_count()) as pool:
        for g in range(1, gens + 1):
            E = rng.normal(0, 1, (pairs, N_PARAMS))
            seeds = rng.integers(0, 1_000_000, eps_per)
            res = pool.map(_eval_pair, [(theta, E[i], sigma, seeds) for i in range(pairs)])
            phys_steps += 2 * pairs * eps_per * steps_per_ep   # upper bound (episodes may end early)
            R = np.array(res)
            shaped = centred_ranks(R.ravel()).reshape(R.shape)
            grad = ((shaped[:, 0] - shaped[:, 1])[:, None] * E).sum(0) / (2 * pairs * sigma)
            grad -= 0.005 * theta
            m = b1 * m + (1 - b1) * grad; v = b2 * v + (1 - b2) * grad ** 2
            mh = m / (1 - b1 ** g); vh = v / (1 - b2 ** g)
            theta = theta + lr * mh / (np.sqrt(vh) + 1e-8)
            if g % 10 == 0 or g == 1:
                ev = pool.map(_val_one, [(theta, s) for s in val_seeds])
                vr = float(np.mean([e[0] for e in ev])); vt = float(np.median([e[1] for e in ev]))
                el = time.perf_counter() - t0
                log.append(dict(gen=g, wall_s=el, phys_steps=phys_steps, val_return=vr, val_tail_err_median=vt))
                print(f"gen {g:4d}  wall {el:7.1f}s  val_return {vr:8.4f}  tail_err {vt:.3f} m", flush=True)
    wall = time.perf_counter() - t0
    np.save(os.path.join(out, "es_theta.npy"), theta)
    test = list(range(20_000, 20_100))
    res = dict(n_params=N_PARAMS, gens=gens, pairs=pairs, eps_per=eps_per, wall_s=wall,
               phys_steps_upper=phys_steps, cpus=os.cpu_count(),
               es=evaluate(theta, test), pid=evaluate(None, test, pid=True), log=log)
    json.dump(res, open(os.path.join(out, "es_results.json"), "w"), indent=1)
    print(json.dumps({k: res[k] for k in ("wall_s", "es", "pid")}, indent=1))


def _val_one(args):
    theta, s = args
    r = rollout(theta, s)
    return r[0], r[1]


if __name__ == "__main__":
    main(sys.argv[1])
