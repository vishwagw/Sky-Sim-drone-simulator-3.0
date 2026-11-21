"""Generate all paper figures (PDF, IEEE single-column width) from paper/data."""
import json
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

D = sys.argv[1] if len(sys.argv) > 1 else "paper/data"
F = sys.argv[2] if len(sys.argv) > 2 else "paper/figs"
os.makedirs(F, exist_ok=True)

C1, C2, C3, C4 = "#2a78d6", "#eb6834", "#1baf7a", "#eda100"
INK, MUTED, GRID = "#222222", "#6b6b6b", "#e4e4e0"
plt.rcParams.update({
    "font.family": "serif", "font.serif": ["STIXGeneral", "DejaVu Serif"], "mathtext.fontset": "stix",
    "font.size": 8, "axes.labelsize": 8, "legend.fontsize": 7, "xtick.labelsize": 7, "ytick.labelsize": 7,
    "axes.edgecolor": MUTED, "axes.labelcolor": INK, "xtick.color": MUTED, "ytick.color": MUTED,
    "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.6, "axes.spines.top": False,
    "axes.spines.right": False, "lines.linewidth": 1.4, "legend.frameon": False,
    "savefig.bbox": "tight", "savefig.pad_inches": 0.02,
})
W = 3.45


def save(fig, name):
    for a in fig.axes:
        try:
            a.ticklabel_format(axis="y", style="plain", useOffset=False)
        except AttributeError:
            pass
    fig.savefig(os.path.join(F, name + ".pdf"))
    fig.savefig(os.path.join(F, name + ".png"), dpi=200)
    plt.close(fig)


# 1. Static rotor coefficients vs UIUC -------------------------------------------------
u = pd.read_csv(os.path.join(D, "uiuc_apcsf_10x4.7_static.txt"), sep=r"\s+", comment="#")
s0 = pd.read_csv(os.path.join(D, "static_skysim.csv")); s0 = s0[s0.config == "apc10x4.7"]
s1 = pd.read_csv(os.path.join(D, "static_skysim_cal.csv")); s1 = s1[s1.config == "apc10x4.7"]
rb = pd.read_csv(os.path.join(D, "static_refbemt.csv")); rb = rb[rb.variant == "bemt_tiploss"]
off = float(open(os.path.join(D, "apc_offset.txt")).read())
fig, ax = plt.subplots(1, 2, figsize=(W, 1.75))
for a, col, lab in [(ax[0], "CT", "$C_T$"), (ax[1], "CP", "$C_P$")]:
    a.plot(u.RPM, u[col], "o", ms=3.2, color=INK, mfc="white", mew=0.9, label="UIUC measured", zorder=5)
    a.plot(s0.rpm, s0[col], "--", color=C2, label="SkySim, uncalibrated")
    a.plot(s1.rpm, s1[col], "-", color=C1, label=f"SkySim, $+{off:g}^\\circ$ pitch")
    a.plot(rb.rpm, rb[col], ":", color=C3, lw=1.6, label="Reference BEMT")
    a.set_xlim(2000, 7000); a.set_xlabel("RPM"); a.set_ylabel(lab)
    a.set_ylim(0, None)
ax[0].legend(loc="lower center", bbox_to_anchor=(1.1, 1.0), ncol=2, handlelength=2.2, columnspacing=1.0)
fig.subplots_adjust(wspace=0.42)
save(fig, "fig_static")

# 2. Axial flight: thrust vs climb rate at hover RPM ----------------------------------
af = pd.read_csv(os.path.join(D, "axial_fixed.csv")); ao = pd.read_csv(os.path.join(D, "axial_orig.csv"))
# Momentum theory is only valid outside the windmill-brake/VRS region; keep
# |descent| < 0.4 v_h (v_h = 5.44 m/s) for both curves.
af = af[(af.Vc >= -2.0) & (af.Vc <= 2.0)]; ao = ao[(ao.Vc >= -4.0) & (ao.Vc <= 2.0)]
ao = ao[(ao.Vc >= -2.0)]
fig, a = plt.subplots(figsize=(W, 1.8))
a.plot(af.Vc, af["T"], "-", color=C1, label="corrected: thrust falls in climb (damping)")
a.plot(ao.Vc, ao["T"], "--", color=C2, label="v3.0: thrust rises in climb (anti-damping)")
a.set_xlim(-2, 2)
a.axhline(af.loc[af.Vc.abs() < 1e-9, "T"].iloc[0], color=MUTED, lw=0.7)
a.axvline(0, color=MUTED, lw=0.7)
a.set_xlabel("Axial velocity $V_c$ (m/s, climb $+$)"); a.set_ylabel("Rotor thrust (N)")

a.legend(loc="lower center", bbox_to_anchor=(0.5, 1.0), ncol=1)
save(fig, "fig_axial")

# 3. Ground effect --------------------------------------------------------------------
g = pd.read_csv(os.path.join(D, "ground.csv")); m = g[g.kind == "mult"]
fig, a = plt.subplots(figsize=(W, 1.7))
a.plot(m.h_over_R, m.value_cb, ":", color=INK, lw=1.4, label="Cheeseman–Bennett")
a.plot(m.h_over_R, m.value, "-", color=C1, label="SkySim (blended)")
a.set_xlabel("Height above ground $z/R$"); a.set_ylabel("$T_{IGE}/T_{OGE}$")
a.set_xlim(0.25, 4); a.set_ylim(0.98, 1.6); a.legend()
save(fig, "fig_ground")

# 4. VRS map ---------------------------------------------------------------------------
v = pd.read_csv(os.path.join(D, "vrs.csv"))
fig, a = plt.subplots(figsize=(W, 1.6))
for mul, col, ls in [(0.0, C1, "-"), (0.25, C2, "--")]:
    q = v[np.isclose(v.mu_l, mul)]
    a.plot(q.mu_d, q.factor_mean, ls, color=col, label=f"$\\mu_l = {mul:g}$")
a.set_xlabel("Normalised descent rate $V_d/v_h$"); a.set_ylabel("Thrust factor")
a.set_xlim(0, 2); a.legend(loc="lower right")
save(fig, "fig_vrs")

# 5. Battery ---------------------------------------------------------------------------
b = pd.read_csv(os.path.join(D, "battery.csv"))
fig, ax = plt.subplots(1, 2, figsize=(W, 1.65))
ax[0].plot(b.t_min, b.v_ocv, ":", color=INK, lw=1.3, label="OCV")
ax[0].plot(b.t_min, b.v_term, "-", color=C1, label="terminal")
ax[0].axhline(14.0, color=MUTED, lw=0.6); ax[0].axhline(13.2, color=MUTED, lw=0.6)
ax[0].set_xlabel("Hover time (min)"); ax[0].set_ylabel("Pack voltage (V)"); ax[0].legend(loc="upper right")
ax[1].plot(b.t_min, b.thr, "-", color=C1)
ax[1].set_xlabel("Hover time (min)"); ax[1].set_ylabel("Hover throttle")
fig.subplots_adjust(wspace=0.5)
save(fig, "fig_battery")

# 6. ES learning curve ------------------------------------------------------------------
p = os.path.join(D, "es", "es_results.json")
if os.path.exists(p):
    r = json.load(open(p)); L = pd.DataFrame(r["log"])
    fig, a = plt.subplots(figsize=(W, 1.7))
    a.plot(L.wall_s / 60, L.val_return, "-o", ms=2.2, color=C1, label="ES (NumPy)")
    pp = os.path.join(D, "ppo", "ppo_results.json")
    if os.path.exists(pp):
        P = pd.DataFrame(json.load(open(pp))["log"])
        a.plot(P.wall_s / 60, P.val_return, "-s", ms=2.0, color=C3, label="PPO (SB3, CPU)")
    a.axhline(r["pid"]["return_mean"], ls="--", color=C2, label="cascaded PID")
    a.set_ylim(-3.0, 0.0)
    a.set_xlabel("Wall-clock time on 2 vCPUs (min)"); a.set_ylabel("Mean episode return")
    a.legend(loc="lower right")
    save(fig, "fig_es")
print("figures written to", F)
