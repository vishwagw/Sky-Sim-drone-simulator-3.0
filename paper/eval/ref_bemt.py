"""Independent reference blade-element momentum theory (BEMT) for hover/axial flight.

Annulus-wise inflow (Leishman, Principles of Helicopter Aerodynamics, ch. 3)
with Prandtl tip-loss, using the full UIUC radial chord/twist distribution.
Used to separate SkySim's parametrisation/uniform-inflow error from
aerofoil-model error when comparing against UIUC static measurements.
"""
import sys
import numpy as np

RHO = 1.225


def load_geom(path):
    d = np.loadtxt(path, comments="#", skiprows=2)
    return d[:, 0], d[:, 1], np.deg2rad(d[:, 2])


def bemt(rpm, R, r_R, c_R, beta, nb=2, a=5.73, cd0=0.012, cd2=0.080,
         Vc=0.0, tip_loss=True, n=200):
    omega = rpm * np.pi / 30.0
    x = np.linspace(r_R[0], 0.999, n)
    c = np.interp(x, r_R, c_R) * R
    th = np.interp(x, r_R, beta)
    sigma = nb * c / (np.pi * R)
    lc = Vc / (omega * R)
    lam = np.full_like(x, 0.05)
    F = np.ones_like(x)
    for _ in range(100):
        if tip_loss:
            phi = np.maximum(lam / x, 1e-4)
            f = 0.5 * nb * (1 - x) / (x * phi)
            F = (2 / np.pi) * np.arccos(np.clip(np.exp(-f), 0, 1))
            F = np.maximum(F, 1e-3)
        k = sigma * a / (16 * F) - lc / 2
        lam_new = np.sqrt(k ** 2 + sigma * a * th * x / (8 * F)) - k
        if np.max(np.abs(lam_new - lam)) < 1e-10:
            lam = lam_new
            break
        lam = 0.5 * lam + 0.5 * lam_new
    phi = np.arctan2(lam, x)
    alpha = th - phi
    cl = a * alpha
    cd = cd0 + cd2 * cl ** 2
    Vt = omega * x * R
    V2 = Vt ** 2 + (lam * omega * R) ** 2
    q = 0.5 * RHO * V2
    dr = np.gradient(x) * R
    dT = nb * q * c * (cl * np.cos(phi) - cd * np.sin(phi)) * dr
    dQ = nb * q * c * (cl * np.sin(phi) + cd * np.cos(phi)) * x * R * dr
    T, Q = dT.sum(), dQ.sum()
    P = Q * omega
    ns, D = rpm / 60.0, 2 * R
    return T, P, T / (RHO * ns ** 2 * D ** 4), P / (RHO * ns ** 3 * D ** 5)


if __name__ == "__main__":
    geom = sys.argv[1]
    r_R, c_R, beta = load_geom(geom)
    print("variant,rpm,T,P,CT,CP")
    for rpm in range(2000, 8001, 250):
        for name, kw in [("bemt_tiploss", dict(tip_loss=True)),
                         ("bemt_notiploss", dict(tip_loss=False))]:
            T, P, CT, CP = bemt(rpm, 0.127, r_R, c_R, beta, **kw)
            print(f"{name},{rpm},{T:.6f},{P:.6f},{CT:.6f},{CP:.6f}")
