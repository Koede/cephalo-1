"""Equo-1 v2 look study: DEEP-Robotics-horse proportions and sculpted legs, a War-Horse-style ribbed frame (open hoops
on a spine, translucent skin), CORLEO's headless-horse outline and swinging rear leg unit, XPeng-style arm as a tail.
Octopus on the forward-leaning neck.  3D sketch only.  python3 equo_v2.py -> sheet_equo_v2.png"""
import sys, contextlib, io
sys.path.insert(0, "/home/claude/octodrone")
import numpy as np, sk3d
with contextlib.redirect_stdout(io.StringIO()):
    import sketch_body as SB
C = sk3d.COL
b = SB.bC; out = b["out"] / 1000.0; cc = b["cc"] / 1000.0; n_up = SB.n_up; TILT = SB.TILT
hipF = b["hipF"] / 1000.0; hipR = b["hipR"] / 1000.0

def hits(p, d):
    ts = []
    for a, c in zip(out, np.roll(out, -1, axis=0)):
        e = c - a; M = np.array([d, -e]).T
        if abs(np.linalg.det(M)) < 1e-12: continue
        t, u = np.linalg.solve(M, a - p)
        if 0 <= u <= 1: ts.append(t)
    return sorted(ts)

# spine from the croup, along the back, up the neck to the collar
x_tail = out[:, 0].min(); base = cc - n_up * 0.26
body = []
for x in np.linspace(x_tail + 0.03, base[0] - 0.04, 12):
    t = hits(np.array([x, 0.0]), np.array([0, 1.0])); body.append(np.array([x, 0.5 * (t[0] + t[-1])]))
spine = np.array(body + [base + (cc - base) * u for u in np.linspace(0.2, 1.0, 6)])
L = np.r_[0, np.cumsum(np.linalg.norm(np.diff(spine, axis=0), axis=1))]; u = L / L[-1]
W = np.interp(u, [0, 0.08, 0.3, 0.55, 0.72, 0.85, 1.0], [0.05, 0.11, 0.135, 0.15, 0.14, 0.13, 0.118])
rings = []
for i, p in enumerate(spine):
    tg = spine[min(i + 1, len(spine) - 1)] - spine[max(i - 1, 0)]; tg /= np.linalg.norm(tg); nr = np.array([-tg[1], tg[0]])
    t = hits(p, nr); lo, hi = (t[0], t[-1]) if len(t) >= 2 else (-0.05, 0.05)
    if i == len(spine) - 1: lo, hi = -0.118, 0.118
    c2 = p + nr * 0.5 * (lo + hi); h = min(0.5 * (hi - lo), 0.15)   # clamp the chest-to-neck junction sections (loft artefact)
    th = np.linspace(0, 2 * np.pi, 29)
    rings.append(np.array([[c2[0] + nr[0] * h * np.sin(a), W[i] * np.cos(a), c2[1] + nr[1] * h * np.sin(a)] for a in th]))

def P3(p2, y=0.0): return np.array([p2[0], y, p2[1]])

def frame(sc):
    for r in rings[1:]:                                                     # rib hoops (printed PA-CF)
        sc.tube(list(r), 0.006, C["frame"], n=6, balls=False)
    for j in (0, 7, 14, 21, 4, 10, 18, 24):                                 # stringers
        sc.tube([r[j] for r in rings], 0.005, C["frame"], n=6, balls=False)
    for r0, r1 in zip(rings[:-1], rings[1:]):                               # translucent skin
        for j in range(28):
            sc.poly([r0[j], r0[j + 1], r1[j + 1], r1[j]], C["horse"], alpha=0.16, ec=(0, 0, 0, 0))
    sc.tube([P3(p) for p in spine[:12]], 0.012, C["bar"], n=8)              # carbon keel along the back
    # packs at the croup (on the sled), controller mid-body
    sc.box(P3(spine[3]) - [0, 0, 0.035], (0.16, 0.10, 0.05), fc=C["pack"])
    sc.box(P3(spine[6]) - [0, 0, 0.04], (0.10, 0.07, 0.02), fc=C["spring"])

def hoof(sc, p):
    sc.cyl(p + [0, 0, 0.0], p + [0, 0, 0.035], 0.034, C["tpu"], r1=0.026, n=16)

def leg(sc, pts, y, rads, fc=C["horse"]):
    pts = [np.array([q[0], y, q[1]]) for q in pts]
    for a, c, (r0, r1) in zip(pts[:-1], pts[1:], rads):
        sc.cyl(a, c, r0, fc, n=14, r1=r1)
    for q, r in zip(pts[1:-1], [rr[1] for rr in rads[:-1]]):
        sc.sphere(q, r * 1.15, C["act"] if r > 0.02 else fc, n=6)
    hoof(sc, pts[-1] - [0, 0, 0.0])

def legs(sc):
    xf, zf = hipF; xr, zr = hipR
    front = [(xf, zf), (xf - 0.01, zf - 0.17), (xf + 0.005, zf - 0.34), (xf + 0.01, 0.075), (xf + 0.04, 0.035)]   # shoulder, knee, ..., fetlock, hoof
    hind = [(xr, zr), (xr + 0.07, zr - 0.15), (xr - 0.05, zr - 0.29), (xr - 0.04, 0.075), (xr - 0.01, 0.035)]     # hip, stifle, hock, fetlock, hoof
    for s in (1, -1):
        y = s * 0.115
        leg(sc, front, y, [(0.032, 0.026), (0.024, 0.016), (0.016, 0.014), (0.014, 0.012)])
        leg(sc, hind, y, [(0.034, 0.028), (0.026, 0.017), (0.016, 0.014), (0.014, 0.012)])
        for (x, z) in (hipF, hipR):                                          # hip actuator pods (RobStride / CyberGear class)
            sc.cyl([x, s * 0.07, z], [x, s * 0.115, z], 0.042, C["act"], n=18)
    # CORLEO-style rear swing unit: both hind hips on a sub-frame pivoting under the barrel, with a coil-over
    piv = np.array([xr + 0.20, 0, zr + 0.01])
    for s in (1, -1):
        sc.cyl(piv + [0, s * 0.06, 0], [xr, s * 0.07, zr], 0.012, C["bar"], n=8)
    sc.cyl(piv - [0, 0.08, 0], piv + [0, 0.08, 0], 0.014, C["act"], n=10)
    sc.cyl([xr + 0.03, 0, zr + 0.02], [xr + 0.10, 0, zr + 0.11], 0.010, C["spring"], n=8)

def tail(sc):
    t0 = P3(spine[0]) + [-0.01, 0, 0.0]
    pts = [t0, t0 + [-0.08, 0, 0.03], t0 + [-0.15, 0, 0.0], t0 + [-0.19, 0, -0.08], t0 + [-0.18, 0, -0.17]]
    sc.tube(pts, 0.013, C["octo2"], n=8)
    for q in pts[1:-1]: sc.sphere(q, 0.018, C["act"], n=5)
    sc.sphere(pts[-1], 0.022, C["tpu"], n=6)

def octopus(sc):
    n0 = len(sc.polys)
    sc.octopus(base=(0, 0, 0), yaw=45, bars=0.194, arms="centaur")
    R = sk3d.rot("y", TILT)
    for k in range(n0, len(sc.polys)):
        sc.polys[k] = sc.polys[k] @ R.T + np.array([cc[0], 0, cc[1]])
    sc.ring((cc[0], 0), 0.104, 0.120, cc[1] - 0.014, cc[1] - 0.002, C["latch"])

def scene(sc, labels=False):
    sc.ground((-0.75, -0.42), (0.55, 0.42))
    frame(sc); legs(sc); tail(sc); octopus(sc)
    if labels:
        T = lambda p, s: sc.text(p, s, fs=11)
        T(P3(spine[5]) + [0, -0.2, 0.18], "open rib hoops + translucent skin")
        T(P3(spine[4]) + [0.0, -0.25, 0.10], "packs on the sled")
        T([hipR[0] - 0.2, -0.3, hipR[1] - 0.12], "rear swing unit + coil-over")
        T([hipF[0] + 0.06, -0.25, 0.20], "sculpted legs, TPU hooves")
        T(P3(spine[0]) + [-0.32, -0.15, -0.10], "tail = 4-joint arm / kickstand")
        T([cc[0] + 0.05, -0.25, cc[1] - 0.08], "collar on the neck")

fig = sk3d.figure(16, 9, "Equo-1 v2: DEEP-horse legs, War-Horse ribbed body, CORLEO rear swing unit, arm tail")
sc = sk3d.Scene(fig, [0.0, 0.02, 0.62, 0.92], elev=10, azim=-62); scene(sc, labels=True); sc.done((-0.8, -0.45, 0), (0.6, 0.45, 1.1))
sc = sk3d.Scene(fig, [0.60, 0.46, 0.40, 0.48], elev=0, azim=-90); scene(sc); sc.done((-0.8, -0.45, 0), (0.6, 0.45, 1.1))
sc = sk3d.Scene(fig, [0.62, 0.02, 0.36, 0.44], elev=22, azim=-145); scene(sc); sc.done((-0.8, -0.45, 0), (0.6, 0.45, 1.1))
r = SB.rC
sk3d.panel_text(fig, 0.02, 0.10, [
    f"Shoulder 490 mm (DEEP horse scale), top of octopus {r['height_top']:.0f} mm, body {r['length']:.0f} mm, centaur about {r['m']:.0f} kg (est.).",
    "Neck leans 17 deg; the bars run down the neck.  Forelegs under the neck, packs aft: front legs carry about 62 %.",
], fs=12)
sk3d.save(fig, "sheet_equo_v2.png")
