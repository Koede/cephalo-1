"""Organic horse body following Forest's sketch (version C: forelegs under the neck), lofted as elliptical sections
along a curved spine that runs from the tail up the neck to the collar.  3D sketch only (no CAD).
python3 horse_shell.py -> sheet_horse_shell.png"""
import sys
sys.path.insert(0, "/home/claude/octodrone")
import numpy as np, sk3d
import sketch_body as SB
C = sk3d.COL
b = SB.bC; out = b["out"] / 1000.0; cc = b["cc"] / 1000.0; n_up = SB.n_up; band = SB.band
TILT = SB.TILT

def seg_hits(p, d):
    """Intersections of the line p + t d with the outline polygon -> sorted t."""
    ts = []
    for a, c in zip(out, np.roll(out, -1, axis=0)):
        e = c - a; M = np.array([d, -e]).T
        if abs(np.linalg.det(M)) < 1e-12: continue
        t, u = np.linalg.solve(M, a - p)
        if 0 <= u <= 1: ts.append(t)
    return sorted(ts)

# spine: through the body middle, bending up the neck axis to the collar
xs = out[:, 0]; x_tail, x_front = xs.min(), xs.max()
base = cc - n_up * 0.26                                       # where the neck axis enters the chest
body_pts = []
for x in np.linspace(x_tail + 0.02, base[0] - 0.05, 14):
    t = seg_hits(np.array([x, 0.0]), np.array([0, 1.0]))
    body_pts.append(np.array([x, 0.5 * (t[0] + t[-1])]))
neck_pts = [base + (cc - base) * u for u in np.linspace(0.15, 1.0, 9)]
spine = np.array(body_pts + neck_pts)
# half-widths (y) along the spine, est.: croup 0.13, barrel 0.15, chest 0.14, neck base 0.135, collar = 0.118 round
L = np.r_[0, np.cumsum(np.linalg.norm(np.diff(spine, axis=0), axis=1))]; u = L / L[-1]
W = np.interp(u, [0, 0.06, 0.25, 0.5, 0.68, 0.8, 1.0], [0.03, 0.10, 0.13, 0.15, 0.14, 0.13, 0.118])

rings = []
for i, p in enumerate(spine):
    tng = spine[min(i + 1, len(spine) - 1)] - spine[max(i - 1, 0)]; tng /= np.linalg.norm(tng)
    nrm = np.array([-tng[1], tng[0]])
    t = seg_hits(p, nrm)
    lo, hi = (t[0], t[-1]) if len(t) >= 2 else (-0.05, 0.05)
    if i == len(spine) - 1: lo, hi = -0.118, 0.118          # the collar is round
    c2 = p + nrm * 0.5 * (lo + hi); h = 0.5 * (hi - lo)
    th = np.linspace(0, 2 * np.pi, 33)
    ring = [np.array([c2[0] + nrm[0] * h * np.sin(a), W[i] * np.cos(a), c2[1] + nrm[1] * h * np.sin(a)]) for a in th]
    rings.append(ring)

def body(sc):
    for r0, r1 in zip(rings[:-1], rings[1:]):
        for j in range(32):
            sc.poly([r0[j], r0[j + 1], r1[j + 1], r1[j]], C["horse"], ec=(0, 0, 0, 0.08))
    tail = rings[0]; ctr = np.mean(tail[:-1], axis=0)
    for j in range(32): sc.poly([ctr, tail[j], tail[j + 1]], C["horse"], ec=(0, 0, 0, 0))
    sc.ring((cc[0], 0), 0.100, 0.118, cc[1] - 0.012, cc[1] + 0.001, C["latch"])   # collar band (drawn level; tilt shown in the octopus)

def legs(sc):
    for L_, hip in ((b["legF"], b["hipF"]), (b["legR"], b["hipR"])):
        P = [np.array(p) / 1000.0 for p in L_]
        for s in (1, -1):
            y0 = s * 0.13; y1 = s * 0.16
            pts = [np.array([P[0][0], y0, P[0][1]]), np.array([P[1][0], y1, P[1][1]]), np.array([P[2][0], y1, P[2][1]]), np.array([P[3][0], y1, P[3][1] + 0.02])]
            sc.cyl(pts[0] - [0, 0.03 * s, 0], pts[0] + [0, 0.03 * s, 0], 0.045, C["act"])
            sc.tube(pts[:3], 0.020, C["horse"]); sc.tube(pts[2:], 0.014, C["horse"]); sc.sphere(pts[3] - [0, 0, 0.005], 0.026, C["tpu"])

def octopus(sc):
    n0 = len(sc.polys)
    sc.octopus(base=(0, 0, 0), yaw=45, bars=0.194, arms="centaur")
    R = sk3d.rot("y", TILT)
    for k in range(n0, len(sc.polys)):
        sc.polys[k] = sc.polys[k] @ R.T + np.array([cc[0], 0, cc[1]])

fig = sk3d.figure(16, 9, "Equo-1 as a horse, not a box: the body sweeps up into the neck, the collar is the top of the neck")
views = [([0.0, 0.08, 0.52, 0.86], 8, -70, "three-quarter"), ([0.50, 0.40, 0.50, 0.55], 0, -90, "side"), ([0.55, 0.0, 0.40, 0.45], 12, -10, "front")]
for pos, el, az, name in views:
    sc = sk3d.Scene(fig, pos, elev=el, azim=az)
    sc.ground((-0.72, -0.45), (0.55, 0.45))
    body(sc); legs(sc); octopus(sc)
    sc.done((-0.72, -0.5, 0.0), (0.55, 0.5, 1.1))
r = SB.rC
sk3d.panel_text(fig, 0.02, 0.10, [
    f"Neck leans {TILT:.0f} deg forward; bars run down the neck.  Forelegs under the neck, packs at the croup.",
    f"Top of octopus about {r['height_top']:.0f} mm, body {r['length']:.0f} mm long, centaur about {r['m']:.0f} kg (est.), front legs carry {r['share']*100:.0f} %.",
], fs=12)
sk3d.save(fig, "sheet_horse_shell.png")
