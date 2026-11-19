"""Equo-1 scale for working arms: how big can the horse be while Cephalo-1's two front arms still work as arms?
Same technology (RS03 front hips, RS06 elsewhere, 30T:60T belts, shell v4); only the horse scales, by s about the ground
origin.  The octopus, its arms (cephalo4 rev 4: 8 joints p-y-p-y, s = 50..442 mm, spigot 478, limits from cephalo4),
the collar (dia 236) and the 228 mm neck sheaths do not scale.
Tasks (hand point = spigot + 60 mm along the tip axis: claw palm / fiddlehead wrap):
  T1 floor pick standing      hand on the ground 120 mm ahead of the front hooves, y = +-80 mm
  T2 floor pick bowing        same with the front end lowered (body pitched about the rear hips, <= 12 deg)
  T3 table work               hand 250 mm ahead of the chest at 750 mm height (a counter / bench)
  T4 hands meet               both hands at the centreline 200 mm ahead of the chest, at collar height - 150
  T5 reach the withers         hand on top of the back at the front hip line - 150 mm (front or side arm; hands things to the pack)
Every pose: joint limits, arm capsules (r 27..20 mm) >= 15 mm from the horse shell and the front legs, no self clash
with the octopus dome is assumed covered by the cephalo4 j0 limit.
Loads: horse mass at scale s = 12.81 octopus + 7.97 actuators (fixed) + 13.2 s^2.5 structure (s = 1 basis 34.0 kg);
joint torque ~ m g L, so margins scale as 34 / (m s); joint speed for the same 0.8 m/s fast trot ~ 1/s."""
import sys, io, contextlib, json
sys.path[:0] = ["/home/claude/octodrone", "/home/claude/octodrone/centaur", "/home/claude/octodrone/centaur/preview",
                "/home/claude/octodrone/centaur/legs_v2"]
import numpy as np
from scipy.optimize import least_squares
from matplotlib.path import Path
with contextlib.redirect_stdout(io.StringIO()):
    import chassis_v3 as C3
    import chassis_v4 as C4
    import legs_v2_study as LS
MM = 1e-3
# ---------------------------------------------------------------- arm (cephalo4 rev 4, copied constants)
ROOT_R, ROOT_Z = 104 * MM, 39 * MM
S_J = (50 + 56 * np.arange(8)) * MM; TYP = "pypypypy"; SPIGOT = 478 * MM; NJ = 8
LIM = np.radians([[-59, 55], [-30, 30]] + [[-100, 100]] * 6)
LEAD = 0.060
def rot(ax, a):
    c, s = np.cos(a), np.sin(a)
    if ax == "p": return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    if ax == "y": return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
def Ry(a):  # world pitch: +a tips +x down
    c, s = np.cos(a), np.sin(a); return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
def arm_fk(az, q, bp, bR):
    a = np.radians(az); x = np.array([np.cos(a), np.sin(a), 0]); y = np.array([-np.sin(a), np.cos(a), 0])
    R = bR @ np.stack([x, y, [0, 0, 1.0]], 1); p = bp + bR @ (ROOT_R * x + ROOT_Z * np.array([0, 0, 1.0]))
    pts, s = [p.copy()], 0.0
    for k in range(NJ):
        p = p + R[:, 0] * (S_J[k] - s); s = S_J[k]; pts.append(p.copy())
        R = R @ rot(TYP[k], q[k])
    p = p + R[:, 0] * (SPIGOT - s); pts.append(p.copy())
    return np.array(pts), p + R[:, 0] * LEAD, R
def capsules(pts, n=36):
    L = np.r_[0, np.cumsum(np.linalg.norm(np.diff(pts, axis=0), axis=1))]
    ss = np.linspace(S_J[0] + 0.015, L[-1], n)
    P = np.array([[np.interp(t, L, pts[:, j]) for j in range(3)] for t in ss])
    return P, np.interp(ss, [0, 0.2, SPIGOT], [0.027, 0.025, 0.020])

# ---------------------------------------------------------------- horse at scale s
SHELL = C4.shell_points()                          # (NX, NA+1, 3) at s = 1
B = LS.ARCH["B"]; PP = LS.panto_points(B, B.ik(0.0, -LS.H))
def horse(s, pitch=0.0):
    """shell points, leg capsule chains, hoof points, collar centre / tilt at scale s, body pitched by `pitch` (rad,
    +ve = nose down) about the rear hips (front legs bend, rear legs stand)."""
    f = s / C3.S                                   # geometry is built at C3.S; scale it to s for the sweep
    piv = np.array([C3.HIPR[0] * f, 0, C3.HIPR[1] * f])
    Rp = Ry(pitch)
    tf = lambda P: (P - piv) @ Rp.T + piv
    sh = tf(SHELL * f)
    legs, hooves = [], []
    for front, xh in ((True, C3.HIPF[0]), (False, C3.HIPR[0])):
        sg = -1.0 if front else 1.0
        for side in (1, -1):
            yl = C3.Y_LEG_F if front else C3.Y_LEG_R
            W = lambda p: np.array([xh * f + sg * p[0] * s, side * yl, LS.H * s + p[1] * s])
            chain = np.array([W(PP[k]) for k in ("hip", "stifle", "hock", "fet", "foot")])
            if getattr(C3, "KNEE_MODE", "") == "scapula":      # the pauldron over the knee motor, above the hip
                dfm = (chain[1] - chain[0]) / np.linalg.norm(chain[1] - chain[0])
                chain = np.vstack([chain[0] - dfm * (C3.R_K + 0.03), chain])
            if front: chain = tf(chain); chain[-1, 2] = 0.0          # bowing: hoof stays on the ground (approx.)
            legs.append(chain); hooves.append(chain[-1])
    ccw = np.array([C3.cc[0] * f, 0, C3.cc[1] * f])
    tilt = np.radians(C3.TILT) + pitch
    oct_p = tf(ccw); oct_R = Ry(tilt)
    return dict(sh=sh, legs=legs, hooves=np.array(hooves), oct_p=oct_p, oct_R=oct_R, piv=piv)

def shell_dist(H, P):
    """approx. signed clearance of points to the (scaled, pitched) shell: >0 outside.  Uses each x-section polygon."""
    sh = H["sh"]; out = np.full(len(P), 1.0)
    cen = sh[:, :-1].mean(1)
    for j, p in enumerate(P):
        # nearest section by x along the shell axis
        i = np.argmin(np.abs(cen[:, 0] - p[0]))
        if p[0] < sh[:, :, 0].min() - 0.03 or p[0] > sh[:, :, 0].max() + 0.03: continue
        poly = sh[i, :-1][:, 1:]
        q = p[1:]; seg = np.r_[poly, poly[:1]]; a, b = seg[:-1], seg[1:]
        t = np.clip(np.einsum("ij,ij->i", q - a, b - a) / np.einsum("ij,ij->i", b - a, b - a), 0, 1)
        d = np.min(np.linalg.norm(a + t[:, None] * (b - a) - q, axis=1))
        # in-x distance to the section plane also counts near the ends
        dx = 0.0 if sh[:, :, 0].min() <= p[0] <= sh[:, :, 0].max() else min(abs(p[0] - sh[:, :, 0].min()), abs(p[0] - sh[:, :, 0].max()))
        out[j] = -d if (Path(poly).contains_point(q) and dx == 0) else np.hypot(d, dx)
    return out

def leg_dist(H, P, r_leg):
    d = np.full(len(P), 9.0)
    for ch in H["legs"]:
        for a, b in zip(ch[:-1], ch[1:]):
            ab = b - a; t = np.clip(((P - a) @ ab) / (ab @ ab), 0, 1)
            d = np.minimum(d, np.linalg.norm(a + t[:, None] * ab - P, axis=1) - r_leg)
    return d

CLEAR = 0.015
def solve(H, az, target, s, q0=None):
    lo, hi = LIM.T; q_nom = np.zeros(NJ)
    def res(q):
        pts, hand, R = arm_fk(az, q, H["oct_p"], H["oct_R"])
        P, r = capsules(pts)
        cl = np.minimum(shell_dist(H, P), leg_dist(H, P, 0.045 * s)) - r - CLEAR
        cl = np.r_[cl, P[:, 2] - r - 0.0]                                          # arm above the floor
        return np.concatenate([(hand - target) * 10, np.minimum(cl, 0) * 20, 0.01 * (q - q_nom)])
    best = None
    starts = [q0] if q0 is not None else []
    starts += [np.radians([30, 0, 30, 0, 20, 0, 20, 0]), np.radians([50, 0, 60, 0, 40, 0, 10, 0]),
               np.radians([-20, 0, 50, 0, 50, 0, 30, 0]), np.radians([10, 0, 0, 0, 0, 0, 0, 0])]
    for st in starts:
        sol = least_squares(res, np.clip(st, lo + 1e-6, hi - 1e-6), bounds=(lo, hi), max_nfev=250)
        pts, hand, R = arm_fk(az, sol.x, H["oct_p"], H["oct_R"])
        P, r = capsules(pts)
        cl = min((shell_dist(H, P) - r).min(), (leg_dist(H, P, 0.045 * s) - r).min())
        err = np.linalg.norm(hand - target)
        sc_ = err + 10 * max(0, CLEAR - cl)
        if best is None or sc_ < best[0]: best = (sc_, sol.x, err, cl, pts, hand)
    return dict(q=best[1], err=best[2], clear=best[3], pts=best[4], hand=best[5],
                ok=bool(best[2] < 0.010 and best[3] >= CLEAR - 0.002))

AZ_R, AZ_L = 22.5, -22.5        # the front pair (octopus yaw 45 in the model: arms at +-22.5 deg from the heading)

def tasks(s):
    out = {}
    H = horse(s)
    chest = H["sh"][:, :, 0].max(); hoof = H["hooves"][:2, 0].max()
    out["T1 floor, standing"] = solve(H, AZ_R, np.array([hoof + 0.12, 0.08, 0.0]), s)
    # T2: smallest pitch (0..12 deg) that reaches
    out["T2 floor, bowing"] = None
    for deg in range(0, 13, 2):
        H2 = horse(s, np.radians(deg)); hoof2 = H2["hooves"][:2, 0].max()
        r = solve(H2, AZ_R, np.array([hoof2 + 0.12, 0.08, 0.0]), s)
        r["bow_deg"] = deg
        if r["ok"]: out["T2 floor, bowing"] = r; break
    if out["T2 floor, bowing"] is None: out["T2 floor, bowing"] = r
    out["T3 table 750 mm"] = solve(H, AZ_R, np.array([chest + 0.25, 0.08, 0.75]), s)
    zc = H["oct_p"][2] - 0.15
    out["T4 hands meet"] = solve(H, AZ_R, np.array([chest + 0.20, 0.035, zc]), s)          # hands side by side, 70 mm apart
    top_x = (C3.HIPF[0] / C3.S - 0.15) * s; sec = H["sh"][np.argmin(np.abs(H["sh"][:, :-1, 0].mean(1) - top_x))]
    for az in (AZ_R, 67.5, 112.5):                 # front arm, or a side arm reaching back over the horse
        r = solve(H, az, np.array([top_x, 0.05, sec[:, 2].max() + 0.03]), s); r["arm_az"] = az
        if r["ok"]: break
    out["T5 hand on the withers"] = r
    return out, H

def loads(s):
    m = 12.81 + 7.97 + 13.2 * s ** 2.5
    k = 34.0 / (m * s)
    return dict(mass=m, rated=1.47 * k, peak=1.34 * k, belt=1.28 * k, speed=2.05 * s, horse_height=LS.H * s)

def neck_fit(s):
    """sheath end inside the (scaled) side profile and clear of the front hip actuator (r 53 + 15 mm)"""
    cc = C3.cc * s; end = cc - C3.n_up * 0.228
    inside = Path(C3.OUT * s).contains_point(end)
    gap = np.linalg.norm(end - C3.HIPF * s) - 0.053 - 0.075      # sheath block r 75
    # collar vs neck: neck half-width at the collar scales (0.118 s) unless held at the collar size
    return dict(sheath_inside=bool(inside), sheath_to_hip_pod=gap, neck_vs_collar=0.118 * s / 0.118)

if __name__ == "__main__":
    SC = [1.0, 0.95, 0.9, 0.85, 0.8, 0.75, 0.7, 0.65]
    rows = {}
    for s in SC:
        T, H = tasks(s)
        rows[s] = dict(tasks={k: dict(ok=v["ok"], err_mm=round(1e3 * v["err"], 1), clear_mm=round(1e3 * v["clear"], 1),
                                      bow_deg=v.get("bow_deg"), q_deg=np.degrees(v["q"]).round(1).tolist(),
                                      pts=v["pts"].round(4).tolist(), hand=v["hand"].round(4).tolist()) for k, v in T.items()},
                       loads=loads(s), neck=neck_fit(s), top=float(H["oct_p"][2] + 0.224), hoof_x=float(H["hooves"][:2, 0].max()),
                       chest_x=float(H["sh"][:, :, 0].max()))
        t = rows[s]["tasks"]; L = rows[s]["loads"]; N = rows[s]["neck"]
        print(f"s {s:.2f} | hip {1e3*L['horse_height']:.0f} top {1e3*rows[s]['top']:.0f} | m {L['mass']:.1f} kg  peak x{L['peak']:.2f} "
              f"speed x{L['speed']:.2f} | sheath {'in' if N['sheath_inside'] else 'OUT'} gap {1e3*N['sheath_to_hip_pod']:.0f} | "
              + " ".join(f"{k.split()[0]}:{'Y' if v['ok'] else 'n'}({v['err_mm']:.0f}/{v['clear_mm']:.0f}{'' if v['bow_deg'] is None else ' b'+str(v['bow_deg'])})" for k, v in t.items()))
    json.dump({str(k): v for k, v in rows.items()}, open("arm_scale.json", "w"), default=float)
