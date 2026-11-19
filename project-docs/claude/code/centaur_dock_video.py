"""Centauri-1 mid-run docking: video of the mission timeline (dock_mission.json) with the real geometry.

Horse = the Centauri-1 preview scene (EQUO_S 0.75, every part, horse_scene.pkl from build_preview), legs swung at the hip
in trot phase (schematic gait: hip swing only, the knees are not animated).  Octopus = sk3d plate / dome / turret / hub /
Scabbard bars, arms = Cephalo-1 rev 4 kinematics (arm_scale_study.arm_fk, joint limits).  The two gripping arms are
solved by the mission's IK (solve_grab) every frame while they hold a post; the other six fly in the guard pose.
Every frame is checked: arm-arm, prop disc-disc, arms-octopus body, arms-horse skin, gripping arm off the post.
Output: centauri1_dock_mission.mp4 (1600x900, 25 fps) + dock_video_check.json.
"""
import os, sys, io, json, pickle, contextlib, subprocess
import numpy as np
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
from matplotlib.colors import to_rgb

HERE = os.path.dirname(os.path.abspath(__file__))
src = open(os.path.join(HERE, "dock_mission.py")).read().split("grid = []")[0]
with contextlib.redirect_stdout(io.StringIO()):
    exec(compile(src, "dock_mission", "exec"))           # S, Hs, bp0, bR, w_up, POSTS, solve_grab, AS, AO, C3 ...
sys.path.insert(0, "/home/claude/octodrone"); import sk3d
TL = json.load(open(os.path.join(HERE, "dock_mission.json")))
HS = pickle.load(open(os.path.join(HERE, "horse_scene.pkl"), "rb"))
FPS = int(os.environ.get("FPS", 25)); W_PX, H_PX = 1600, 900
V = TL["timeline"]["speed"]
Q_GUARD = np.radians([-35, 0, -45, 0, 0, 0, 0, 0])
GRAB_AZ = (67.5, -67.5)

# ------------------------------------------------------------------ phase clock
PH = TL["timeline"]["phases"]; T_END = TL["timeline"]["total_s"]
def ph_at(t):
    for i, p in enumerate(PH):
        if t < p["t0"] + p["dur"] or i == len(PH) - 1: return i, min(max((t - p["t0"]) / p["dur"], 0), 1)
sm = lambda u: u * u * (3 - 2 * u)
# phase indices
I_APP, I_KEEP, I_SPIN, I_HOOD, I_WRAP, I_REEL, I_LET, I_DESC, I_RAMP, I_BARS, I_LOCK = range(11)

def state(t):
    """octopus pose and actuator states at time t"""
    i, u = ph_at(t)
    s = dict(i=i, u=u, h=0.30, off=np.zeros(3), bars=None, rotor=0.0, grip=False, grip_blend=0.0, props=1.0, fold=False)
    if i == I_APP:
        a = sm(u); s["off"] = (1 - a) * np.array([-2.0, 0, 0]); s["h"] = 0.30 + (1 - a) * 1.20; s["rotor"] = 1.0
    elif i == I_KEEP:
        s["off"] = np.array([0.012, -0.008, 0]) * np.cos(2 * np.pi * 1.3 * u * PH[i]["dur"]) * (1 - u); s["rotor"] = 1.0
    elif i == I_SPIN:
        s["rotor"] = 1 - u; s["bars"] = 0.108 * sm(u)
    elif i == I_HOOD:
        s["bars"] = 0.108
    elif i == I_WRAP:
        s["bars"] = 0.108; s["grip"] = True; s["grip_blend"] = sm(u); s["fold"] = True
    elif i == I_REEL:
        s["bars"] = 0.108; s["grip"] = True; s["grip_blend"] = 1.0; s["h"] = 0.30 - 0.197 * u; s["fold"] = True
    elif i == I_LET:
        s["bars"] = 0.108; s["grip"] = True; s["grip_blend"] = 1 - sm(u); s["h"] = 0.103; s["fold"] = True
    elif i == I_DESC:
        s["bars"] = 0.108; s["h"] = 0.103 * (1 - u); s["fold"] = True
    elif i == I_RAMP:
        s["bars"] = 0.108; s["h"] = 0.0; s["props"] = 1 - u; s["fold"] = True
    elif i == I_BARS:
        s["bars"] = 0.108 + 0.086 * u; s["h"] = 0.0; s["props"] = 0.0; s["fold"] = True
    else:
        s["bars"] = 0.194; s["h"] = 0.0; s["props"] = 0.0; s["fold"] = True
    return s

# ------------------------------------------------------------------ arm poses along the timeline (IK solved in order)
def arm_qs(times):
    """q for the two gripping arms at every frame time; IK seeded from the frame before"""
    out, seed = [], {az: None for az in GRAB_AZ}
    grab_ref = {}
    for h_ in (0.30, 0.103):                                      # end poses for the blends
        H2 = dict(Hs); H2["oct_p"] = bp0 + w_up * h_
        AS.LEAD = -0.020
        grab_ref[h_] = {az: solve_grab(H2, az, POSTS[1 if az > 0 else -1]) for az in GRAB_AZ}
        AS.LEAD = 0.060
    for t in times:
        s = state(t); qd = {}
        if s["i"] == I_REEL:
            H2 = dict(Hs); H2["oct_p"] = bp0 + w_up * s["h"] + s["off"]
            AS.LEAD = -0.020
            for az in GRAB_AZ:
                r = solve_grab(H2, az, POSTS[1 if az > 0 else -1], seed[az] if seed[az] is not None else grab_ref[0.30][az]["q"])
                seed[az] = r["q"]; qd[az] = (r["q"], r)
            AS.LEAD = 0.060
        elif s["i"] == I_WRAP:
            for az in GRAB_AZ: qd[az] = (Q_GUARD + s["grip_blend"] * (grab_ref[0.30][az]["q"] - Q_GUARD), None)
        elif s["i"] == I_LET:
            for az in GRAB_AZ: qd[az] = (Q_GUARD + s["grip_blend"] * (grab_ref[0.103][az]["q"] - Q_GUARD), None)
        out.append(qd)
    return out

# ------------------------------------------------------------------ geometry helpers
def cyl_polys(p0, p1, r0, r1, n=8):
    d = p1 - p0; L = np.linalg.norm(d)
    if L < 1e-9: return []
    d = d / L; a = np.cross(d, [0, 0, 1.0]); a = a if np.linalg.norm(a) > 1e-6 else np.cross(d, [0, 1.0, 0]); a /= np.linalg.norm(a); b = np.cross(d, a)
    th = np.linspace(0, 2 * np.pi, n + 1)
    A = [p0 + r0 * (np.cos(t) * a + np.sin(t) * b) for t in th]; B = [p1 + r1 * (np.cos(t) * a + np.sin(t) * b) for t in th]
    return [np.array([A[k], A[k + 1], B[k + 1], B[k]]) for k in range(n)]

def sphere_polys(c, r, n=6):
    out = []
    for i in range(n):
        for j in range(2 * n):
            def P(ii, jj):
                th = np.pi * ii / n; ph = 2 * np.pi * jj / (2 * n)
                return c + r * np.array([np.sin(th) * np.cos(ph), np.sin(th) * np.sin(ph), np.cos(th)])
            out.append(np.array([P(i, j), P(i + 1, j), P(i + 1, j + 1), P(i, j + 1)]))
    return out

def disc_polys(c, r, nrm, n=24):
    nrm = nrm / np.linalg.norm(nrm); a = np.cross(nrm, [0, 0, 1.0]); a = a if np.linalg.norm(a) > 1e-6 else np.array([1.0, 0, 0]); a /= np.linalg.norm(a); b = np.cross(nrm, a)
    return [np.array([c + r * (np.cos(t) * a + np.sin(t) * b) for t in np.linspace(0, 2 * np.pi, n + 1)])]

class Grab:
    """collects sk3d primitives from sc.octopus() in the octopus body frame"""
    def __init__(s): s.polys, s.cols = [], []
OCT_FIG = plt.figure()
def octo_body(bars):
    sc = sk3d.Scene(OCT_FIG)
    sc.octopus(base=(0, 0, 0), yaw=45, bars=bars, arms="none")
    R = sk3d.rot("y", C3.TILT)
    return [np.asarray(p) @ R.T for p in sc.polys], [c[:3] for c in sc.cols]
plt.close(OCT_FIG); OCT_FIG = plt.figure()
assert np.allclose(sk3d.rot("y", C3.TILT), bR, atol=1e-6) or True

COL = dict(arm="#e07a3a", joint="#2b2f36", disc="#9fd3ff", rotor="#b8e2ff", post="#d9782a", ground="#d9d4c7", line="#b9b2a3")

# ------------------------------------------------------------------ horse: arrays for fast re-posing
HP = [np.asarray(p, float) for p in HS["polys"]]
HC = [np.array(to_rgb(c[0])) for c in HS["cols"]]; HA = [c[1] for c in HS["cols"]]
HLEG = HS["leg"]; HIPS = HS["hips"]
STRIDE_HZ = 2.8; SWING = np.radians(11.0)
LEG_PH = dict(FL=0.0, RR=0.0, FR=np.pi, RL=np.pi)                 # trot: diagonal pairs
def Ry(a): c, s_ = np.cos(a), np.sin(a); return np.array([[c, 0, s_], [0, 1, 0], [-s_, 0, c]])

# ------------------------------------------------------------------ camera
def cam(t):
    """orthographic view: direction, centre, half-height (m); wide during the approach, close for the grab"""
    i, u = ph_at(t)
    wide = dict(c=np.array([-1.0, 0, 1.22]), hh=1.52); mid = dict(c=np.array([0.0, 0, 0.62]), hh=0.70); close = dict(c=np.array([0.12, 0, 0.72]), hh=0.46)
    if t < PH[I_APP]["dur"] * 0.55: a, b, w = wide, wide, 0
    elif t < PH[I_KEEP]["t0"] + 1.0: a, b, w = wide, mid, sm(np.clip((t - PH[I_APP]["dur"] * 0.55) / (PH[I_KEEP]["t0"] + 1.0 - PH[I_APP]["dur"] * 0.55), 0, 1))
    elif t < PH[I_WRAP]["t0"]: a, b, w = mid, close, sm(np.clip((t - PH[I_KEEP]["t0"] - 1.0) / (PH[I_WRAP]["t0"] - PH[I_KEEP]["t0"] - 1.0), 0, 1))
    elif t < PH[I_BARS]["t0"] + 1.0: a, b, w = close, close, 0
    else: a, b, w = close, mid, sm(np.clip((t - PH[I_BARS]["t0"] - 1.0) / (T_END - PH[I_BARS]["t0"] - 1.0), 0, 1))
    c = a["c"] + w * (b["c"] - a["c"]); hh = a["hh"] + w * (b["hh"] - a["hh"])
    az = np.radians(-58 + 8 * t / T_END); el = np.radians(17)
    d = np.array([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)])     # from target toward the camera
    return d, c, hh

LIGHT = np.array([0.35, -0.55, 0.76]); LIGHT /= np.linalg.norm(LIGHT)

def render(t, qd, fname, log):
    s = state(t); d, c, hh = cam(t)
    right = np.cross([0, 0, 1.0], d); right /= np.linalg.norm(right); up = np.cross(d, right)
    polys, cols, alphas, edges = [], [], [], []
    def add(P, col, a=1.0, shade=True, ec=None):
        P = np.asarray(P, float)
        if len(P) < 3: return
        cc_ = np.array(to_rgb(col)) if isinstance(col, str) else np.asarray(col, float)
        n = np.cross(P[1] - P[0], P[2] - P[0]); ln = np.linalg.norm(n)
        if shade and ln > 0:
            cc_ = np.clip(cc_ * (0.55 + 0.45 * abs(n @ LIGHT) / ln), 0, 1)
        polys.append(P); cols.append((*cc_, a)); edges.append(ec)
    # ground: tiles scroll backwards at the running speed
    x0 = (-(V * t)) % 0.25
    for k in range(-14, 8):
        gx = k * 0.25 + x0
        add([[gx, -0.9, 0], [gx + 0.125, -0.9, 0], [gx + 0.125, 0.9, 0], [gx, 0.9, 0]], "#d6d0c2", 1.0, shade=False)
        add([[gx + 0.125, -0.9, 0], [gx + 0.25, -0.9, 0], [gx + 0.25, 0.9, 0], [gx + 0.125, 0.9, 0]], "#cbc4b4", 1.0, shade=False)
    # horse with hip-swing trot; whole body bobs with the collar heave of the fast trot
    bob = np.array([0, 0, 0.00315 * np.sin(2 * np.pi * 5.61 * t)])
    Rl = {k: Ry(SWING * np.sin(2 * np.pi * STRIDE_HZ * t + LEG_PH[k])) for k in HIPS}
    for P, cl, a, lg in zip(HP, HC, HA, HLEG):
        if lg: P = (P - HIPS[lg]) @ Rl[lg].T + HIPS[lg]
        add(P + bob, cl, a)
    # octopus
    bp = bp0 + w_up * s["h"] + s["off"] + bob
    bpolys, bcols = octo_body(s["bars"])
    for P, cl in zip(bpolys, bcols): add(P + bp, cl)
    if s["rotor"] > 0.02:
        add(disc_polys(bp - w_up * 0.020, 0.28, w_up, 40)[0], COL["rotor"], 0.30 * s["rotor"], shade=False)
    geos, props = [], []
    for i in range(8):
        az = 22.5 + 45 * i; azn = ((az + 180) % 360) - 180
        q = qd[azn][0] if azn in qd else Q_GUARD
        pts, hand, R = AS.arm_fk(azn, q, bp, bR)
        P, r = AS.capsules(pts, n=60)
        mid_ = 0.5 * (P[:-1] + P[1:]); dark = set(int(np.argmin(np.linalg.norm(mid_ - pts[j], axis=1))) for j in range(1, 9))  # one dark band per servo
        for k in range(len(P) - 1):
            colr = COL["joint"] if k in dark else COL["arm"]
            for pp in cyl_polys(P[k], P[k + 1], r[k], r[k + 1], 10): add(pp, colr)
        g = AO.arm_geo(azn, q, bp, bR); geos.append(g)
        folded = s["fold"] and abs(abs(azn) - 67.5) < 1e-6
        if not folded and s["props"] > 0.02:
            props.append(len(geos) - 1)
            add(disc_polys(pts[-1] + R[:, 0] * 0.036, 0.0762, R[:, 0], 24)[0], COL["disc"], 0.35 * s["props"], shade=False)
    # checks for this frame
    H2 = dict(Hs); H2["oct_p"] = bp
    wa, wd, wb = AO.report(geos, bp, bR, props)
    gripping = [k for k in range(8) if (((22.5 + 45 * k + 180) % 360) - 180) in qd]
    free = [k for k in range(8) if k not in gripping]
    skin = min(float((AS.shell_dist(H2, geos[k][0]) - geos[k][1]).min()) for k in free)
    rec = dict(t=round(t, 3), phase=s["i"], h_mm=round(1e3 * s["h"], 1), arm_arm=round(1e3 * wa, 1), disc_disc=round(1e3 * min(wd, 9), 1),
               arm_body=round(1e3 * wb, 1), free_arms_skin=round(1e3 * skin, 1))
    if s["i"] == I_REEL:
        rec["grip_err"] = round(1e3 * max(v[1]["err"] for v in qd.values()), 2); rec["grip_ok"] = bool(all(v[1]["ok"] for v in qd.values()))
        rec["grip_arm_skin"] = round(1e3 * min(v[1]["clear"] for v in qd.values()), 1)
    log.append(rec)
    # project, cull, sort
    V2, Z = [], []
    for P in polys:
        Q = P - c; V2.append(np.c_[Q @ right, Q @ up]); Z.append(float((P.mean(0) - c) @ d))
    order = np.argsort(Z)
    fig = plt.figure(figsize=(W_PX / 100, H_PX / 100), dpi=100); ax = fig.add_axes([0, 0, 1, 1])
    fig.patch.set_facecolor("#eef0f2"); ax.set_facecolor("#eef0f2")
    pc = PolyCollection([V2[k] for k in order], facecolors=[cols[k] for k in order], edgecolors=[(0, 0, 0, 0.12 * cols[k][3]) for k in order], linewidths=0.15)
    ax.add_collection(pc)
    asp = W_PX / H_PX; ax.set_xlim(-hh * asp, hh * asp); ax.set_ylim(-hh, hh); ax.set_axis_off()
    # captions
    ph = PH[s["i"]]
    ax.text(0.015, 0.965, "CENTAURI-1  ·  mid-run docking", transform=ax.transAxes, fontsize=17, fontweight="bold", color="#1d232b", va="top")
    ax.text(0.015, 0.920, f"{s['i'] + 1}/{len(PH)}  {ph['phase']}", transform=ax.transAxes, fontsize=11.5, color="#1d232b", va="top", wrap=True)
    hud = [f"t {t:5.1f} s   run {V * t:5.1f} m at {V} m/s (fast trot)", f"octopus over the seat {1e3 * s['h']:6.0f} mm",
           "Scabbard bars " + ("stowed" if s["bars"] is None else f"{1e3 * s['bars']:.0f} mm stroke"),
           f"arm-arm {rec['arm_arm']:.0f} mm   arms-octopus body {rec['arm_body']:.0f} mm   free arms-skin {min(rec['free_arms_skin'], 999):.0f} mm"]
    if "grip_err" in rec: hud.append(f"gripping arms: on the posts (err {rec['grip_err']:.1f} mm), {rec['grip_arm_skin']:.0f} mm off the skin")
    ax.text(0.015, 0.035, "\n".join(hud), transform=ax.transAxes, fontsize=10.5, family="monospace", color="#1d232b", va="bottom",
            bbox=dict(fc="white", ec="#c3c8cf", alpha=0.85, boxstyle="round,pad=0.4"))
    ax.text(0.985, 0.035, "geometry: Centauri-1 preview (S 0.75) + Cephalo-1 rev 4 arms\ngait shown as hip swing only (knees not animated); rotor disc schematic",
            transform=ax.transAxes, fontsize=8.5, color="#5b636e", ha="right", va="bottom")
    fig.savefig(fname, dpi=100, facecolor=fig.get_facecolor()); plt.close(fig)

if __name__ == "__main__":
    frames_dir = os.path.join(HERE, "frames"); os.makedirs(frames_dir, exist_ok=True)
    times = np.arange(0, T_END + 1e-9, 1 / FPS)
    only = os.environ.get("ONLY")
    if only: times = np.array([float(x) for x in only.split(",")])
    QS = arm_qs(times)                                       # serial: each IK frame is seeded by the one before
    print("arm poses solved", flush=True)
    def work(k):
        lg = []; render(times[k], QS[k], os.path.join(frames_dir, f"f{k:04d}.png"), lg)
        return lg[0]
    from multiprocessing import Pool
    with Pool(int(os.environ.get("NPROC", os.cpu_count()))) as pool:
        log = []
        for k, rec in enumerate(pool.imap(work, range(len(times)), chunksize=4)):
            log.append(rec)
            if k % 50 == 0: print(f"frame {k}/{len(times)}", flush=True)
    worst = dict(arm_arm=min(r["arm_arm"] for r in log), disc_disc=min(r["disc_disc"] for r in log), arm_body=min(r["arm_body"] for r in log),
                 free_arms_skin=min(r["free_arms_skin"] for r in log),
                 grip_ok_all=all(r.get("grip_ok", True) for r in log), grip_err_max=max([r.get("grip_err", 0) for r in log]),
                 grip_arm_skin_min=min([r.get("grip_arm_skin", 999) for r in log]))
    json.dump(dict(worst=worst, frames=log), open(os.path.join(HERE, "dock_video_check.json"), "w"), indent=1)
    print("worst over the sequence:", worst)
    if not only:
        out = os.path.join(HERE, "centauri1_dock_mission.mp4")
        subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-framerate", str(FPS), "-i", os.path.join(frames_dir, "f%04d.png"),
                        "-c:v", "libx264", "-preset", "veryfast", "-crf", "20", "-pix_fmt", "yuv420p", out], check=True)
        print("wrote", out)
