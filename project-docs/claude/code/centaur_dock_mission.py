"""Centauri-1 mid-run docking: Cephalo-1 flies down onto Equo-1 while the horse runs, and they become one machine.
Numbers for the mission plan (est. where marked).  Horse at the 0.75 build (solo 18.5 kg), octopus unchanged (rev 4 arms,
Scabbard rotor whose blades are the docking bars, XL claw-props, 90 mm EDF).
  A  collar motion in each gait: heave from the gait study's load profiles, surge from its horizontal forces
  B  catch-up and speed match: octopus forward-flight state, closing profile, power
  C  capture budget: direct (bars as probes into the funnels) vs arm-assisted (two arms wrap the side grab posts first)
  D  arm grab envelope: IK with the real arm (joint limits), clear of the horse, from 300 mm above the seat down to 30 mm
  E  timeline, distance run, energy, load handover, aborts"""
import os, sys, io, json, contextlib
os.environ.setdefault("EQUO_S", "0.75"); os.environ["EQUO_SOLO"] = "0"
sys.path[:0] = ["/home/claude/octodrone", "/home/claude/octodrone/centaur", "/home/claude/octodrone/centaur/preview",
                "/home/claude/octodrone/centaur/legs_v2", "/home/claude/octodrone/centaur/arms_scale"]
import numpy as np
with contextlib.redirect_stdout(io.StringIO()):
    import legs_v2_study as LS
    import chassis_v3 as C3
    import arm_scale_study as AS
G = 9.81; S = C3.S
M_HORSE, M_OCT = 18.5, 12.81                   # build_preview solo mass; octopus (scabbard-dock.md)
SHARE_SOLO = 0.398                              # front share solo (build)
STRIDE, H, ZCOM = LS.STRIDE * S, LS.H * S, 0.40   # stride, hip height, solo CoM height (est.)
OUT = {}

# ------------------------------------------------------------------ A  collar motion
def periodic_integrate2(a, dt):
    """position from a periodic acceleration (mean removed), via FFT"""
    n = len(a); f = np.fft.rfftfreq(n, dt); A = np.fft.rfft(a - a.mean()); X = np.zeros_like(A)
    X[1:] = -A[1:] / (2 * np.pi * f[1:]) ** 2
    return np.fft.irfft(X, n)
def periodic_integrate1(a, dt):
    n = len(a); f = np.fft.rfftfreq(n, dt); A = np.fft.rfft(a - a.mean()); V = np.zeros_like(A)
    V[1:] = A[1:] / (2j * np.pi * f[1:])
    return np.fft.irfft(V, n)

def gait_motion(gait, v):
    duty = LS.GAITS[gait]["duty"]
    t_st = STRIDE / v; T = t_st / duty; n = 2400; t = np.linspace(0, T, n, endpoint=False); dt = T / n
    W = M_HORSE * G; Ff, Fr = SHARE_SOLO * W, (1 - SHARE_SOLO) * W
    # leg phases: trot diagonal pairs (LF+RH at 0, RF+LH at 0.5); walk lateral sequence LH 0, LF .25, RH .5, RF .75
    ph = {"trot": {"LF": 0, "RH": 0, "RF": .5, "LH": .5}, "walk": {"LH": 0, "LF": .25, "RH": .5, "RF": .75}}["walk" if gait == "walk" else "trot"]
    fz = np.zeros(n); fx = np.zeros(n); my = np.zeros(n)
    xh = {"F": C3.HIPF[0], "R": C3.HIPR[0]}
    for leg, p0 in ph.items():
        tau = ((t / T - p0) % 1.0)                     # 0 .. 1 over the cycle, stance first
        st = tau < duty; s = tau / duty
        if gait == "walk":
            lo, hi = (Ff if leg[0] == "L" and False else None), None
            F_stand, F_pair = (Ff / 2 if leg[1] == "F" else Fr / 2), (Ff if leg[1] == "F" else Fr)
            w = np.clip((s - 1 / 3) / 0.06 + 0.5, 0, 1) * np.clip((2 / 3 - s) / 0.06 + 0.5, 0, 1)
            f = np.where(st, F_stand + (LS.PK["walk"] * F_pair - F_stand) * w, 0.0)
        else:
            F_pair = Ff if leg[1] == "F" else Fr
            f = np.where(st, F_pair * (1 + (LS.PK["trot"] - 1) * np.cos(2 * np.pi * (s - 0.5))), 0.0)
        x_foot = np.where(st, STRIDE / 2 - STRIDE * s, 0.0)
        fz += f; fx += -f * x_foot / ZCOM
    fz *= W / fz.mean()                                 # the profiles are shapes; the mean must carry the weight
    az = fz / M_HORSE - G; ax = fx / M_HORSE
    zh = periodic_integrate2(az, dt); xs = periodic_integrate2(ax, dt); vz = periodic_integrate1(az, dt)
    return dict(T=T, f_step=1 / t_st if gait != "walk" else 4 / T, heave_pp=np.ptp(zh), heave_amp=np.ptp(zh) / 2,
                vz_peak=np.abs(vz).max(), surge_pp=np.ptp(xs), az_pp=np.ptp(az) / G, t=t, zh=zh, xs=xs)

GA = {"walk 0.25 m/s": ("walk", 0.25), "trot 0.5 m/s": ("trot", 0.5), "fast trot 0.8 m/s": ("fast trot", 0.8)}
OUT["collar_motion"] = {}
for name, (g, v) in GA.items():
    r = gait_motion(g, v)
    OUT["collar_motion"][name] = {k: round(float(r[k]) * (1e3 if k in ("heave_pp", "heave_amp", "surge_pp", "vz_peak") else 1), 2) for k in ("T", "f_step", "heave_pp", "heave_amp", "vz_peak", "surge_pp", "az_pp")}
    OUT["collar_motion"][name]["units"] = "T s, f_step Hz, heave and surge mm, vz_peak mm/s, az_pp g"
    print(f"A {name:18s} step {r['f_step']:.2f} Hz | heave {1e3*r['heave_pp']:.1f} mm p-p, peak vz {1e3*r['vz_peak']:.0f} mm/s | surge {1e3*r['surge_pp']:.1f} mm p-p | az {np.ptp((r['zh']*0)) :.0f}")
DOCK_GAIT = "fast trot 0.8 m/s"

# ------------------------------------------------------------------ C  capture budget (lateral / fore-aft, mm)
# capture of the first stage (bars at the 108 dock stroke as probes into the funnel mouths), centaur-neck.md sec. 3:
CAPTURE_MM = {"offset alone": 19.9, "with 2 deg yaw": 17.5, "with 2 deg yaw + 1 deg tilt": 14.0}
mot = OUT["collar_motion"][DOCK_GAIT]
budget = {
    "octopus station keeping, calm air (neck study)": 1.0,
    "octopus station keeping, light turbulence (neck study)": 14.0,
    "relative pose estimate (tip cameras on the grab posts' tags + horse state over radio, est.)": 3.0,
    "horse lateral sway in trot (legs hold the collar level; est.)": 5.0,
    "horse surge in the stride (A), not followed by the octopus at %.1f Hz" % mot["f_step"]: mot["surge_pp"] / 2,
}
calm = np.sqrt(sum(v ** 2 for k, v in budget.items() if "turbulence" not in k))
turb = np.sqrt(sum(v ** 2 for k, v in budget.items() if "calm" not in k))
ARM_HOLD = 2.0                                       # arms holding the posts: residual relative error (arm position loop, est.)
assisted = np.sqrt(ARM_HOLD ** 2 + 3.0 ** 2)          # + pose estimate of the funnels relative to the posts (fixed geometry)
OUT["capture"] = dict(capture_mm=CAPTURE_MM, budget_mm=budget, rss_calm=calm, rss_turbulence=turb, rss_arm_assisted=assisted,
                      margin_calm=CAPTURE_MM["with 2 deg yaw + 1 deg tilt"] / calm,
                      margin_turbulence=CAPTURE_MM["with 2 deg yaw + 1 deg tilt"] / turb,
                      margin_arm_assisted=CAPTURE_MM["with 2 deg yaw + 1 deg tilt"] / assisted)
print(f"C capture 14.0 mm (2 deg yaw, 1 deg tilt): direct calm x{OUT['capture']['margin_calm']:.2f}, direct turbulent x{OUT['capture']['margin_turbulence']:.2f}, arm-assisted x{OUT['capture']['margin_arm_assisted']:.2f}")

# ------------------------------------------------------------------ D  arm grab envelope
from scipy.optimize import least_squares
import arm_overlap as AO
TIP_L = 0.050
def solve_grab(H, az, target, q0=None):
    """arm IK to the grab post with joint limits, clear of the horse (shell, legs) AND of the octopus's own body
    (dome, plate, EDF can: cephalo4 body_clearance), >= 15 mm"""
    lo, hi = AS.LIM.T; bp, bR_ = H["oct_p"], H["oct_R"]
    def res(q):
        pts, hand, R = AS.arm_fk(az, q, bp, bR_)
        P, r = AS.capsules(pts)
        tip = np.linalg.norm(P - hand, axis=1) < TIP_L                 # the fiddlehead that closes on the post: may touch it
        need = np.where(tip, 0.002, AS.CLEAR)                         # skin: never penetrated, 2 mm; rest of arm 15 mm
        cl = np.minimum(AS.shell_dist(H, P), AS.leg_dist(H, P, 0.045 * S)) - r - need
        ab = w_up * 0.052; a0 = target - ab; tt = np.clip(((P - a0) @ ab) / (ab @ ab), 0, 1)
        dpost = np.linalg.norm(a0 + tt[:, None] * ab - P, axis=1) - 0.014 - r - 0.005
        cl = np.minimum(cl, np.where(tip, 1.0, dpost))                # the rest of the arm keeps 5 mm off post + cap
        cb = AO.body_d(P, bp, bR_) - r - 0.015
        return np.concatenate([(hand - target) * 10, np.minimum(cl, 0) * 20, np.minimum(cb, 0) * 20, 0.01 * q])
    best = None
    starts = ([q0] if q0 is not None else []) + [np.radians(v) for v in ([20, 0, 60, 0, 40, 0, 20, 0], [-20, 0, 80, 0, 60, 0, 0, 0],
                                                                         [10, 30, 70, 0, 50, 0, 20, 0], [0, 0, 100, 0, 80, 0, 40, 0], [-40, 0, 100, 0, 100, 0, 60, 0])]
    for st in starts:
        sol = least_squares(res, np.clip(st, lo + 1e-6, hi - 1e-6), bounds=(lo, hi), max_nfev=400)
        rr_ = res(sol.x); err = np.linalg.norm(rr_[:3]) / 10; viol = -np.min(rr_[3:-8]) / 20 if len(rr_) > 11 else 0
        sc_ = err + 10 * max(viol, 0)
        if best is None or sc_ < best[0]: best = (sc_, sol.x, err, viol)
    pts, hand, R = AS.arm_fk(az, best[1], bp, bR_); P, r = AS.capsules(pts)
    nt = np.linalg.norm(P - hand, axis=1) >= TIP_L
    clear = min(float((AS.shell_dist(H, P[nt]) - r[nt]).min()), float((AS.leg_dist(H, P, 0.045 * S) - r).min()))
    tip_skin = float((AS.shell_dist(H, P[~nt]) - r[~nt]).min())
    return dict(q=best[1], err=best[2], clear=clear, body=AO.body_clear((P, r), bp, bR_), pts=pts, hand=hand, tip_skin=tip_skin,
                ok=bool(best[2] < 0.010 and tip_skin >= 0.0 and clear >= AS.CLEAR - 0.002 and AO.body_clear((P, r), bp, bR_) >= 0.013))
Hs = AS.horse(S); bp0, bR = Hs["oct_p"], Hs["oct_R"]; w_up = bR @ np.array([0, 0, 1.0])
cc3 = np.array([C3.cc[0], 0, C3.cc[1]])
POSTS = {+1: cc3 + np.array([0, 1.0, 0]) * 0.141 + w_up * 0.052, -1: cc3 - np.array([0, 1.0, 0]) * 0.141 + w_up * 0.052}
AS.LEAD = -0.020                                      # grip point: under the tip bay, where the fiddlehead's spiral core sits
grid = []; SEED = {}
for h in (0.30, 0.20, 0.103):
    for dx in (-0.04, 0.0, 0.04):
        for dy in (-0.04, 0.0, 0.04):
            H2 = dict(Hs); H2["oct_p"] = bp0 + w_up * h + np.array([dx, dy, 0])
            ok = True; worst = 9.0
            for side, az in ((+1, 67.5), (-1, -67.5)):
                r = solve_grab(H2, az, POSTS[side], SEED.get(az)); SEED[az] = r["q"] if r["ok"] else SEED.get(az)
                ok &= r["ok"]; worst = min(worst, r["clear"])
            grid.append(dict(h=h, dx=dx, dy=dy, ok=bool(ok), clear_mm=round(1e3 * worst, 1)))
    print(f"D h {1e3*h:4.0f} mm: both posts reached in {sum(g['ok'] for g in grid if g['h'] == h)}/9 offsets (+-40 mm), min clearance {min(g['clear_mm'] for g in grid if g['h'] == h):.0f} mm")
OUT["arm_grab"] = grid
AS.LEAD = 0.060

# ------------------------------------------------------------------ B + E  timeline
V = 0.80                                              # dock gait: fast trot (smallest heave, A)
ph = [("approach: octopus closes from 2 m behind, 1.5 m up; rotor flight", 6.0),
      ("speed match + station keep 300 mm over the seat (tip cameras lock on the post tags)", 2.0),
      ("rotor spin-down and hub park; flight on 8 claw-props + EDF; bars out to the 108 dock stroke", 2.5),
      ("hood swings forward 130 deg onto its stop (STS3215, 0.6 s); key shaft to 80 deg (armed)", 0.6),
      ("arms 67.5 / -67.5 deg wrap the side grab posts; their props fold (6 props + EDF fly)", 1.0),
      ("arms reel down 300 -> 103 mm at 100 mm/s (bar tips enter the funnels)", 2.0),
      ("at 103 mm the two arms let go and swing to the guard pose (props stay folded): lower down the post cap sits too close under the arm roots to hold", 0.4),
      ("103 -> 0 mm at 40 mm/s: sheaths steer, horns enter the guest sockets, pins snap, plate on the pads", 2.6),
      ("thrust ramps 126 N -> 0 over 1 s: weight onto the horse's front legs", 1.0),
      ("bars 108 -> 194 mm (elevator, 24 mm/s); key to 45 deg: bolts fire through the eyelets (Hall switches confirm)", 3.6),
      ("key to 0 deg: wedges cinch; props stop; gait controller switches to 30.9 kg", 1.0)]
t = 0.0; rows = []
for name, d in ph:
    rows.append(dict(phase=name, t0=round(t, 1), dur=d, dist_m=round(V * d, 2))); t += d
OUT["timeline"] = dict(phases=rows, total_s=round(t, 1), distance_m=round(V * t, 1), speed=V)
P_ROTOR, P_PROPS = 2250.0, 2540.0                     # W, hover (scabbard-dock.md, est.)
e_rotor = P_ROTOR * sum(d for n_, d in ph[:2]) / 3600; e_props = P_PROPS * sum(d for n_, d in ph[2:9]) / 3600 * 0.9
OUT["energy_Wh"] = dict(rotor=round(e_rotor, 1), props=round(e_props, 1), total=round(e_rotor + e_props, 1))
# load handover on the horse
OUT["handover"] = dict(front_share_before=SHARE_SOLO, front_share_after=0.609, load_rate_N_per_s=round(M_OCT * G / 1.0, 0),
                       impact_v_mm_s=round(float(np.hypot(40.0, OUT["collar_motion"][DOCK_GAIT]["vz_peak"])), 0),
                       impact_energy_J=round(0.5 * M_OCT * (np.hypot(0.040, OUT["collar_motion"][DOCK_GAIT]["vz_peak"] / 1e3)) ** 2, 3))
print(f"E timeline {t:.1f} s, {V*t:.1f} m at {V} m/s; energy {OUT['energy_Wh']}; seat at {OUT['handover']['impact_v_mm_s']:.0f} mm/s ({OUT['handover']['impact_energy_J']} J)")
json.dump(OUT, open("dock_mission.json", "w"), indent=1, default=float)

# ------------------------------------------------------------------ D2  nothing overlaps during the grab: gripping arms vs the
# six flying arms (guard pose, props running), prop discs, octopus body, horse
import arm_overlap as AO
Q_GUARD = np.radians([-35, 0, -45, 0, 0, 0, 0, 0])
POSES = {}
D2 = []
def post_clear(g):
    P, r = g[0], g[1]; out = 9.0
    for c in POSTS.values():
        ab = w_up * 0.052; a0 = c - ab; tt = np.clip(((P - a0) @ ab) / (ab @ ab), 0, 1)
        out = min(out, float((np.linalg.norm(a0 + tt[:, None] * ab - P, axis=1) - 0.014 - r).min()))
    return out
for h in (0.30, 0.20, 0.103, 0.06, 0.03, 0.0):
    H2 = dict(Hs); H2["oct_p"] = bp0 + w_up * h; bp = H2["oct_p"]
    if h >= 0.103 - 1e-9:
        AS.LEAD = -0.020
        grab = {az: solve_grab(H2, az, POSTS[1 if az > 0 else -1]) for az in (67.5, -67.5)}
        AS.LEAD = 0.060
    else:
        grab = {}                                     # released: all eight arms in the guard pose
    geos, props = [], []
    for i in range(8):
        az = 22.5 + 45 * i; azn = ((az + 180) % 360) - 180
        if abs(abs(azn) - 67.5) < 1e-6 and grab:
            geos.append(AO.arm_geo(azn, grab[azn]["q"], bp, bR))
        else:
            geos.append(AO.arm_geo(azn, Q_GUARD, bp, bR))
            if abs(abs(azn) - 67.5) > 1e-6: props.append(len(geos) - 1)          # the released pair keeps its props folded
    wa, wd, wb = AO.report(geos, bp, bR, props)
    flying = [k for k in range(8) if k in props or not grab]
    wh = min(float((AS.shell_dist(H2, g[0]) - g[1]).min()) for k, g in enumerate(geos) if k in flying)
    wp = min(post_clear(geos[k]) for k in flying)
    D2.append(dict(h=h, gripping=bool(grab), grab_ok=bool(all(g["ok"] for g in grab.values())), free_arms_to_posts=round(1e3 * wp, 1), arm_arm=round(1e3 * wa, 1), disc_disc=round(1e3 * wd, 1), arm_body=round(1e3 * wb, 1), flying_arms_to_horse=round(1e3 * wh, 1),
                   grab_clear=round(1e3 * min([g["clear"] for g in grab.values()] or [9]), 1)))
    POSES[h] = {az: np.asarray(g["q"]).tolist() for az, g in grab.items()}
    print(f"D2 h {1e3*h:4.0f}: {'grip' if grab else 'free'} ok {D2[-1]['grab_ok']}, free arms-posts {1e3*wp:5.1f}, arm-arm {1e3*wa:5.1f} mm, discs {1e3*wd:5.1f}, arms-octopus body {1e3*wb:5.1f}, flying arms-horse {1e3*wh:5.1f}, grab arms-horse {D2[-1]['grab_clear']:5.1f}")
OUT["overlap_during_grab"] = D2; OUT["grab_poses"] = {str(k): v for k, v in POSES.items()}
json.dump(OUT, open("dock_mission.json", "w"), indent=1, default=float)
