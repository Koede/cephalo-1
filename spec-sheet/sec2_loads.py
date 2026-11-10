"""Section 3: torque demand. Hover and flight poses (thrust along the tip axis), walking stance on the folded pod,
hold-out reach and payload vs reach, random-pose statistics. -> results/loads.json (+ poses.npz)"""
import sys, os, json, math, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, "/mnt/project-files/octopus-drone/joint-study/sim")
os.environ.setdefault("JOINT_SCENARIO", "more_thrust")
import numpy as np
import spec_lib as S, poses as PS
import claw_fans as CF, central_prop as CP

ARRS = [sys.argv[1]] if len(sys.argv) > 1 else ["alt90"]
SECS = sys.argv[2].split(",") if len(sys.argv) > 2 else ["flight", "stance", "holdout", "random"]
FAN = CF.FANS["MN5008 KV400 + 16x5.4 CF"]; FULL = CF.fan_full(FAN); RATED = 0.75 * FULL
ROBOT = 18.95 + 8 * S.WIRE_G_PER_ARM / 1000          # DESIGN.md robot + the claw-fan wiring it did not yet carry
Pw, Tc, FAN_HOVER = CF.hover(ROBOT, FAN, RATED)
FOOT = ROBOT * S.G / 7
out = dict(robot_kg=round(ROBOT, 2), fan=dict(hover_N=round(FAN_HOVER, 2), rated_N=round(RATED, 2), full_N=round(FULL, 2), central_N=round(Tc, 1),
           hover_total_W=round(Pw)), foot_N=round(FOOT, 1))
store = {}
STANCE_R = float(os.environ.get('STANCE_R', 0.30))   # foot radius from the hub axis; 0.25 m (DESIGN) is out of reach without the helix
out['stance_foot_radius_m'] = STANCE_R

def flight(A, T, tilt_dir=None, tilt_deg=0.0, r_tip=0.60, warm=None, n=25):
    d = UPd = np.array([0, 0, 1.0])
    if tilt_dir is not None:
        t = math.radians(tilt_deg); d = math.cos(t) * UPd + math.sin(t) * np.asarray(tilt_dir, float)
    load = lambda q, f: (T * f["tipR"][:, 2], "thrust", None)
    def cons(q, f):
        z = f["tipR"][:, 2]; tip = f["tip"]
        iq = [float(z @ d) - math.cos(math.radians(3)), float(np.hypot(tip[0], tip[1])) - r_tip, S.self_clear(f)]
        return [], iq
    base = np.zeros(S.N); base[0] = 0.5; base[2::2] = 0.13
    rng = np.random.default_rng(11); x0s = [base] + ([warm] if warm is not None else []) + [rng.uniform(-0.8, 0.8, S.N) * PS.LIM for _ in range(n)]
    return PS.solve(A, load, cons, x0s)

def stance(A, foot, side, Ffoot):
    def load(q, f):
        return np.zeros(3), "tip", [(PS.foot_point(A, f, side), np.array([0, 0, Ffoot]))]
    def cons(q, f):
        fp = PS.foot_point(A, f, side); z = f["tipR"][:, 2]
        eq = list(fp - foot)
        iq = [float(z @ PS.DOWN) - math.cos(math.radians(10)), S.self_clear(f)] + list(f["marks"][1:-1, 2] - (foot[2] + S.R_OUT + 0.015)) + PS.azimuth_cons(f)
        return eq, iq
    base = np.zeros(S.N); base[0] = -0.9; base[2::2] = 0.55; base[2] = 0.3
    return PS.solve(A, load, cons, PS.seeds(10, base=base))

def holdout(A, reach, payload_kg=0.0, cap_scale=1.0):
    load = lambda q, f: (np.array([0, 0, -payload_kg * S.G]), "tip", None)
    def cons(q, f):
        tip = f["tip"] - A.base
        return [tip[0] - reach, tip[2], tip[1]], [S.self_clear(f)]
    return PS.solve(A, load, cons, PS.seeds(4, base=np.zeros(S.N)), cap=S.CAP * cap_scale)

for arr in ARRS:
  t0 = time.time(); A = S.Arm(arr); res = {}
  if "flight" in SECS:
    fl = {}; warm = None
    for nm, T in (("hover", FAN_HOVER), ("rated", RATED), ("full", FULL)):
        r = flight(A, T, warm=warm, n=(30 if nm == "hover" else 12)); warm = r["q"] if nm == "hover" else warm; fl[nm] = dict(T_N=round(T, 1), ok=r["ok"], worst=round(r["worst"], 3), need=[round(float(x), 2) for x in r["need"]],
                                        q_deg=[round(math.degrees(x), 1) for x in r["q"]]); store[f"{arr}_flight_{nm}"] = r["q"]
    tilts = {}
    for tn, td in (("out", [1, 0, 0]), ("in", [-1, 0, 0]), ("side", [0, 1, 0])):
        r = flight(A, RATED, td, 30.0, warm=warm, n=12); tilts[tn] = dict(ok=r["ok"], worst=round(r["worst"], 3), need=[round(float(x), 2) for x in r["need"]]); store[f"{arr}_tilt_{tn}"] = r["q"]
    fl["rated_tilted_30"] = tilts
    res["flight"] = fl
  if "stance" in SECS:
    # ---- walking stance on the folded pod: four foot points of the study's step, four pod clockings
    RF = STANCE_R
    feet = {"mid": [RF, 0, -0.25], "side -45": [RF, -0.045, -0.25], "side +45": [RF, 0.045, -0.25], "in 34": [RF - 0.034, 0, -0.25]}
    sides = {"+x": [1, 0, 0], "-x": [-1, 0, 0], "+y": [0, 1, 0], "-y": [0, -1, 0]}
    st = {}
    for sn, sd in sides.items():
        rows = {}
        for fn, fp in feet.items():
            r = stance(A, np.array(fp), sd, FOOT)
            r2n = A.need(r["q"], np.zeros(3), Fpt="tip", extra=[(PS.foot_point(A, A.fk(r["q"]), sd), np.array([0, 0, 2 * FOOT]))])[0]
            rows[fn] = dict(ok=r["ok"], worst=round(r["worst"], 3), worst_2x=round(float(r2n.max()), 3), need=[round(float(x), 2) for x in r["need"]],
                            q_deg=[round(math.degrees(x), 1) for x in r["q"]])
            store[f"{arr}_stance_{sn}_{fn}"] = r["q"]
        st[sn] = dict(rows=rows, worst=max(v["worst"] for v in rows.values()), worst_2x=max(v["worst_2x"] for v in rows.values()),
                      all_ok=all(v["ok"] for v in rows.values()))
    res["stance"] = st
  if "holdout" in SECS:
    # ---- hold-out: worst need holding the tip level at a reach, fans off, and payload at 2x / stall
    ho = []
    for reach in (0.3, 0.5, 0.6, 0.7, 0.8, 0.9, 0.95):
        r0 = holdout(A, reach)
        row = dict(reach_m=reach, ok=r0["ok"], worst_no_payload=round(r0["worst"], 3))
        for lab, lim in (("payload_kg_2x", 0.5), ("payload_kg_stall", 1.0)):
            lo, hi = 0.0, 3.0
            if not (r0["ok"] and r0["worst"] <= lim): row[lab] = 0.0; continue
            for _ in range(7):
                mid = (lo + hi) / 2; r = holdout(A, reach, mid)
                if r["ok"] and r["worst"] <= lim: lo = mid
                else: hi = mid
            row[lab] = round(lo, 3)
        ho.append(row)
    res["holdout"] = ho
  if "random" in SECS:
    # ---- random poses, thrust along the tip axis
    rng = np.random.default_rng(7); n = 1500; cnt = dict(off=[0, 0], hover=[0, 0], rated=[0, 0])
    for _ in range(n):
        q = rng.uniform(-PS.LIM, PS.LIM, S.N); f = A.fk(q)
        for k, T in (("off", 0.0), ("hover", FAN_HOVER), ("rated", RATED)):
            w = A.need(q, T * f["tipR"][:, 2])[0].max()
            cnt[k][0] += w <= 1.0; cnt[k][1] += w <= 0.5
    res["random_poses"] = {k: dict(held_at_stall=round(v[0] / n, 3), held_at_2x=round(v[1] / n, 3)) for k, v in cnt.items()}
    res["random_poses"]["n"] = n
  out[arr] = res
  print(arr, SECS, "done", round(time.time() - t0), "s", json.dumps(res, default=str)[:3000])
tag = "_" + ARRS[0] + "_" + "_".join(SECS)
json.dump(out, open(os.path.join(S.HERE, "results", f"loads{tag}.json"), "w"), indent=1, default=float)
np.savez(os.path.join(S.HERE, "results", f"poses{tag}.npz"), **store)
