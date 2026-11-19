"""Checks on the true solids of the hand-prop rev B (hand3_cad.py).  python3 hand3_cad_checks.py -> hand3_cad_checks.json
1 parts (single watertight bodies, masses)   2 straight finger: overlaps and gaps   3 knuckle fold sweeps and stops
4 full curl and spread   5 flight spin, rotating against stationary   6 claw stroke at the park index   7 servo pack
8 loads at proof speed from the CAD masses, ring net sections   9 frontal area of the knuckle hardware (for hand3_thrust.py)"""
import json, itertools
import numpy as np
import manifold3d as mf
import hand3_cad as C
import hand3_design as H

M_ = mf.Manifold; out = {}; TOL = 0.02                    # mm3: faceted cylinders sharing an axis overlap by a few 0.001 mm3
def ov(a, b): return float((a ^ b).volume())
def gap(a, b, L=6.0): return float(a.min_gap(b, L))
def grp(asm, pred): return M_.compose([m for n, (m, _) in asm.items() if pred(n)])
P = C.parts()

# ---- 1 parts
rep = C.export(); out["parts"] = rep
bad = [n for n, r in rep.items() if not (r["watertight"] and r["bodies"] == 1)]
print("1 parts:", len(rep), "all single watertight bodies:", not bad, bad)

# ---- 2 straight finger
A = C.assembly(hw=False); f0 = {n: m for n, (m, _) in A.items() if n in ("grip0", "tube0", "ph0_1", "ph0_2", "ph0_3", "pad0_1", "pad0_2", "pad0_3", "hub_lower", "hub_upper")}
pairs = {}
for a, b in itertools.combinations(f0, 2):
    v = ov(f0[a], f0[b])
    if v > TOL: pairs[f"{a}|{b}"] = round(v, 3)
out["straight_overlaps"] = pairs
out["straight_gaps"] = {f"{a}|{b}": round(gap(f0[a], f0[b]), 3) for a, b in (("grip0", "ph0_1"), ("ph0_1", "ph0_2"), ("ph0_2", "ph0_3"), ("grip0", "hub_lower"), ("grip0", "hub_upper"), ("grip0", "tube0"))}
print("2 straight: overlaps", pairs, "| gaps", out["straight_gaps"])

# ---- 3 fold sweeps (finger 0; members = phalanx + its pad; member 0 = grip)
def members(fold):
    a = C.assembly(fold=(fold,) + ((0, 0, 0),) * 3, hw=False)
    return [a["grip0"][0]] + [a[f"ph0_{j}"][0] + a[f"pad0_{j}"][0] for j in (1, 2, 3)]
sweep = {}
for j in range(3):
    free = []
    for th in np.arange(-8.0, H.FOLD_MAX[j] + 6.01, 1.0):
        f = [0.0, 0.0, 0.0]; f[j] = float(th); mm = members(tuple(f))
        if ov(mm[j], mm[j + 1]) <= TOL: free.append(float(th))
    sweep[f"K{j + 1}"] = dict(stop_back=min(free), stop_fold=max(free), want=(-H.PRECONE, H.FOLD_MAX[j]),
                              continuous=bool(len(free) == int(max(free) - min(free)) + 1))
out["fold"] = sweep; print("3 fold range free of contact (deg):", sweep)

# ---- 4 full curl, spread
full = tuple(H.FOLD_MAX); a = C.assembly(fold=(full,) * 4, hw=False); rot = {n: m for n, (m, r) in a.items() if r}
hits = {}
names = [n for n in rot if n.startswith(("ph", "pad", "grip", "hub", "heel", "tube"))]
for x, y in itertools.combinations(names, 2):
    if x[2:4] == y[2:4] and x[:2] in ("ph", "pa") and y[:2] in ("ph", "pa") and abs(int(x[-1]) - int(y[-1])) <= 1 and x.split("_")[0][-1] == y.split("_")[0][-1]: continue
    v = ov(rot[x], rot[y])
    if v > TOL: hits[f"{x}|{y}"] = round(v, 2)
out["full_curl_overlaps"] = hits; print("4 full curl (90/90/85 on all four) overlaps:", hits if len(hits) < 9 else f"{len(hits)} pairs, e.g. {dict(list(hits.items())[:4])}")
lagS = [s * H.SPREAD_MAX for s in H.LAG_SIGN]; a = C.assembly(lag=lagS, fold=((55, 60, 50),) * 4, pull=[9.0] * len(H.CHANNELS))
allp = {n: m for n, (m, r) in a.items()}; gh = {}
for x, y in itertools.combinations([n for n in allp if not n.startswith(("hw_servo", "hw_spool", "pack"))], 2):
    v = ov(allp[x], allp[y])
    if v > TOL: gh[f"{x}|{y}"] = round(v, 2)
out["grasp_pose_overlaps"] = gh; print("  grasp pose (spread 47, fold 55/60/50, pull 9): overlaps", gh)
sp = {}
for frac in (0.0, 0.25, 0.5, 0.75, 1.0):
    lag = [s * H.SPREAD_MAX * frac for s in H.LAG_SIGN]; a = C.assembly(lag=lag, hw=False)
    hub = a["hub_lower"][0] + a["hub_upper"][0]
    fing = [M_.compose([a[f"grip{k}"][0]] + [a[f"ph{k}_{j}"][0] for j in (1, 2, 3)] + [a[f"pad{k}_{j}"][0] for j in (1, 2, 3)]) for k in range(4)]
    sp[f"{frac * H.SPREAD_MAX:.1f}"] = dict(grip_hub_overlap=round(max(ov(a[f"grip{k}"][0], hub) for k in range(4)), 3), grip_hub_gap=round(min(gap(a[f"grip{k}"][0], hub) for k in range(4)), 3),
                                             chain_hub_gap=round(min(gap(a[f"ph{k}_1"][0], hub) for k in range(4)), 2),
                                             pair_gap=round(min(gap(fing[0], fing[3], 12.0), gap(fing[1], fing[2], 12.0)), 2), pair_overlap=round(ov(fing[0], fing[3]) + ov(fing[1], fing[2]), 3))
out["spread"] = sp; print("  spread sweep:", sp)

# ---- 5 flight spin
A0 = C.assembly()
stat = grp(A0, lambda n: not A0[n][1]); g = []
for spin in np.arange(0.0, 360.0, 6.0):
    a = C.assembly(spin=float(spin)); g.append(gap(grp(a, lambda n: a[n][1]), stat, 8.0))
out["flight_spin"] = dict(min_gap=round(min(g), 3), at_deg=float(np.argmin(g) * 6), overlap_mm3=round(ov(grp(A0, lambda n: A0[n][1]), stat), 3))
near = {}
for n, (m, r) in A0.items():
    if r:
        d = gap(m, stat, 3.0)
        if d < 2.5: near[n] = round(d, 2)
out["flight_spin"]["closest_rotating_parts_at_0"] = dict(sorted(near.items(), key=lambda kv: kv[1])[:6]); print("5 flight spin:", out["flight_spin"])

# ---- 6 claw stroke at the park index: the claw travels 1 mm before it meets the fin, then both move together
st = {}
for p in (0.0, 1.0, 7.0, H.HUB["follow_travel"] + 1.0):
    a = C.assembly(pull=[max(p - 1.0, 0)] * len(H.CHANNELS), claw_pull=[p] * len(H.CHANNELS))
    claws = grp(a, lambda n: n.startswith("claw")); fol = grp(a, lambda n: n.startswith("fol"))
    rest_rot = grp(a, lambda n: a[n][1] and not n.startswith("fol")); rest_st = grp(a, lambda n: not a[n][1] and not n.startswith("claw"))
    st[f"{p:.1f}"] = dict(claw_fin_gap=round(gap(claws, fol), 3), claw_fin_overlap=round(ov(claws, fol), 3), claw_rotor_gap=round(gap(claws, rest_rot), 2), claw_pack_overlap=round(ov(claws, rest_st), 3),
                          claw_pack_gap=round(gap(claws, grp(a, lambda n: n.startswith("hw_"))), 2), follower_cage_overlap=round(ov(fol, a["cage"][0]), 3), follower_rotor_gap=round(gap(fol, grp(a, lambda n: n.startswith(("hub", "grip", "hw_motor")))), 2))
cl = [a[f"claw{i}"][0] for i in range(len(H.CHANNELS))]
out["claw_stroke"] = st; out["claw_to_claw_gap"] = round(min(gap(x, y) for x, y in itertools.combinations(cl, 2)), 2); print("6 claw stroke:", st, "claw-claw", out["claw_to_claw_gap"])

# ---- 7 servo pack
hw = C.hardware(); sv = {n: m for n, (m, _, _) in hw.items() if n.startswith("servo")}; spl = {n: m for n, (m, _, _) in hw.items() if n.startswith("spool")}
out["pack"] = dict(servo_servo_gap=round(min(gap(x, y) for x, y in itertools.combinations(sv.values(), 2)), 2), servo_frame_overlap=round(sum(ov(m, P["pack_frame"]) for m in sv.values()), 3),
                   spool_frame_overlap=round(sum(ov(m, P["pack_frame"]) for m in spl.values()), 3), servo_frame_gap=round(min(gap(m, P["pack_frame"]) for m in sv.values()), 2),
                   spool_to_other_servo_gap=round(min(gap(s, v) for (ns, s), (nv, v) in itertools.product(spl.items(), sv.items()) if ns[6:] != nv[6:]), 2),
                   outer_diameter=round(float(2 * np.max(np.linalg.norm(np.asarray(P["pack_frame"].to_mesh().vert_properties)[:, :2], axis=1))), 1))
print("7 pack:", out["pack"])

# ---- 8 loads from the CAD masses at proof speed
w = 2 * np.pi * 1.15 * H.RPM_MAX / 60; Fr = C.finger_frames(0); cm = []
for j in range(3):
    for n, rho in ((f"phalanx{j + 1}", C.RHO["pacf"]), (f"pad{j + 1}", C.RHO["tpu"])):
        t = C.to_trimesh(C.app(P[n], Fr[j + 1])); cm.append((j, float(t.volume) * rho * 1e-6, float(np.linalg.norm(t.center_mass[:2]))))
HWG = [0.9, 0.9, 0.35]                                   # pins + bushings per knuckle, g (steel dowels, brass bushings)
loads = []
for j in range(3):
    m = sum(x[1] for x in cm if x[0] >= j) + sum(HWG[j:]) * 1e-3; F = sum(x[1] * x[2] * 1e-3 for x in cm if x[0] >= j) * w * w + sum(g_ * 1e-3 * H.KN[i] * 1e-3 for i, g_ in enumerate(HWG) if i >= j) * w * w
    ring = (H.BARREL[j] - H.BUSH_OD[j]) / 2 * 2
    loads.append(dict(knuckle=f"K{j + 1}", m_out_g=round(m * 1e3, 2), pull_N=round(F), design_file_N=H.knuckle_loads(1.15 * H.RPM_MAX)[j + 1]["F_proof"],
                      bearing_cheek_MPa=round(F / 2 / (H.BUSH_OD[j] * H.CHEEK[j]), 1), bearing_lug_MPa=round(F / 2 / (H.BUSH_OD[j] * H.LUG[j]), 1),
                      ring_cheek_MPa=round(F / 2 / (ring * H.CHEEK[j]), 1), ring_lug_MPa=round(F / 2 / (ring * H.LUG[j]), 1)))
tg = C.to_trimesh(C.app(P["root_grip"], Fr[0])); mg = float(tg.volume) * C.RHO["alu"] * 1e-6
Flag = loads[0]["pull_N"] + mg * float(np.linalg.norm(tg.center_mass[:2])) * 1e-3 * w * w
out["loads"] = dict(proof_rpm=round(1.15 * H.RPM_MAX), knuckles=loads, lag_pin_N=round(Flag), lag_eye_MPa=round(Flag / (H.LAG_TUBE[0] * 4.8), 1), tube_shear_MPa=round(Flag / (2 * np.pi * (H.LAG_TUBE[0] ** 2 - H.LAG_TUBE[1] ** 2) / 4), 1),
                    hub_plate_shearout_MPa=round(Flag / 2 / (2 * 2.5 * (H.HUB["plate_d"] / 2 - H.R_LAG - H.LAG_TUBE[0] / 2)), 1))
rot_mass = {n: rep[n]["mass_g"] for n in ("root_grip", "phalanx1", "phalanx2", "phalanx3", "pad1", "pad2", "pad3", "lag_tube")}
finger_g = sum(rot_mass.values()) + sum(HWG)
tot = dict(fingers=round(4 * finger_g, 1), hub_plates=round(rep["hub_lower"]["mass_g"] + rep["hub_upper"]["mass_g"], 1), cage_followers=round(rep["cage"]["mass_g"] + 14 * rep["follower"]["mass_g"], 1),
           heel_nut=round(rep["heel"]["mass_g"] + 2.0, 1), claw_bars=round(sum(r["mass_g"] for n, r in rep.items() if n.startswith("claw")), 1), pack_frame=rep["pack_frame"]["mass_g"],
           servos=H.design()["mass"]["servos"], motor=H.MOTOR["m"], spools_springs_links_screws_wiring=28.0)
tot["total"] = round(sum(tot.values()), 1); out["mass"] = tot; out["design_file_mass"] = H.design()["mass"]["total"]
print("8 loads:", out["loads"]); print("  mass from CAD:", tot, "| design file:", out["design_file_mass"])

# ---- 9 frontal area of the knuckle hardware seen along the chord (flow direction), beyond the clean airfoil
clean = C.loft(np.linspace(H.KN[0] - 12, H.R_TIP, 12), C.foil)
view = C.T(C.rot("x", 90.0))                              # chain y -> z, then project along z
fa = []
chain = M_.compose([C.app(P["root_grip"], C.T(C.rot("x", -H.PITCH)))] + [P[f"phalanx{j}"] for j in (1, 2, 3)] + [P[f"pad{j}"] for j in (1, 2, 3)])
for j in range(3):
    slab = C.box([H.KN[j] - 12, -30, -30], [H.KN[j] + 12, 30, 30])
    a_all = C.app((chain + clean) ^ slab, view).project().area(); a_clean = C.app(clean ^ slab, view).project().area()
    fa.append(round(float(a_all - a_clean), 1))
out["knuckle_frontal_area_mm2"] = fa; print("9 knuckle hardware frontal area beyond the airfoil (mm2):", fa)
json.dump(out, open("hand3_cad_checks.json", "w"), indent=1, default=float)
