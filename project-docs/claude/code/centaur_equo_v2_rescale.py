"""Equo-1 v2 at its new mass: re-scale the leg and body study results, pick leg actuators, keel tube and the rear shocks.
Scaling (est.): joint torque ~ mass x leg length at the same front share and gait; frame bending ~ mass x keel span;
power ~ mass.  Leg study basis: 21.4 kg, leg 380 mm (hip 361).  Body study basis: 21.97 kg, keel 490 mm.
python3 equo_v2_rescale.py -> equo_v2_rescale.json, sheet_equo_v2_rescale.png"""
import sys, json
sys.path.insert(0, "/home/claude/octodrone")
import numpy as np, sk3d
L = json.load(open("legs/legs.json")); B = json.load(open("body/body.json")); Z = json.load(open("equo_v2_sizing.json"))
M_LEG0, LEG0 = 21.4, 0.38
M_BODY0, KEEL0 = 21.97, 0.49
m_v2 = Z["totals"]["centaur"]                                   # 30.9 kg with RS02 legs and 490 mm legs
w0 = L["worst"]["R  12 x RobStride 02, 12S bus"]                # actuator / need, all brief cases, worst joint
# actuators (makers' listings): rated, peak N m, mass kg, USD
ACT = {"RS02": (6.0, 17.0, 0.405, 145), "RS06": (11.0, 36.0, 0.621, 210), "RS03": (20.0, 60.0, 0.880, 225)}
LEGS = {"490 mm (v2 as drawn)": 0.490 / 0.95 * 0.95 * (0.38 / 0.361), "380 mm (pony, = leg study)": 0.38}
SETS = {"RS02 x 12": ("RS02", "RS02"), "RS06 hips + knees, RS02 abduction": ("RS06", "RS02"), "RS03 hips + knees, RS02 abduction": ("RS03", "RS02")}
rows = []
for ln, leg in LEGS.items():
    for sn, (main, abd) in SETS.items():
        dm = 8 * (ACT[main][2] - ACT["RS02"][2])
        dleg = 0.0 if leg > 0.45 else -0.35                     # shorter legs: about 0.35 kg less structure (est.)
        m = m_v2 + dm + dleg
        f = (m / M_LEG0) * (leg / LEG0)
        r_rms = w0["req_rms"] * ACT[main][0] / 6.0 / f; r_pk = w0["req_peak"] * ACT[main][1] / 17.0 / f
        usd = 8 * ACT[main][3] + 4 * ACT[abd][3]
        top = (0.490 if leg > 0.45 else 0.361) + (1.043 - 0.490)
        rows.append(dict(legs=ln, set=sn, m=m, factor=f, rms=r_rms, peak=r_pk, usd=usd, top=top, ok=bool(r_rms >= 1.2 and r_pk >= 1.1)))
m_sel = [r for r in rows if r['legs'].startswith('490') and r['set'].startswith('RS06')][0]['m']   # recommended set
sc_m = m_sel / Z['totals']['centaur']
# keel tube
keel_new = 0.65                                                 # v2 body 740 mm long: keel about 650 (est.)
s0 = 96.0                                                       # MPa worst keel stress in the body study (2.5 g diagonal landing)
Zs = {"25 x 22 (body study)": (25, 22), "30 x 27": (30, 27), "32 x 28": (32, 28)}
keel = {}
for n, (D, d) in Zs.items():
    zr = ((D ** 4 - d ** 4) / D) / ((25 ** 4 - 22 ** 4) / 25)
    mass = np.pi / 4 * (D ** 2 - d ** 2) * 1e-6 * keel_new * 1550 * 2
    keel[n] = dict(stress=s0 * (m_sel / M_BODY0) * (keel_new / KEEL0) / zr, margin=250 / (s0 * (m_sel / M_BODY0) * (keel_new / KEEL0) / zr), mass=mass)
# power / runtime (body study 2 packs)
pw = {}
for g, P0, t0 in (("stand", 33.0, None), ("walk 0.5 m/s", 114.0, 94.0), ("trot", 209.0, 51.0)):
    P = P0 * m_sel / M_BODY0; pw[g] = dict(P=P, minutes=(t0 * M_BODY0 / m_sel) if t0 else None)
# rear shocks: two dampers mounted at the hip line (motion ratio ~1), one per side
F_rear = Z["swing"]["F_rear_stand"] * m_v2 / Z["totals"]["centaur"] if False else Z["swing"]["F_rear_stand"]
F_rear = F_rear * sc_m; k_wheel = Z["swing"]["k_wheel_N_per_mm"] * sc_m
shock = dict(n=2, per_shock_rate=k_wheel / 2, per_shock_static=F_rear / 2, per_shock_landing=2.5 * F_rear / 2, stroke=40.0,
             damping_each=Z["swing"]["damping_target_Ns_per_m"] * sc_m / 2,
             part="goBILDA / ServoCity Big Bore Shock, 190 mm, 2-pack ($39.99), 6 mm shaft, oil filled, M4 eyelets, 135 g each; spring rate and stroke not published")
out = dict(m_sel=m_sel, rows=rows, keel=keel, power=pw, shock=shock, m_v2=m_v2)
json.dump(out, open("equo_v2_rescale.json", "w"), indent=1, default=float)
print(f"centaur v2 with RS02 legs: {m_v2:.1f} kg; recommended set {m_sel:.1f} kg (keel, power, shocks below use this)")
for r in rows:
    print(f"  {r['legs']:28s} {r['set']:36s} {r['m']:5.1f} kg  torque x{r['factor']:.2f}  margin rms {r['rms']:.2f} peak {r['peak']:.2f}  ${r['usd']}  top {r['top']*1e3:.0f}  {'OK' if r['ok'] else 'short'}")
for n, k in keel.items(): print(f"  keel {n:22s} {k['stress']:.0f} MPa  x{k['margin']:.2f}  {k['mass']*1e3:.0f} g (pair)")
for g, p in pw.items(): print(f"  {g:14s} {p['P']:.0f} W" + (f"  {p['minutes']:.0f} min on 2 packs" if p['minutes'] else ""))
print(f"  shocks: 2 x {shock['per_shock_rate']:.1f} N/mm, {shock['per_shock_static']:.0f} N static, {shock['per_shock_landing']:.0f} N landing, 40 mm stroke, {shock['damping_each']:.0f} N s/m each")

# sheet: a plain table
import matplotlib.pyplot as plt
fig = sk3d.figure(16, 7, f"Equo-1 v2 at its new mass: which legs and actuators carry it")
ax = fig.add_axes([0.02, 0.50, 0.96, 0.42]); ax.axis("off")
hdr = ["Leg length", "Hip + knee actuators", "Centaur", "Torque x", "Margin rated", "Margin peak", "Actuators", "Top of octopus", ""]
cells = [[r["legs"], r["set"], f"{r['m']:.1f} kg", f"x{r['factor']:.2f}", f"{r['rms']:.2f}", f"{r['peak']:.2f}", f"${r['usd']:,}", f"{r['top']*1e3:.0f} mm", "OK" if r["ok"] else "short"] for r in rows]
tb = ax.table(cellText=cells, colLabels=hdr, loc="upper left", cellLoc="left", colLoc="left",
              colWidths=[0.16, 0.22, 0.07, 0.10, 0.08, 0.08, 0.08, 0.10, 0.06])
tb.auto_set_font_size(False); tb.set_fontsize(12); tb.scale(1, 2.0)
best = max([r for r in rows if r["ok"]], key=lambda r: (r["legs"].startswith("490"), -r["usd"]))
for i, r in enumerate(rows):
    for j in range(len(hdr)):
        c = tb[i + 1, j]
        if not r["ok"]: c.set_facecolor("#fbe3e1")
        if r is best: c.set_facecolor("#dff3e4"); c.get_text().set_fontweight("bold")
for j in range(len(hdr)): tb[0, j].get_text().set_fontweight("bold")
k = keel["30 x 27"]
sk3d.panel_text(fig, 0.02, 0.50, [
    "Margin = what the actuator gives / what the worst joint needs over all the leg study's cases.  Want at least 1.2 rated, 1.1 peak.",
    f"Keel: the body study's 25 x 22 tubes reach {keel['25 x 22 (body study)']['stress']:.0f} MPa (x{keel['25 x 22 (body study)']['margin']:.2f}) on the longer v2 body; "
    f"30 x 27 tubes give {k['stress']:.0f} MPa (x{k['margin']:.2f}) for {k['mass']*1e3:.0f} g the pair.",
    f"Power: walk {pw['walk 0.5 m/s']['P']:.0f} W ({pw['walk 0.5 m/s']['minutes']:.0f} min on two packs), trot {pw['trot']['P']:.0f} W ({pw['trot']['minutes']:.0f} min).",
    f"Rear shocks: two goBILDA Big Bore 190 mm dampers at the hip line, {shock['per_shock_rate']:.1f} N/mm each, {shock['per_shock_static']:.0f} N static, "
    f"{shock['per_shock_landing']:.0f} N landing.  Stock spring rate unpublished: weigh it in, swap springs if needed.",
], fs=12)
sk3d.save(fig, "sheet_equo_v2_rescale.png")
