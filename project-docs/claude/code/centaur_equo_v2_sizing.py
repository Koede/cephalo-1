"""Equo-1 v2: first sizing of the new parts.  (1) rib-hoop frame + skin, (2) CORLEO-style rear swing unit with a
coil-over and a lockout, (3) 4-joint tail (kickstand / balance / third hand).  Hand calculations, estimates marked.
python3 equo_v2_sizing.py -> equo_v2_sizing.json, sheet_equo_v2_sizing.png"""
import sys, json, contextlib, io
sys.path.insert(0, "/home/claude/octodrone")
import numpy as np, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import sk3d
with contextlib.redirect_stdout(io.StringIO()):
    import equo_v2 as V
G = 9.81
S_PACF, S_PETG = 80.0, 20.0                      # MPa allowables used in the project (est.)
RHO = dict(pacf=1150.0, petg=1250.0, cf=1550.0, steel=7850.0)
r = V.SB.rC; M = r["m"]; W = M * G; SHARE = r["share"]
out = {}

# ---------------------------------------------------------------- 1. rib frame
rib_h, rib_t = 10.0, 5.0                         # mm: radial depth x thickness (PA-CF)
per, bbox, nsplit = [], [], []
for ring in V.rings[1:]:
    seg = np.linalg.norm(np.diff(ring, axis=0), axis=1).sum() * 1e3
    ext = (ring.max(0) - ring.min(0)) * 1e3
    per.append(seg); big = max(ext[0] ** 2 + ext[2] ** 2, 0) ** 0.5
    bbox.append(ext); nsplit.append(1 if max(ext[1], np.hypot(ext[0], ext[2])) < 235 else 2)
per = np.array(per)
m_ribs = per.sum() * rib_h * rib_t * 1e-9 * RHO["pacf"] * 0.85          # 85 % infill-equivalent (est.)
n_str = 8; L_spine = np.linalg.norm(np.diff(V.spine, axis=0), axis=1).sum()
m_str = n_str * L_spine * 1.2 * np.pi * (0.0025 ** 2 - 0.0015 ** 2) * RHO["cf"]   # 5 x 3 mm carbon tube stringers
area = 0.0
for r0, r1 in zip(V.rings[:-1], V.rings[1:]):
    for j in range(28):
        a, b, c, d = r0[j], r0[j + 1], r1[j + 1], r1[j]
        area += 0.5 * np.linalg.norm(np.cross(b - a, d - a)) + 0.5 * np.linalg.norm(np.cross(c - b, d - b))
skins = {"ripstop nylon fabric (40 g/m2, laced on, like the War Horse gauze)": 0.040 * area * 1.15,
         "PETG panels 0.8 mm (translucent)": area * 0.8e-3 * RHO["petg"],
         "no skin (open frame)": 0.0}
F_poke = 50.0; R_max = max(np.hypot(b[0], b[2]) for b in bbox) / 2 / 1e3
Mr = F_poke * R_max / np.pi                       # thin ring, diametral load
Z = rib_t * rib_h ** 2 / 6
out["frame"] = dict(n_ribs=len(per), perimeter_total_mm=per.sum(), split_in_two=int(sum(n == 2 for n in nsplit)), m_ribs=m_ribs, m_stringers=m_str,
                    skin_area_m2=area, skins=skins, rib_stress_50N=Mr * 1e3 / Z, rib_margin=S_PACF / (Mr * 1e3 / Z), R_max_mm=R_max * 1e3,
                    largest_rib_mm=float(max(np.hypot(b[0], b[2]) for b in bbox)))

# ---------------------------------------------------------------- 2. rear swing unit
L_pv = 0.200            # pivot ahead of the rear hip line
L_sp = 0.150            # coil-over attachment on the subframe, from the pivot
ang = np.radians(35)    # coil-over line from vertical (from the v2 sketch)
F_rear = (1 - SHARE) * W                         # both hind feet, standing
travel_hip = 0.040; sag_frac = 0.30              # wheel travel at the hip, static sag 30 % (est., bike practice)
sag = travel_hip * sag_frac
k_wheel = F_rear / sag                            # N/m at the hip
MR = (L_sp * np.cos(ang)) / L_pv                  # motion ratio spring / hip
k_spring = k_wheel / MR ** 2
F_spring_static = F_rear / MR
stroke = travel_hip * MR
m_rear = (1 - SHARE) * M
f_n = np.sqrt(k_wheel / m_rear) / (2 * np.pi)
c_crit = 2 * np.sqrt(k_wheel * m_rear)
cases = {"stand": 1.0, "trot peak (x1.4 on the pair, est.)": 1.4 * 2 / 2, "2.5 g landing": 2.5}
sw = {}
for n, fac in cases.items():
    F = F_rear * fac
    sw[n] = dict(F_hip=F, spring=F / MR, defl_hip_mm=min(F / k_wheel, travel_hip) * 1e3, bottoms=F / k_wheel > travel_hip,
                 pivot_load=F * (1 + L_pv / L_pv) - F, pivot_shear=F + F / MR * np.cos(ang))
# pivot: 2 x 6800 bearings (C0 0.84 kN each, as in the project's monitor) on a 10 mm shaft
C0 = 2 * 840.0
out["swing"] = dict(F_rear_stand=F_rear, k_wheel_N_per_mm=k_wheel / 1e3, k_spring_N_per_mm=k_spring / 1e3, motion_ratio=MR,
                    F_spring_static=F_spring_static, spring_stroke_mm=stroke * 1e3, f_n_Hz=f_n, damping_crit_Ns_per_m=c_crit,
                    damping_target_Ns_per_m=0.3 * c_crit, cases=sw, pivot_C0=C0,
                    pivot_margin_landing=C0 / sw["2.5 g landing"]["pivot_shear"],
                    m_subframe=0.18 + 0.12 + 0.06 + 0.05)   # 2 carbon arms + cross tube, shock, pivot bearings/shaft, lockout servo (est.)
# lockout: an STS3215 swings a pin into the subframe so the rear is rigid for precise work (standing, mounting, rear-up)
pin_d = 5.0; F_lock = sw["2.5 g landing"]["F_hip"] * L_pv / 0.05            # pin 50 mm from the pivot
out["swing"]["lockout"] = dict(pin_mm=pin_d, pin_force_landing=F_lock, pin_shear_MPa=F_lock / (2 * np.pi * pin_d ** 2 / 4))

# ---------------------------------------------------------------- 3. tail
links = np.array([0.080, 0.070, 0.060, 0.090])            # root -> tip (m)
servos = ["STS3250", "STS3250", "STS3215", "STS3215"]
SV = {"STS3215": (0.98, 2.94, 0.055), "STS3250": (1.63, 4.90, 0.0745)}
m_link = 0.025; m_grip = 0.060
def tail_torques(th, payload, tip_force=np.zeros(2)):
    """th: absolute link angles from horizontal-back (rad); returns torque at each joint (N m)."""
    P = [np.zeros(2)]
    for L, a in zip(links, th): P.append(P[-1] + L * np.array([-np.cos(a), np.sin(a)]))
    P = np.array(P); tq = []
    for j in range(4):
        t = 0.0
        for k in range(j, 4):
            mid = 0.5 * (P[k] + P[k + 1]); t += (m_link * G) * (mid[0] - P[j][0])
            if k + 1 < 4: t += SV[servos[k + 1]][2] * G * (P[k + 1][0] - P[j][0])
        t += (m_grip + payload) * G * (P[-1][0] - P[j][0])
        t += tip_force[1] * (P[-1][0] - P[j][0]) - tip_force[0] * (P[-1][1] - P[j][1])
        tq.append(abs(t))
    return np.array(tq), P
poses = {"held out level, 0.3 kg in the gripper": (np.zeros(4), 0.3, np.zeros(2)),
         "curled up (rest)": (np.radians([40, 90, 140, 200]), 0.0, np.zeros(2)),
         "kickstand, column straight down, 10 % of weight": (np.radians([-90, -90, -90, -90]), 0.0, np.array([0, 0.10 * W])),
         "kickstand raked 15 deg back, 10 % of weight": (np.radians([-75, -75, -75, -75]), 0.0, np.array([0, 0.10 * W]))}
tl = {}
for n, (th, pl, f) in poses.items():
    tq, P = tail_torques(th, pl, f)
    tl[n] = dict(torques=tq.tolist(), frac_rated=[t / SV[s][0] for t, s in zip(tq, servos)], frac_stall=[t / SV[s][1] for t, s in zip(tq, servos)])
out["tail"] = dict(length_mm=links.sum() * 1e3, servos=servos, mass=4 * m_link + m_grip + sum(SV[s][2] for s in servos), poses=tl,
                   column_axial_N=0.10 * W)

# ---------------------------------------------------------------- totals
body_frame_old = 0.854                                    # body study frame + shells (keel tubes kept)
keel = 0.45                                               # the two 25 x 22 keel tubes + girdles kept (est. share of 0.854)
skin = skins["ripstop nylon fabric (40 g/m2, laced on, like the War Horse gauze)"]
new_frame = keel + m_ribs + m_str + skin + 0.08            # + lacing, clips (est.)
d_mass = (new_frame - body_frame_old) + out["swing"]["m_subframe"] + out["tail"]["mass"]
out["totals"] = dict(frame_old=body_frame_old, frame_new=new_frame, added_swing=out["swing"]["m_subframe"], added_tail=out["tail"]["mass"],
                     delta=d_mass, centaur=M + d_mass)
json.dump(out, open("equo_v2_sizing.json", "w"), indent=1, default=lambda v: v.tolist() if hasattr(v, "tolist") else bool(v) if isinstance(v, np.bool_) else float(v))

f = out["frame"]; s = out["swing"]; t = out["tail"]; T = out["totals"]
print(f"FRAME  {f['n_ribs']} ribs 10x5 PA-CF, {f['perimeter_total_mm']:.0f} mm of rib, {f['split_in_two']} printed in two halves (largest {f['largest_rib_mm']:.0f} mm)")
print(f"       ribs {f['m_ribs']*1e3:.0f} g, stringers {f['m_stringers']*1e3:.0f} g, skin area {f['skin_area_m2']:.2f} m2: " + ", ".join(f"{k.split(' (')[0]} {v*1e3:.0f} g" for k, v in f['skins'].items()))
print(f"       rib under a 50 N poke: {f['rib_stress_50N']:.0f} MPa (x{f['rib_margin']:.1f})")
print(f"SWING  rear {s['F_rear_stand']:.0f} N standing; wheel rate {s['k_wheel_N_per_mm']:.1f} N/mm, motion ratio {s['motion_ratio']:.2f}, spring {s['k_spring_N_per_mm']:.1f} N/mm, preload-free static {s['F_spring_static']:.0f} N, stroke {s['spring_stroke_mm']:.0f} mm")
print(f"       f_n {s['f_n_Hz']:.1f} Hz, damping target {s['damping_target_Ns_per_m']:.0f} N s/m (30 % of critical)")
for n, c in s["cases"].items(): print(f"       {n:32s} hip {c['F_hip']:.0f} N, spring {c['spring']:.0f} N, sink {c['defl_hip_mm']:.0f} mm{'  BOTTOMS (bump stop)' if c['bottoms'] else ''}, pivot {c['pivot_shear']:.0f} N")
print(f"       pivot 2 x 6800 C0 {s['pivot_C0']:.0f} N: x{s['pivot_margin_landing']:.1f} at landing; lockout pin dia 5 double shear {s['lockout']['pin_shear_MPa']:.0f} MPa at landing ({s['lockout']['pin_force_landing']:.0f} N)")
print(f"TAIL   {t['length_mm']:.0f} mm, {t['mass']*1e3:.0f} g, servos {t['servos']}")
for n, p in t["poses"].items(): print(f"       {n:48s} torque " + " ".join(f"{x:.2f}" for x in p['torques']) + "  of rated " + " ".join(f"{x:.2f}" for x in p['frac_rated']))
print(f"TOTAL  frame {T['frame_old']*1e3:.0f} -> {T['frame_new']*1e3:.0f} g, + swing {T['added_swing']*1e3:.0f} g, + tail {T['added_tail']*1e3:.0f} g: delta {T['delta']*1e3:+.0f} g, centaur {T['centaur']:.1f} kg")

# ---------------------------------------------------------------- sheet
C = sk3d.COL
fig = sk3d.figure(16, 9, "Equo-1 v2 sizing: rib frame, rear swing unit, tail")
sc = sk3d.Scene(fig, [0.0, 0.30, 0.36, 0.62], elev=14, azim=-60, title="1  Rib frame")
V.frame(sc); sc.done((-0.75, -0.3, 0.2), (0.45, 0.3, 0.8))
sk3d.panel_text(fig, 0.02, 0.30, [
    f"{f['n_ribs']} hoops, 10 x 5 mm PA-CF, {f['split_in_two']} printed in two halves",
    f"ribs {f['m_ribs']*1e3:.0f} g + 8 carbon stringers {f['m_stringers']*1e3:.0f} g",
    f"skin: ripstop nylon {f['skins'][list(f['skins'])[0]]*1e3:.0f} g (PETG panels would be {f['skins'][list(f['skins'])[1]]*1e3:.0f} g)",
    f"50 N poke on the biggest hoop: {f['rib_stress_50N']:.0f} MPa (x{f['rib_margin']:.1f})",
    "Ribs carry the skin only; the keel tubes",
    "carry the octopus and the legs.",
], fs=12)
# swing unit diagram
ax = fig.add_axes([0.38, 0.30, 0.28, 0.58]); ax.set_title("2  Rear swing unit (side)", loc="left", fontsize=13, fontweight="bold")
hip = np.array([0.0, 0.0]); piv = np.array([L_pv, 0.0]); sp = np.array([L_pv - L_sp, 0.0])
top = sp + 0.12 * np.array([np.sin(ang), np.cos(ang)])
ax.plot([0, L_pv], [0, 0], color=C["bar"], lw=8, solid_capstyle="round")
ax.add_patch(plt.Circle(piv, 0.012, color=C["act"])); ax.add_patch(plt.Circle(hip, 0.03, color=C["act"]))
zz = np.linspace(0, 1, 40); dv = top - sp; nv = np.array([-dv[1], dv[0]]) / np.linalg.norm(dv)
coil = sp[None] + zz[:, None] * dv[None] + 0.010 * np.sin(zz * 2 * np.pi * 7)[:, None] * nv[None]
ax.plot(coil[:, 0], coil[:, 1], color=C["spring"], lw=2.5)
ax.plot([top[0] - 0.03, top[0] + 0.05], [top[1], top[1]], color="#888", lw=6)
ax.plot([0, 0], [0, -0.20], color=C["horse"], lw=7); ax.text(-0.02, -0.22, "hind legs", fontsize=11)
ax.annotate("", xy=(0, -0.07), xytext=(0, -0.17), arrowprops=dict(arrowstyle="->", color=C["hi"], lw=2))
ax.text(0.01, -0.13, f"{s['F_rear_stand']:.0f} N stand\n{s['cases']['2.5 g landing']['F_hip']:.0f} N landing", color=C["hi"], fontsize=11)
ax.text(piv[0] + 0.005, -0.055, "pivot\n2 x 6800", fontsize=11, ha="center")
ax.text(top[0] + 0.03, top[1] - 0.05, f"coil-over\n{s['k_spring_N_per_mm']:.0f} N/mm\n{s['spring_stroke_mm']:.0f} mm stroke", fontsize=11, color=C["spring"])
ax.plot([L_pv - 0.05], [0.0], "s", color=C["latch"], ms=9); ax.text(L_pv - 0.12, -0.05, "lockout pin\n(STS3215)", fontsize=10, color=C["latch"])
ax.set_xlim(-0.08, 0.30); ax.set_ylim(-0.26, 0.17); ax.set_aspect("equal"); ax.axis("off")
sk3d.panel_text(fig, 0.39, 0.30, [
    f"Hip sinks {s['cases']['stand']['defl_hip_mm']:.0f} mm standing, {s['cases']['trot peak (x1.4 on the pair, est.)']['defl_hip_mm']:.0f} mm",
    f"in a trot, {s['cases']['2.5 g landing']['defl_hip_mm']:.0f} mm (bump stop) landing.",
    f"Bounce {s['f_n_Hz']:.1f} Hz: needs a damper,",
    f"about {s['damping_target_Ns_per_m']:.0f} N s/m at the hip.",
    f"Pivot bearings x{s['pivot_margin_landing']:.1f} at landing.",
    "Lockout makes the rear rigid for",
    "standing, mounting and rearing up.",
], fs=12)
# tail
ax = fig.add_axes([0.68, 0.30, 0.31, 0.58]); ax.set_title("3  Tail: 4 joints, 300 mm", loc="left", fontsize=13, fontweight="bold")
cols = ["#2f6db3", "#7a7a7a", "#2e9e5b", "#d43a2f"]
for (n, (th, pl, fv)), c, dx in zip(poses.items(), cols, (0.0, 0.0, 0.0, 0.0)):
    tq, P = tail_torques(th, pl, fv)
    ax.plot(P[:, 0], P[:, 1], "-o", color=c, lw=3, ms=5, label=n.split(",")[0])
ax.plot(0, 0, "s", color="k", ms=10); ax.axhline(-0.31, color="#aaa", lw=1)
ax.set_aspect("equal"); ax.axis("off"); ax.legend(loc="upper left", fontsize=10, frameon=False, bbox_to_anchor=(0.0, 1.0))
ax.set_xlim(-0.34, 0.12); ax.set_ylim(-0.33, 0.26)
p1 = t["poses"]["held out level, 0.3 kg in the gripper"]; p4 = t["poses"]["kickstand raked 15 deg back, 10 % of weight"]
sk3d.panel_text(fig, 0.69, 0.30, [
    f"{t['mass']*1e3:.0f} g: 2 x STS3250 at the root, 2 x STS3215.",
    f"Holding 0.3 kg straight out: root at {p1['frac_rated'][0]*100:.0f} % of rated.",
    f"Kickstand straight down: {t['column_axial_N']:.0f} N in the column, 0 torque.",
    f"Raked 15 deg on the servos: {p4['frac_rated'][0]*100:.0f} % of rated: too much.",
    "Fix: hard stops at 'straight' on the side the",
    "ground pushes, so a raked kickstand locks",
    "on its stops (like the knee stay), 0 servo torque.",
], fs=12)
sk3d.panel_text(fig, 0.02, 0.07, [
    f"Mass: frame {T['frame_old']*1e3:.0f} -> {T['frame_new']*1e3:.0f} g, swing unit +{T['added_swing']*1e3:.0f} g, tail +{T['added_tail']*1e3:.0f} g.  Centaur about {T['centaur']:.1f} kg (est.).",
], fs=13)
sk3d.save(fig, "sheet_equo_v2_sizing.png")
