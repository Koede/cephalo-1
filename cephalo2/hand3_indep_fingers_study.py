"""Options study (no CAD): how can each of hand3's four fingers move round the hub ring on its own?

Dimensions come from hand3e/hand3_design.py, hand3_cad.py and hand3_hubc.py (hub E), so the plan panels are to scale.
Two readings are covered: (a) each finger's lag (swing) angle set separately, (b) each finger's root travels round the hub.
Writes hand3_indep_fingers_study.png and hand3_indep_fingers_study.json.  Run from cephalo2/:  python3 hand3_indep_fingers_study.py
Units mm, N, g, deg.  Everything not read from a module or a maker's sheet is an estimate and is marked est.
"""
import os, sys, json, textwrap
import numpy as np, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Polygon, Wedge, Rectangle, FancyArrowPatch

HERE = os.path.dirname(os.path.abspath(__file__)); E_DIR = os.path.join(HERE, "hand3e")
sys.path.insert(0, E_DIR); _cwd = os.getcwd(); os.chdir(E_DIR)
import hand3_design as H
import hand3_cad as C
import hand3_hubc as E
HUBC = json.load(open("hand3_hubc.json")); THR = json.load(open("hand3_thrust_cad.json")); PARTS = json.load(open("hand3_cad_parts.json"))
os.chdir(_cwd)

rnd = lambda v, n=1: round(float(v), n)
def overlap(a, b): return min(a[1], b[1]) - max(a[0], b[0])          # > 0: the two ranges overlap by this much; < 0: gap

# ------------------------------------------------------------------ common loads
RPM_MAX, RPM_HOVER = 12500.0, 10900.0                                # est. (task brief; hand3_thrust_cad.json: 11,292-12,472 rpm at full thrust)
W = 2 * np.pi * RPM_MAX / 60; W2 = W * W
def cf(m_g, r_mm, w2=W2): return m_g * 1e-3 * w2 * r_mm * 1e-3      # centrifugal force, N
G_AT_LAG = W2 * H.R_LAG * 1e-3 / 9.81

# finger mass and centre of mass from the CAD solids (rotor frame, finger 0 straight at the flight cross)
_P = C.parts(); _Fr = C.finger_frames(0, 0.0, (0, 0, 0)); _m = 0.0; _mom = np.zeros(3)
for _n, _f in (("root_grip", 0), ("phalanx1", 1), ("pad1", 1), ("phalanx2", 2), ("pad2", 2), ("phalanx3", 3), ("pad3", 3)):
    _t = C.to_trimesh(C.app(_P[_n], _Fr[_f])); _m += PARTS[_n]["mass_g"]; _mom += PARTS[_n]["mass_g"] * _t.center_mass
M_F = _m; COM = _mom / _m; R_COM = float(np.hypot(COM[0], COM[1]))
PIN = (C.rot("z", H.FINGER_AZ[0]) @ np.array([H.R_LAG, 0, 0]))[:2]; D_COM = float(np.hypot(*(COM[:2] - PIN)))
F_FINGER = cf(M_F, R_COM); F_FINGER_PROOF = F_FINGER * 1.15 ** 2
K_RESTORE = M_F * 1e-3 * W2 * H.R_LAG * 1e-3 * D_COM * 1e-3            # N m per sin(lag): centrifugal pulls every finger back to the cross
T_RESTORE_47 = K_RESTORE * np.sin(np.radians(H.SPREAD_MAX))
RPM_10X = RPM_MAX * np.sqrt(10 * E.LAG_SPRING / T_RESTORE_47)          # rpm where the centrifugal restoring torque at 47 deg is 10 x the lag spring
IMB_STUCK = M_F * 1e-3 * W2 * 2 * D_COM * 1e-3 * np.sin(np.radians(H.SPREAD_MAX / 2))   # one finger held at 47 deg at full speed (cannot happen: see K_RESTORE)
def imb_az(deg): return F_FINGER * 2 * np.sin(np.radians(deg) / 2)     # rotor imbalance force when one finger root sits deg off its cross azimuth
def imb_set(az):                                                       # rotor imbalance force for four finger azimuths
    a = np.radians(az); return F_FINGER * float(np.hypot(np.cos(a).sum(), np.sin(a).sum()))

# what a finger needs from its base cord (hand3_design requirement: 5 N peak / 2 N hold at the middle of the tip phalanx)
F_PEAK_NEED = H.joint_torques(H.F_TIP_PEAK)[0] / (E.LEVER[0] * 1e-3); F_HOLD_NEED = H.joint_torques(H.F_TIP_HOLD)[0] / (E.LEVER[0] * 1e-3)
F_STALL_E, F_RATED_E = HUBC["base"]["F_stall_finger"], HUBC["base"]["F_rated_finger"]
ETA_E = E.ETA_SPOOL * E.ETA_GUIDE * 2.0 / (1.0 / E.ETA_IDLER + 1.0 / E.ETA_IDLER ** 3)
LAG_TRAVEL = E.E_LAG * np.radians(H.SPREAD_MAX)                        # ring stroke that is pure spread (before K1 starts to fold)
F_LAG_NEED = E.LAG_SPRING / (E.E_LAG * 1e-3)                           # cord pull to overcome the lag spring
ARM_TIP = H.R_TIP - H.PH[2] / 2 - H.R_LAG                              # lag pin -> middle of the tip phalanx
def side_force(F_cord): return F_cord * E.E_LAG / ARM_TIP              # sideways (in-plane) fingertip force from the lag lever
MASS_E = HUBC["mass"]["total"]; R_MAX_E = HUBC["envelope"]["r_max"]
STACK_MARGIN_E = E.Z_FL_IN[0] - E.TRAVEL_C - (E.Z_SERVO_TOP + E.BRIDGE_T)   # curl flange at full stroke above the servo block
S15 = E.SERVO; S32 = H.SERVOS["STS3032"]; S09 = H.SERVOS["SCS0009"]   # Feetech maker data already in the project
P_STALL_E = 2 * S15["stall_A"] * 12.0

# ------------------------------------------------------------------ option checks
CHK = {}
# A keep
CHK["A"] = dict(F_stall_finger=F_STALL_E, F_rated_finger=F_RATED_E, margin_peak=rnd(F_STALL_E / F_PEAK_NEED, 2), margin_hold=rnd(F_RATED_E / F_HOLD_NEED, 2),
                mass=MASS_E, r_max=R_MAX_E, stack_margin=rnd(STACK_MARGIN_E, 2), stall_W=rnd(P_STALL_E))

# B third nested ring: pair (0,3) on one ring, pair (1,2) on another, curl ring as today
PITCH_R = E.R_OUT[0] - E.R_IN[0]; PITCH_FL = E.R_FL_IN - E.R_FL_OUT; LEVEL_Z = E.Z_FL_OUT[0] - E.Z_FL_IN[0]   # 2.4, 2.5, 16.7 per extra ring
def nested(n_extra):
    return dict(r_max=rnd(R_MAX_E + n_extra * PITCH_FL), d_max=rnd(2 * (R_MAX_E + n_extra * PITCH_FL)), longer=rnd(n_extra * LEVEL_Z),
                ring_r=[rnd(E.R_OUT[0] + PITCH_R), rnd(E.R_OUT[1] + PITCH_R)])
m_ring = E._parts()["ring_out"][0].volume() * 1.25e-3
CHK["B"] = dict(**nested(1), F_stall_finger=rnd(2 * F_STALL_E), margin_peak=rnd(2 * F_STALL_E / F_PEAK_NEED, 2), ring_g=rnd(m_ring, 1),
                cf_ring_N="0 net (closed ring, balanced)", added_g=rnd(S15["m"] + m_ring + 4.0 + 12.0), stall_W=rnd(3 * S15["stall_A"] * 12.0),
                four_rings=nested(3))
CHK["B"]["mass"] = rnd(MASS_E + CHK["B"]["added_g"])

# C tilting base ring on three pull points (swashplate): anchors lie in one plane, cords only pull
tilt = np.degrees(np.arctan(E.TRAVEL_B / (2 * E.R_LINK)))             # one finger at full stroke, the opposite one at zero
D_bore = 2 * E.R_OUT[0]; h_ring = E.Z_OUT[1] - E.Z_OUT[0]
clr_need = D_bore * (1 / np.cos(np.radians(tilt)) - 1) + h_ring * np.tan(np.radians(tilt)); clr_have = 2 * (E.R_OUT[0] - E.R_IN[1])
rim_rise = E.R_FL_OUT * np.tan(np.radians(tilt)) - E.TRAVEL_B / 2      # far rim of the flange above its flight height
head = (C.ZB - 2.4) - E.Z_LIP_OUT[1]                                   # lip top (flight) to the underside of the finger roots
CHK["C"] = dict(tilt_deg=rnd(tilt), bore_clearance_need=rnd(clr_need, 2), bore_clearance_have=rnd(clr_have, 2), fits_ring_guide=bool(clr_need <= clr_have),
                far_rim_rise=rnd(rim_rise, 2), headroom_to_roots=rnd(head, 2), rim_gap=rnd(head - rim_rise, 2),
                patterns="one finger alone, two neighbours, all four; not two opposite, not three (plane: z0 + z2 = z1 + z3)",
                added_g=rnd(2 * S15["m"] + 8.0), stall_W=rnd(4 * S15["stall_A"] * 12.0))
CHK["C"]["mass"] = rnd(MASS_E + CHK["C"]["added_g"])

# D base ring cut into four sectors (same radii and height as today's base ring), four small pullers
SEC_HALF, LIP_HALF_D = 38.0, 12.0
SPOOL_D = 3.0
spool_deg = np.degrees(E.TRAVEL_B / SPOOL_D)
eta_d = E.ETA_SPOOL * E.ETA_GUIDE * E.ETA_IDLER ** 2
F_stall_d = S32["stall"] * eta_d / (SPOOL_D * 1e-3); F_rated_d = S32["rated"] * eta_d / (SPOOL_D * 1e-3)
hw = E.ROD_DAZ + np.degrees(3.2 / E.R_ROD)                             # half width of one puller's rod pair + bosses, deg
curl_az = [E.HALVES[("in", "near")][0] % 360, E.HALVES[("in", "far")][0] % 360]
def min_gap(p):
    az = sorted([(p + 90 * k) % 360 for k in range(4)] + curl_az); return min((b - a) % 360 for a, b in zip(az, az[1:] + [az[0] + 360])) - 2 * hw
p_best = max(np.arange(0, 90, 0.5), key=min_gap); gap_deg = min_gap(p_best)
m_sec = m_ring / 4 + 0.3; r_sec = (E.R_OUT[0] + E.R_FL_OUT) / 2
pack = dict(x=(2 * S32["size"][0], E.SL), y=(S32["size"][2], E.SH), z=(2 * S32["size"][1], E.SW))   # 2 x 2 STS3032 in the upper servo tier
CHK["D"] = dict(sector_half_deg=SEC_HALF, lip_half_deg=LIP_HALF_D, park_tolerance_deg=rnd(SEC_HALF - LIP_HALF_D - 2.0),
                spool_r=SPOOL_D, spool_deg=rnd(spool_deg), spool_deg_have=S32["deg"] - 30.0, eta=rnd(eta_d, 3),
                F_stall_finger=rnd(F_stall_d), F_rated_finger=rnd(F_rated_d), margin_peak=rnd(F_stall_d / F_PEAK_NEED, 2), margin_hold=rnd(F_rated_d / F_HOLD_NEED, 2),
                side_force_rated=rnd(side_force(F_rated_d), 2), side_force_stall=rnd(side_force(F_stall_d), 2),
                puller_half_deg=rnd(hw), puller_az0=rnd(p_best), puller_gap_deg=rnd(gap_deg), puller_gap_mm=rnd(np.radians(gap_deg) * E.R_ROD),
                tier_fit={k: dict(need=rnd(v[0]), have=rnd(v[1]), over=rnd(v[0] - v[1])) for k, v in pack.items()},
                sector_g=rnd(m_sec, 2), cf_sector_N=rnd(cf(m_sec, r_sec)), key_shear_MPa=rnd(cf(m_sec, r_sec) / (2 * 4.0), 1),
                added_rot_g=1.5, added_g=rnd(4 * S32["m"] - S15["m"] + 1.5 + 3.0 + 3.6 + 10.0 + 6.0 + 5.0 + 4.0),
                stall_W=rnd(S15["stall_A"] * 12.0 + 4 * S32["stall_A"] * 6.0), rail_6V_A=rnd(4 * S32["stall_A"]), r_max=R_MAX_E, longer=0.0)
CHK["D"]["mass"] = rnd(MASS_E + CHK["D"]["added_g"])

# E one base ring + four pick-up latches set by one small stationary selector servo (binary: finger joins the stroke or not)
m_key, r_key = 0.4, (E.LUG["r"][0] + E.LUG["r"][1]) / 2
CHK["E"] = dict(key_g=m_key, cf_key_N=rnd(cf(m_key, r_key)), park_tolerance_deg=rnd(np.degrees(0.5 / r_key), 1), F_stall_finger_one=rnd(4 * F_STALL_E),
                added_rot_g=rnd(4 * m_key + 2.0), added_g=rnd(S09["m"] + 4 * m_key + 2.0 + 8.0), stall_W=rnd(P_STALL_E + S09["stall_A"] * 6.0))
CHK["E"]["mass"] = rnd(MASS_E + CHK["E"]["added_g"])

# F finger roots on four coaxial carriers (interleaved 6061 leaves), V-seat dogs at the cross, indexed parked by a roller on the motor bell + pawl
LEAF_T, LEAF_W, SPINDLE_D, R_DOG, DETENT = 1.5, 10.0, 14.0, 22.0, 15.0
root_half = np.degrees(np.arctan(6.0 / 14.5))                          # root grip flat: 12 wide from r 14.5 (hand3_finger.root_grip)
min_space = 2 * root_half
tension = F_FINGER / 2 / ((LEAF_W - H.LAG_TUBE[0]) * LEAF_T); bearing = F_FINGER / 2 / (H.LAG_TUBE[0] * LEAF_T)
span = (H.HUB_GRIP_T + 6 * LEAF_T) / 2                                 # outermost leaf pair: half distance between its two leaves
M_tube = F_FINGER / 2 * (span - H.HUB_GRIP_T / 4) * 1e-3               # N m, tube as a beam between the outer leaves (est.)
Z_tube = np.pi * (H.LAG_TUBE[0] ** 4 - H.LAG_TUBE[1] ** 4) / (32 * H.LAG_TUBE[0])
T_shaft = THR["low"]["P_shaft"] / (2 * np.pi * THR["low"]["rpm"] / 60)  # N m at full thrust
ROLL_R, BELL_R, T_COG = 6.0, 17.0, 0.03                                # roller Ø12 on the Ø34 bell; cogging + dog drag est.
arc4 = [0, min_space, 2 * min_space, 3 * min_space]
CHK["F"] = dict(min_root_spacing_deg=rnd(min_space), four_fingers_within_deg=rnd(3 * min_space), detent_deg=DETENT,
                finger_radial_N=rnd(F_FINGER), finger_radial_proof_N=rnd(F_FINGER_PROOF), leaf_tension_MPa=rnd(tension), leaf_tension_proof_MPa=rnd(tension * 1.3225),
                leaf_bearing_MPa=rnd(bearing), spindle_bearing_MPa=rnd(F_FINGER / 2 / (SPINDLE_D * LEAF_T)), al6061_yield_MPa=276,
                tube_bending_MPa=rnd(M_tube / Z_tube * 1e9 * 1e-6), hub_stack=rnd(8 * LEAF_T), hub_stack_today=rnd(2 * H.HUB["plate_t"] + H.HUB["grip_t"]),
                dog_load_N=rnd(T_shaft / 4 / (R_DOG * 1e-3) * 2), imb_per_0p05mm_seat_N=rnd(imb_az(np.degrees(0.05 / R_DOG)), 2),
                imb_today_0p05mm_pin_N=rnd(imb_az(np.degrees(0.05 / H.R_LAG)), 2), imb_1deg_N=rnd(imb_az(1.0)), imb_one_detent_N=rnd(imb_az(DETENT)),
                imb_3plus1_N=rnd(imb_set([45, 90, 135, 270])), imb_all_one_side_N=rnd(imb_set(arc4)),
                roller_torque_need=rnd(T_COG * ROLL_R / BELL_R, 4), roller_torque_rated=S32["rated"], roller_margin=rnd(S32["rated"] / (T_COG * ROLL_R / BELL_R)),
                index_90deg_s=rnd(0.25 * BELL_R / ROLL_R * 6 * S32["sec60"], 2), com_shift_parked_mm=rnd(imb_set(arc4) / F_FINGER * M_F * R_COM / (MASS_E + 55.0), 1),
                added_rot_g=12.0, added_g=rnd(S32["m"] + S09["m"] + 12.0 + 10.0), stall_W=rnd(P_STALL_E + (S32["stall_A"] + S09["stall_A"]) * 6.0))
CHK["F"]["mass"] = rnd(MASS_E + CHK["F"]["added_g"])

# G same carriers, indexed by the flight motor itself against one pawl (needs position control of the SE 2816)
CHK["G"] = dict(motor_I_max_A=H.MOTOR["I_max"], thrust_I_A=THR["low"]["I_A"], shaft_torque_Nm=rnd(T_shaft, 2), index_torque_need_Nm=T_COG,
                added_rot_g=12.0, added_g=rnd(S09["m"] + 12.0 + 6.0 + 15.0), note="sensored FOC controller rated 55 A phase + shaft encoder replaces the planned ESC (est.)")
CHK["G"]["mass"] = rnd(MASS_E + CHK["G"]["added_g"])

# H servos on the rotor, power through a slip ring
SG_M, SLIP_RPM, SLIP_A = 9.0, 300.0, 2.0                              # 9 g micro servo class; Adafruit 736 slip ring: 6 wires, 2 A, rated to 300 rpm
CHK["H"] = dict(g_at_r20=rnd(G_AT_LAG, 0), servo_cf_N=rnd(cf(SG_M, H.R_LAG)), servo_cf_kgf=rnd(cf(SG_M, H.R_LAG) / 9.81), slip_ring_rated_rpm=SLIP_RPM,
                overspeed=rnd(RPM_MAX / SLIP_RPM), shaft="SE 2816 shaft is solid: no bore for wires", added_rot_g=rnd(4 * SG_M + 8.0),
                imb_1g_at_r20_N=rnd(cf(1.0, H.R_LAG)))

COMMON = dict(rpm_max=RPM_MAX, rpm_hover=RPM_HOVER, g_at_lag_pin=rnd(G_AT_LAG, 0), finger_g=rnd(M_F, 2), finger_com_r=rnd(R_COM), pin_to_com=rnd(D_COM),
              finger_cf_N=rnd(F_FINGER), finger_cf_design_module_N=H.knuckle_loads(RPM_MAX)[0]["F_proof"], lag_restoring_Nm_per_sin=rnd(K_RESTORE, 2),
              lag_restoring_at_47_Nm=rnd(T_RESTORE_47, 2), lag_spring_Nm=E.LAG_SPRING, rpm_restoring_10x_spring=rnd(RPM_10X, 0),
              imbalance_one_finger_stuck_47_N=rnd(IMB_STUCK), base_cord_peak_need_N=rnd(F_PEAK_NEED), base_cord_hold_need_N=rnd(F_HOLD_NEED),
              lag_only_travel=rnd(LAG_TRAVEL, 2), lag_cord_need_N=rnd(F_LAG_NEED, 2), side_force_rated_today=rnd(side_force(F_RATED_E), 2),
              hub_E=dict(mass=MASS_E, r_max=R_MAX_E, stack_margin=rnd(STACK_MARGIN_E, 2), radii=dict(cage=E.R_CAGE, ring_in=E.R_IN, ring_out=E.R_OUT,
                         lip_out=E.R_LIP_OUT, lip_in=E.R_LIP_IN, leg=E.R_LEG, rod=E.R_ROD, bell=17.0), lag_sign=H.LAG_SIGN, spread_max=H.SPREAD_MAX))

# ------------------------------------------------------------------ figure
COL = dict(rot="#8a97a8", rotf="#dfe5ec", stat="#d98a2b", new="#c0392b", new2="#1f7a4d", ghost="#c9ced6", ink="#1c2530", f=["#2c6fbb", "#8e44ad", "#16a085", "#c0392b"])
FPOLY = np.array([(14.5, -6), (31.6, -6), (34, -9.5), (40, -10), (H.R_TIP, -8.6), (H.R_TIP, 8.6), (40, 10), (34, 9.5), (31.6, 6), (14.5, 6)], float)
def Rz(d): a = np.radians(d); return np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]])
def finger_xy(az, lag=0.0):
    p = FPOLY - [H.R_LAG, 0]; p = p @ Rz(lag).T + [H.R_LAG, 0]; return p @ Rz(az).T
def draw_finger(ax, az, lag=0.0, fc="#b9c6d6", ec=COL["ink"], lw=1.0, alpha=1.0, z=4, ls="-"):
    ax.add_patch(Polygon(finger_xy(az, lag), closed=True, fc=fc, ec=ec, lw=lw, alpha=alpha, zorder=z, ls=ls))
    q = Rz(az) @ np.array([H.R_LAG, 0]); ax.add_patch(Circle(q, H.LAG_TUBE[0] / 2, fc="white", ec=ec, lw=0.8, zorder=z + 1, alpha=alpha))
def ring(ax, r, **k): ax.add_patch(Circle((0, 0), r, fill=False, **k))
def band(ax, r0, r1, a0=0, a1=360, **k): ax.add_patch(Wedge((0, 0), r1, a0, a1, width=r1 - r0, **k))
def pol(r, a): return np.array([r * np.cos(np.radians(a)), r * np.sin(np.radians(a))])
def swing(ax, az, lag, col):
    c = Rz(az) @ np.array([H.R_LAG, 0]); a0, a1 = sorted((az, az + lag))
    ax.add_patch(matplotlib.patches.Arc(c, 56, 56, theta1=a0, theta2=a1, color=col, lw=1.6, zorder=8))
    tip = c + 28 * np.array([np.cos(np.radians(az + lag)), np.sin(np.radians(az + lag))]); ax.plot(*tip, marker="o", ms=3.5, color=col, zorder=9)
def base(ax, rmax=R_MAX_E, base_ring=True, yokes=True, plate=True):
    if plate: ax.add_patch(Circle((0, 0), H.HUB["plate_d"] / 2, fc=COL["rotf"], ec=COL["rot"], lw=1.0, zorder=1))
    ring(ax, 17.0, ec=COL["rot"], lw=0.8, ls=":", zorder=2)                                 # motor bell (turns with the hub)
    band(ax, *E.R_CAGE, fc="#b6bfca", ec="none", zorder=2); band(ax, *E.R_IN, fc="#9fb4cc", ec="none", zorder=2)
    if base_ring: band(ax, *E.R_OUT, fc="#6f8fb5", ec="none", zorder=2)
    if yokes:
        for (w, s), (az, xy, _, _) in E.HALVES.items():
            lr = E.R_LIP_OUT if w == "out" else E.R_LIP_IN
            band(ax, lr[0], lr[1], az - E.LIP_HALF, az + E.LIP_HALF, fc=COL["stat"], ec="none", alpha=0.75, zorder=3)
            for k in (-1, 1): ax.add_patch(Circle(E.rod_xy(az, k), 1.5, fc=COL["stat"], ec="none", zorder=3))
    ring(ax, rmax, ec="#555", lw=0.7, ls="--", zorder=1)
def note(ax, x, y, s, col=COL["ink"], ha="left", va="center", fs=7.6, box=True):
    ax.text(x, y, s, color=col, ha=ha, va=va, fontsize=fs, zorder=20, linespacing=1.15,
            bbox=dict(fc="white", ec="none", alpha=0.82, pad=1.2) if box else None)

LIM = 56.0
def panel_A(ax):
    base(ax)
    for k in range(4):
        draw_finger(ax, H.FINGER_AZ[k], 0, fc="none", ec=COL["ghost"], ls="--", z=3)
        draw_finger(ax, H.FINGER_AZ[k], H.LAG_SIGN[k] * H.SPREAD_MAX); swing(ax, H.FINGER_AZ[k], H.LAG_SIGN[k] * H.SPREAD_MAX, COL["f"][0])
    return "One base ring pulls all four: neighbours mirror each other and form two pairs (dashed = flight cross)."
def panel_B(ax):
    r = CHK["B"]["r_max"]; base(ax, rmax=r)
    band(ax, *CHK["B"]["ring_r"], fc=COL["new"], ec="none", zorder=3); ring(ax, R_MAX_E, ec="#999", lw=0.6, ls=":", zorder=1)
    lag = [47, 0, 0, 47]
    for k in range(4):
        draw_finger(ax, H.FINGER_AZ[k], H.LAG_SIGN[k] * lag[k], fc="#b9c6d6" if lag[k] == 0 else "#f1b9b1")
        if lag[k]: swing(ax, H.FINGER_AZ[k], H.LAG_SIGN[k] * lag[k], COL["new"])
    return f"Third ring (red) moves pair F1 + F4 alone. Envelope grows to Ø{2 * r:.0f} (dotted = today), hand {CHK['B']['longer']:.1f} mm longer."
def panel_C(ax):
    base(ax, yokes=False)
    for i, a in enumerate((90, 210, 330)):
        band(ax, E.R_LIP_OUT[0], E.R_LIP_OUT[1], a - 14, a + 14, fc=COL["new"], ec="none", zorder=3)
        for k in (-1, 1): ax.add_patch(Circle(E.rod_xy(a, k), 1.5, fc=COL["new"], ec="none", zorder=3))
    ax.plot(*np.array([pol(46, 135), pol(46, 315)]).T, color=COL["new2"], lw=1.2, ls="-.", zorder=6)
    lag = [47, 0, 0, 0]
    for k in range(4):
        draw_finger(ax, H.FINGER_AZ[k], H.LAG_SIGN[k] * lag[k], fc="#b9c6d6" if lag[k] == 0 else "#f1b9b1")
        if lag[k]: swing(ax, H.FINGER_AZ[k], H.LAG_SIGN[k] * lag[k], COL["new"])
    return f"Three pull points (red) tilt the base ring {CHK['C']['tilt_deg']:.0f}° about the green axis: one finger, or two neighbours, move alone."
def panel_D(ax):
    base(ax, base_ring=False, yokes=False)
    lag = [47, 20, 0, 34]
    for k in range(4):
        a = H.FINGER_AZ[k]; band(ax, E.R_OUT[0], E.R_FL_OUT, a - SEC_HALF, a + SEC_HALF, fc=COL["f"][k], ec="none", alpha=0.85, zorder=3)
        pa = p_best + 90 * k
        band(ax, E.R_FL_OUT + 0.6, E.R_LEG[1], pa - LIP_HALF_D, pa + LIP_HALF_D, fc=COL["stat"], ec=COL["f"][k], lw=1.2, zorder=3)
        for s in (-1, 1): ax.add_patch(Circle(E.rod_xy(pa, s), 1.5, fc=COL["stat"], ec="none", zorder=3))
        draw_finger(ax, a, H.LAG_SIGN[k] * lag[k], fc=matplotlib.colors.to_rgba(COL["f"][k], 0.35), ec=COL["f"][k], lw=1.3)
        if lag[k]: swing(ax, a, H.LAG_SIGN[k] * lag[k], COL["f"][k])
    for az in curl_az:
        for s in (-1, 1): ax.add_patch(Circle(E.rod_xy(az, s), 1.5, fc="#777", ec="none", zorder=3))
    return "Base ring cut into four flange sectors (colours), one puller each (orange, outlined): here 47°, 20°, 0° and 34° at once."
def panel_E(ax):
    base(ax)
    sel = [1, 1, 0, 0]
    for k in range(4):
        a = H.FINGER_AZ[k]; q = pol(r_key, a - E.LINK_AZ)
        ax.add_patch(Rectangle(q - 2.2, 4.4, 4.4, fc=COL["new2"] if sel[k] else "white", ec=COL["new2"], lw=1.3, zorder=9))
        draw_finger(ax, a, H.LAG_SIGN[k] * 47 * sel[k], fc="#bfe3d0" if sel[k] else "#b9c6d6")
        if sel[k]: swing(ax, a, H.LAG_SIGN[k] * 47, COL["new2"])
    s = pol(44, 100); ax.add_patch(Rectangle(s - [11.6, 6], 23.2, 12.1, fc=COL["stat"], ec=COL["ink"], lw=0.8, zorder=9, angle=0))
    ax.annotate("selector servo", s + [-14, 2], xytext=(-92, 52), color=COL["stat"], fontsize=7.6, arrowprops=dict(arrowstyle="-", color=COL["stat"], lw=0.8), zorder=20)
    return "One latch per finger (filled green = joins the base stroke): F1 and F2 swing, F3 and F4 stay at the cross."
def carriers(ax, az, pawl=True, roller=True, psgn=1, plab=(62, -44)):
    base(ax, plate=False)
    ax.add_patch(Circle((0, 0), SPINDLE_D / 2, fc="#666", ec="none", zorder=6))
    for a in np.arange(0, 360, DETENT): ax.plot(*np.array([pol(R_DOG - 1.2, a), pol(R_DOG + 1.2, a)]).T, color="#444", lw=0.7, zorder=5)
    for k in range(4): draw_finger(ax, H.FINGER_AZ[k], 0, fc="none", ec=COL["ghost"], ls="--", z=3)
    for k in range(4):
        band(ax, SPINDLE_D / 2, H.R_LAG + 4.5, az[k] - 13, az[k] + 13, fc=COL["f"][k], ec="none", alpha=0.55, zorder=4)
        draw_finger(ax, az[k], 0, fc=matplotlib.colors.to_rgba(COL["f"][k], 0.35), ec=COL["f"][k], lw=1.3, z=5)
        ax.plot(*pol(R_DOG, az[k]), marker="v", ms=4.5, color=COL["f"][k], zorder=9)
    if roller:
        c = pol(BELL_R + ROLL_R, 185); ax.add_patch(Circle(c, ROLL_R, fc=COL["stat"], ec=COL["ink"], lw=0.8, zorder=9))
        ax.annotate("roller on the\nmotor bell", c + [-6, 0], xytext=(-93, -6), color=COL["stat"], fontsize=7.6, va="center", arrowprops=dict(arrowstyle="-", color=COL["stat"], lw=0.8), zorder=20)
    if pawl:
        p = np.array([pol(25.5, az[3] + psgn * 15), pol(47, az[3] + psgn * 30)]); ax.plot(*p.T, color=COL["new"], lw=4, solid_capstyle="butt", zorder=10)
        ax.annotate("pawl", p[1], xytext=plab, color=COL["new"], fontsize=7.6, arrowprops=dict(arrowstyle="-", color=COL["new"], lw=0.8), zorder=20)
def panel_F(ax):
    carriers(ax, [45, 90, 135, 270])
    return f"Each root rides its own carrier (coloured) and locks in {DETENT:.0f}° seats (ticks). Shown: 3 + 1 thumb grasp; dashed = flight cross."
def panel_G(ax):
    a = CHK["F"]["min_root_spacing_deg"]
    carriers(ax, [90 - 0.5 * a, 90 + 0.5 * a, 90 + 1.5 * a, 90 - 1.5 * a], roller=False, psgn=-1, plab=(66, -34))
    return f"Same carriers; the flight motor turns the hub against the pawl. Shown: all four inside {3 * a:.0f}° (roots {a:.0f}° apart)."
def panel_H(ax):
    base(ax, yokes=False)
    for k in range(4):
        a = H.FINGER_AZ[k]; draw_finger(ax, a, H.LAG_SIGN[k] * [47, 15, 30, 0][k])
        c = pol(H.R_LAG - 1, a + 38); ax.add_patch(Polygon((np.array([(-11.5, -6.1), (11.5, -6.1), (11.5, 6.1), (-11.5, 6.1)]) @ Rz(a + 128).T) + c, fc="#f4c542", ec=COL["ink"], lw=0.8, zorder=9))
    ax.add_patch(Circle((0, 0), 11.0, fc="#333", ec="none", zorder=10)); note(ax, 0, 0, "slip\nring", col="white", ha="center", box=False, fs=6.5)
    ax.plot([-60, 60], [-60, 60], color=COL["new"], lw=3, alpha=0.55, zorder=15); ax.plot([-60, 60], [60, -60], color=COL["new"], lw=3, alpha=0.55, zorder=15)
    return f"Servos (yellow) ride the rotor: a 9 g servo at r 20 pulls {CHK['H']['servo_cf_N']:.0f} N; the slip ring is rated {SLIP_RPM:.0f} rpm."

D_, B_, C_, E_, F_, G_, H_, A_ = (CHK[k] for k in "DBCEFGHA")
ROWS = [
 dict(key="A", name="Keep hub E\n(two rings, mirror spread)", verdict="baseline", draw=panel_A, cells=[
   "All four together. Neighbours swing opposite ways into two pairs, 0-47°. Roots fixed at 45/135/225/315°.",
   "2 x STS3215, not rotating (base ring: spread + K1; curl ring: K2 + K3).",
   "Nothing. Yoke lips hook the ring flange across a 1 mm gap.",
   f"0 g. Hand {A_['mass']:.0f} g.",
   f"Self-centring: centrifugal pull returns each finger to the cross with {COMMON['lag_restoring_at_47_Nm']:.1f} N m at 47° (lag spring {E.LAG_SPRING * 1e3:.1f} mN m).",
   f"No new parts. Each finger pulls {COMMON['finger_cf_N']:.0f} N on its lag pin ({COMMON['g_at_lag_pin']:,.0f} x gravity at r 20).",
   f"Base cord {A_['F_stall_finger']:.0f} N stall / {A_['F_rated_finger']:.0f} N rated per finger; need {F_PEAK_NEED:.0f} / {F_HOLD_NEED:.0f} N: margin {A_['margin_peak']:.2f} / {A_['margin_hold']:.2f}. Stall {A_['stall_W']:.0f} W at 12 V.",
   "Nothing.",
   f"Ø{2 * R_MAX_E:.1f}, no change. Stack margin {A_['stack_margin']:.1f} mm.",
   "Does not do what was asked."]),
 dict(key="B", name="Third nested ring\n(pairs split)", verdict="works, partial", draw=panel_B, cells=[
   "Each PAIR on its own (F1+F4, F2+F3); inside a pair still mirrored. Works at any rotor angle.",
   "3 x STS3215, not rotating.",
   "Nothing (third hooked flange).",
   f"+{B_['ring_g']:.1f} g rotating ring; hand about {B_['mass']:.0f} g (+{B_['added_g']:.0f}, est.).",
   "Self-centring as A. Ring is closed and round: balanced.",
   "Closed ring: hoop load only, no net force.",
   f"One servo per two fingers: {B_['F_stall_finger']:.0f} N stall per finger, margin {B_['margin_peak']:.2f}. Stall {B_['stall_W']:.0f} W.",
   "1 x STS3215, 2 x Ø3 carbon rod, 2 x MR63 bearing.",
   f"Ø{B_['d_max']:.1f} (+{2 * PITCH_FL:.0f}); hand +{B_['longer']:.1f} mm long (one more flange level). Four rings: Ø{B_['four_rings']['d_max']:.0f}, +{B_['four_rings']['longer']:.0f} mm: rejected.",
   "Longer wrist lever for a half answer; a ring per finger does not fit."]),
 dict(key="C", name="Tilting base ring\n(3 pull points)", verdict="tight, partial", draw=panel_C, cells=[
   "One finger alone, two neighbours, or all four. Not two opposite, not three (cord anchors stay in one plane).",
   "4 x STS3215, not rotating (3 on the base ring, 1 curl).",
   "Nothing.",
   f"about 0 g rotating; hand about {C_['mass']:.0f} g (+{C_['added_g']:.0f}, est.).",
   "Self-centring as A; ring must sit flat before spin-up (3 servo positions).",
   "No new rotor parts; a tilted ring would wobble, so flat is a spin-up condition.",
   f"Tilt {C_['tilt_deg']:.0f}° needs {C_['bore_clearance_need']:.1f} mm bore clearance, ring has {C_['bore_clearance_have']:.1f}: new ball-type guide. Rim rises {C_['far_rim_rise']:.1f} mm, {C_['rim_gap']:.1f} mm left under the roots. Stall {C_['stall_W']:.0f} W.",
   "2 x STS3215, 2 x Ø3 rod, MR63 bearings.",
   "Ø unchanged; servo block one servo wider or taller (est.).",
   "Ring jams or keys bind when tilted; only some patterns."]),
 dict(key="D", name="Base ring cut into\nfour sectors, 4 pullers", verdict="RECOMMENDED", draw=panel_D, cells=[
   "EACH finger on its own: any spread 0-47°, then its own K1 fold. Curl (K2 + K3) stays shared. Roots stay at the cross.",
   "4 x STS3032 (20 g) + 1 x STS3215, all not rotating. The four small servos fill today's upper servo bay.",
   f"Nothing. Four lips hook four flange sectors; rotor parks within ±{D_['park_tolerance_deg']:.0f}°.",
   f"+{D_['added_rot_g']:.1f} g rotating (keys); hand about {D_['mass']:.0f} g (+{D_['added_g']:.0f}, est.).",
   f"Self-centring as A: by {COMMON['rpm_restoring_10x_spring']:,.0f} rpm the centrifugal return is 10 x the lag spring. Rule: all pullers released before spin-up.",
   f"Each sector {D_['sector_g']:.1f} g at r {r_sec:.0f}: {D_['cf_sector_N']:.0f} N on its key ({D_['key_shear_MPa']:.0f} MPa on 8 mm², PA-CF). Four equal sectors: balanced.",
   f"STS3032 on Ø{2 * SPOOL_D:.0f} spool: {D_['F_stall_finger']:.0f} N stall / {D_['F_rated_finger']:.0f} N rated; margin {D_['margin_peak']:.2f} / {D_['margin_hold']:.2f} (target 1.25 / 1.0). Stall {D_['stall_W']:.0f} W; 6 V rail {D_['rail_6V_A']:.1f} A.",
   "4 x Feetech STS3032, 4 x Ø3 carbon rod, 8 x MR63, 6 V 5 A buck regulator (est.).",
   f"Ø{2 * R_MAX_E:.1f} and height unchanged (est.). Servo bay {D_['tier_fit']['x']['need']:.1f} vs {D_['tier_fit']['x']['have']:.1f} mm: +{D_['tier_fit']['x']['over']:.1f} mm. Rod pairs {D_['puller_gap_mm']:.1f} mm apart.",
   "A sector cocks on its key under an off-centre pull; sideways pinch is weak (" + f"{D_['side_force_rated']:.1f} N)."]),
 dict(key="E", name="One ring + pick-up\nlatch per finger", verdict="works, on/off only", draw=panel_E, cells=[
   "Each finger is IN or OUT of the base stroke. Those that are in move together by the same amount.",
   "2 x STS3215 + 1 x SCS0009 selector (13 g), not rotating.",
   f"Nothing; the selector flips latches only when parked within ±{E_['park_tolerance_deg']:.1f}°.",
   f"+{E_['added_rot_g']:.1f} g rotating; hand about {E_['mass']:.0f} g (+{E_['added_g']:.0f}, est.).",
   "Self-centring as A. A latch that flips in flight does nothing (cords are slack).",
   f"Each {m_key:.1f} g latch at r {r_key:.0f}: {E_['cf_key_N']:.0f} N, taken by the cage slot; it must slide along the axis so the load cannot flip it.",
   f"One finger alone gets the whole ring: {E_['F_stall_finger_one']:.0f} N stall (TPU link limits it). Stall {E_['stall_W']:.0f} W.",
   "1 x Feetech SCS0009; 4 printed latches, Ø1.5 pins.",
   "Ø and height unchanged (est.).",
   "±1° parking again (rev D's weak point); small latches at 3,500 x gravity; no in-between angles."]),
 dict(key="F", name="Roots on carrier rings,\nroller + pawl index", verdict="only way to move roots", draw=panel_F, cells=[
   f"Each ROOT travels round the hub in {DETENT:.0f}° steps: 3 + 1 thumb, 2 + 2, all four inside {F_['four_fingers_within_deg']:.0f}°. Roots no closer than {F_['min_root_spacing_deg']:.0f}°. Spread and curl as A.",
   "2 x STS3215 + STS3032 roller drive + SCS0009 cam (roller in, pawl in). Not rotating.",
   "Nothing. Roller and pawl touch the rotor only when parked.",
   f"+{F_['added_rot_g']:.0f} g rotating (est.); hand about {F_['mass']:.0f} g (+{F_['added_g']:.0f}, est.).",
   f"NOT self-centring. Must be back at the cross and locked before spin-up: 1° off = {F_['imb_1deg_N']:.0f} N, one step off = {F_['imb_one_detent_N']:.0f} N, 3 + 1 = {F_['imb_3plus1_N']:.0f} N. V-seats at 0.05 mm: {F_['imb_per_0p05mm_seat_N']:.1f} N (today's pins: {F_['imb_today_0p05mm_pin_N']:.1f} N).",
   f"Each carrier takes its finger's {F_['finger_radial_N']:.0f} N ({F_['finger_radial_proof_N']:.0f} N proof). 1.5 mm 6061 leaves: {F_['leaf_tension_MPa']:.0f} MPa tension, {F_['leaf_bearing_MPa']:.0f} MPa bearing (yield 276). Lag tube bending {F_['tube_bending_MPa']:.0f} MPa: too high, needs a stiffer pin.",
   f"Roller needs {F_['roller_torque_need'] * 1e3:.0f} mN m, has {F_['roller_torque_rated'] * 1e3:.0f}: x{F_['roller_margin']:.0f}. 90° in {F_['index_90deg_s']:.1f} s. Dogs carry {F_['dog_load_N']:.0f} N. Stall {F_['stall_W']:.0f} W.",
   "STS3032, SCS0009, laser-cut 6061 leaves (8), Ø14 spindle, 4 dog pins + springs, Hall sensor + 4 magnets.",
   f"Hub stack {F_['hub_stack']:.0f} mm (today {F_['hub_stack_today']:.0f}); Ø unchanged (est.). New hub, cage and cord followers.",
   "A missed lock is a destroyed hand; whole rotor hub redesigned and re-proofed."]),
 dict(key="G", name="Carrier rings, flight\nmotor indexes vs pawl", verdict="rejected (controller)", draw=panel_G, cells=[
   "As F.",
   "2 x STS3215 + SCS0009 pawl. The SE 2816 itself turns the hub to each new angle.",
   "Nothing.",
   f"+{G_['added_rot_g']:.0f} g rotating (est.); hand about {G_['mass']:.0f} g (est.).",
   "As F: locked cross is a hard spin-up condition.",
   "As F.",
   f"Motor has {G_['shaft_torque_Nm']:.2f} N m, indexing needs about {T_COG:.2f}: plenty. But the planned ESC cannot hold an angle at zero speed.",
   f"Sensored FOC drive for {G_['thrust_I_A']:.0f} A + shaft encoder in place of the ESC (est.), SCS0009, carrier parts as F.",
   "As F.",
   "Changes the motor drive of a flight-critical rotor for a parked-only feature."]),
 dict(key="H", name="Servos on the rotor,\nslip ring", verdict="rejected (physics)", draw=panel_H, cells=[
   "Anything, in principle.",
   "4 micro servos ON the rotor.",
   f"Slip ring for power + data: 6 wires, 2 A, rated {SLIP_RPM:.0f} rpm (Adafruit 736). Rotor runs {RPM_MAX:,.0f} rpm: {H_['overspeed']:.0f} x over.",
   f"+{H_['added_rot_g']:.0f} g rotating.",
   f"Every 1 g out of place at r 20 is {H_['imb_1g_at_r20_N']:.0f} N of shake.",
   f"A 9 g servo at r 20 weighs {H_['servo_cf_N']:.0f} N ({H_['servo_cf_kgf']:.0f} kgf).",
   "Not sized: fails before torque matters.",
   "None that survive.",
   "n/a",
   "Already rejected in the hub rethink (option E there); the motor shaft is solid."]),
]
HEAD = ["Option", "What each finger can do", "Actuators (rotating or not)", "What crosses to the spinning side", "Added mass", "Balance in flight",
        f"Centrifugal load at {RPM_MAX:,.0f} rpm", "Force / torque: have vs need", "Parts to buy", "Hub Ø / height", "Main risk"]
CW = np.array([1.62, 2.05, 1.7, 1.55, 1.3, 2.2, 2.2, 2.25, 1.75, 1.6, 1.65]); FIG_W = 20.0
CW = CW / CW.sum() * (FIG_W - 0.5)
FS, CPI = 7.7, 15.6                                           # table font size (pt) and characters per inch used for wrapping
LH = FS * 1.2 * 1.22 / 72                                           # line height, in
def wrap(s, w_in): return "\n".join("\n".join(textwrap.wrap(p, max(8, int(w_in * CPI - 1)))) for p in s.split("\n"))
table = [[wrap(h, w) for h, w in zip(HEAD, CW)]]
for r in ROWS: table.append([wrap(c, w) for c, w in zip([f"{r['key']}  {r['name'].replace(chr(10), ' ')}\n[{r['verdict']}]"] + r["cells"], CW)])
RH = [max(c.count("\n") + 1 for c in row) * LH + 0.11 for row in table]
T_H = sum(RH); P_H = 3.45; TOP = 1.02; FOOT = 0.42; GAPV = 0.16
FIG_H = TOP + 2 * P_H + GAPV + T_H + FOOT

fig = plt.figure(figsize=(FIG_W, FIG_H), dpi=85, facecolor="white")
fig.text(0.5, 1 - 0.30 / FIG_H, "hand3 hub E: how can each of the four fingers move round the hub ring on its own?   Options study, plan views to scale (mm), rotor axis at the centre",
         ha="center", va="center", fontsize=15, weight="bold", color=COL["ink"])
req = (f"Must carry: fingers are the rotor blades ({RPM_HOVER:,.0f} rpm hover, {RPM_MAX:,.0f} max, est.) = {COMMON['g_at_lag_pin']:,.0f} x gravity at the lag pins; each {M_F:.1f} g finger pulls {F_FINGER:.0f} N.   "
       f"Hand works only with the rotor parked.   No wires across the spinning joint.   Bought parts with maker data only.   Printed PA-CF.   "
       f"Today: Ø{2 * R_MAX_E:.1f}, {MASS_E:.0f} g, {STACK_MARGIN_E:.1f} mm left in the ring stack.")
fig.text(0.5, 1 - 0.62 / FIG_H, req, ha="center", va="center", fontsize=9.3, color="#333")
fig.text(0.5, 1 - 0.86 / FIG_H, "Grey/blue = turns with the rotor (hub plate, motor bell dotted, cage and rings)   |   orange = stands still (yoke lips, guide rods, servos)   |   red/green/colours = what the option adds or moves   |   "
         "dashed circle = hand envelope   |   fingers run off the panel (tip is at r 114)", ha="center", va="center", fontsize=8.3, color="#555")
pw = (FIG_W - 0.5) / 4
for i, r in enumerate(ROWS):
    cx, cy = i % 4, i // 4
    w_ax, h_ax = pw - 0.22, P_H - 0.78
    x0 = 0.25 + cx * pw + 0.11; y0 = FIG_H - TOP - (cy + 1) * P_H + 0.50
    ax = fig.add_axes([x0 / FIG_W, y0 / FIG_H, w_ax / FIG_W, h_ax / FIG_H]); XL = LIM * w_ax / h_ax
    ax.set_xlim(-XL, XL); ax.set_ylim(-LIM, LIM); ax.set_aspect("equal"); ax.set_xticks([]); ax.set_yticks([])
    rec = r["key"] == "D"; bad = "rejected" in r["verdict"]
    for sp in ax.spines.values(): sp.set_edgecolor("#1f7a4d" if rec else "#b0b6be"); sp.set_linewidth(2.6 if rec else 0.9)
    if rec: ax.set_facecolor("#f0f8f2")
    cap = r["draw"](ax)
    ax.plot([XL - 26, XL - 6], [-LIM + 5, -LIM + 5], color=COL["ink"], lw=2, zorder=20); ax.text(XL - 16, -LIM + 6.5, "20 mm", ha="center", va="bottom", fontsize=6.5, zorder=20)
    ax.set_title(f"{r['key']}   {r['name'].replace(chr(10), ' ')}   [{r['verdict']}]", fontsize=10.2, weight="bold", color="#1f7a4d" if rec else ("#9b2c2c" if bad else COL["ink"]), pad=4)
    fig.text((x0 + w_ax / 2) / FIG_W, (y0 - 0.06) / FIG_H, "\n".join(textwrap.wrap(cap, 86)), ha="center", va="top", fontsize=8.0, color="#333", linespacing=1.2)

# table
tx0 = 0.25; ty = FOOT + T_H
tab = fig.add_axes([tx0 / FIG_W, FOOT / FIG_H, (FIG_W - 0.5) / FIG_W, T_H / FIG_H]); tab.set_xlim(0, FIG_W - 0.5); tab.set_ylim(0, T_H); tab.axis("off")
y = T_H; xs = np.r_[0, np.cumsum(CW)]
for i, (row, h) in enumerate(zip(table, RH)):
    key = None if i == 0 else ROWS[i - 1]["key"]
    fc = "#2b3a4a" if i == 0 else ("#dff1e4" if key == "D" else ("#f7e9e9" if "rejected" in ROWS[i - 1]["verdict"] else ("#f6f7f9" if i % 2 else "white")))
    tab.add_patch(Rectangle((0, y - h), FIG_W - 0.5, h, fc=fc, ec="#9aa3ad", lw=0.6))
    if key == "D": tab.add_patch(Rectangle((0, y - h), FIG_W - 0.5, h, fill=False, ec="#1f7a4d", lw=2.2, zorder=5))
    for j, c in enumerate(row):
        tab.text(xs[j] + 0.05, y - 0.055, c, ha="left", va="top", fontsize=FS, color="white" if i == 0 else COL["ink"], weight="bold" if (i == 0 or j == 0) else "normal", linespacing=1.22)
        if j: tab.plot([xs[j], xs[j]], [y - h, y], color="#c3c9d0", lw=0.5)
    y -= h
fig.text(0.5, 0.2 / FIG_H, "Numbers: dimensions, strokes, servo data and finger masses are read from hand3_design.py / hand3_cad.py / hand3_hubc.py and their json files; forces are m·ω²·r at 12,500 rpm; "
         "masses of new parts, efficiencies, cogging torque and fits marked est. are first estimates, not CAD.  Fits are range (interval) checks.  Full list: hand3_indep_fingers_study.json",
         ha="center", va="center", fontsize=8, color="#555")
out = os.path.join(HERE, "hand3_indep_fingers_study.png"); fig.savefig(out, dpi=85, facecolor="white")

json.dump(dict(question="each hand3 finger moves round the hub ring independently", date="2026-11-19", recommended="D", common=COMMON, options=CHK,
               table=[dict(key=r["key"], name=r["name"].replace("\n", " "), verdict=r["verdict"], **dict(zip(HEAD[1:], r["cells"]))) for r in ROWS]),
          open(os.path.join(HERE, "hand3_indep_fingers_study.json"), "w"), indent=1, default=lambda o: o.tolist() if hasattr(o, "tolist") else float(o))
print(out, f"{FIG_W:.0f} x {FIG_H:.1f} in"); print(json.dumps(COMMON, default=float)[:900])
for k, v in CHK.items(): print(k, json.dumps(v, default=float))
