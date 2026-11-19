"""Fly the full Cephalo-1 rev 4 with hand3 (standard hand) on arms 2 and 6, XL claw-props on the other six.
usage: python3 hand3_fly.py <low|mid|high> <program>      (CEPH_PACKS=2 current build)
Hand-arm model: pod 0.570 kg (cylinder 61 % at 0..30 mm + rotor plane 39 % at 92 mm -> COM 45 mm), disc R 127 mm,
blade plane 92 mm from the spigot, full-throttle thrust 21.3/23.35/25.4 N at 22.2 V (scales (V/22.2)^1.5), K 10.1 W/N^1.5.
Ground contact: the heel sphere is put where the XL toe is, so stance/IK are unchanged (real heel is 110 mm out: needs a
per-arm foot lead later).  Rotor reaction torque uses the XL q/T = 0.0173 m (hand3 value not measured)."""
import os, sys, json
os.environ.setdefault("CEPH_PROP", "xl"); os.environ.setdefault("CEPH_PACKS", "2"); os.environ.setdefault("CEPH_J1", "STS3250")
import numpy as np
import hand3_params as HP
T_ENV = HP.T[sys.argv[1]]
os.environ['CEPH_HAND_K'] = str(HP.K_HAND)
os.environ['CEPH_HAND_T'] = str(T_ENV); os.environ['CEPH_HAND_ARMS'] = '' if sys.argv[1]=='none' else '2,6'; os.environ['CEPH_MON_PLOT']='0'; os.environ['CEPH_MON_OUT']='monitor_out_'+sys.argv[1]+'_'+(sys.argv[2] if len(sys.argv)>2 else 'liftoff')
import cephalo4 as C
import monitor4; monitor4.install()
import xlpod4; xlpod4.install()
import sim4 as S4
import body4h as B4

LVL = sys.argv[1]; PROG = sys.argv[2] if len(sys.argv) > 2 else "liftoff"
T_H = 0.0 if LVL == "none" else HP.T[LVL]
M_HAND, K_HAND, BO_H, R_H, HANDS = HP.M_HAND, HP.K_HAND, HP.BO_H, HP.R_H, HP.HANDS
if LVL != "none":
    for i in HANDS: B4.HAND[i] = dict(T=T_H, K=K_HAND); B4.BO[i] = BO_H; B4.QT[i] = HP.QT[LVL]   # Q = P_shaft/omega at full throttle / T (hand3_thrust_cad.json)
    dm = M_HAND - C.M_POD
    C.M_ROBOT += len(HANDS) * dm; C.W_ROBOT = C.M_ROBOT * C.G
    _orig = S4.arm_xml
    def arm_xml(i):
        x = _orig(i)
        if i not in HANDS: return x
        a, b = f"{C.M_POD * 0.8:.4f}", f"{C.M_POD * 0.2:.4f}"
        assert a in x and b in x
        x = x.replace(f'mass="{a}"', f'mass="{M_HAND * (1 - HP.F_ROTOR):.4f}"').replace(f'mass="{b}"', f'mass="{M_HAND * HP.F_ROTOR:.4f}"')
        old = f'fromto="{C.BLADE_OFF - 0.002:.4f} 0 0 {C.BLADE_OFF + 0.002:.4f} 0 0" size="{C.CLAW_R:.4f}"'
        assert old in x
        return x.replace(old, f'fromto="{BO_H - 0.002:.4f} 0 0 {BO_H + 0.002:.4f} 0 0" size="{R_H:.4f}"')
    S4.arm_xml = arm_xml
if os.environ.get('H3_SPIN','opp')=='opp': B4.SPIN[:] = [1,-1,-1,1,-1,1,1,-1]     # hands 2 and 6 opposite-handed; 8-prop, 6-prop and quad-mode sums all zero
print(f"robot {C.M_ROBOT:.2f} kg  W {C.W_ROBOT:.1f} N  hand thrust {T_H} N")
P = B4.PROGRAMS[PROG]
start = "lying" if P[0][0] in ("rest", "get_up") else "stand"
if PROG == "step":
    S4.EXTRA_XML = '<geom name="ground_step" type="box" pos="0 1.6 0.06" size="0.5 0.5 0.06" rgba="0.75 0.7 0.6 1" contype="1" conaffinity="1" friction="1.0 0.01 0.001"/>'
B = B4.Body(P, start)
print("load fraction 6 / 8 props:", [round(B.load_fraction(n), 3) for n in (6, 8)])
frames, stats = B.run()
for k, s in stats.items():
    print(f"{k:11s} {s['t0']:5.1f}-{s['t1']:5.1f} s | servo {s['servo']:.2f} | tilt {s['tilt']:4.1f} | sat {s['sat']} | gov min {s['rate_min']:.2f} | fan max {s['fan']:.1f} N | {s.get('at','')}")
# electrical / thrust channels
t = np.array([f["t"] for f in frames]); fan = np.array([f["fan"] for f in frames]); V = np.array([f["V"] for f in frames])
I = np.array([f["I"] for f in frames]); edf = np.array([f["edf"] for f in frames]); edfcap = np.array([f["edfcap"] for f in frames])
Kc = np.array([K_HAND if (i in HANDS and LVL != "none") else 5.1 for i in range(8)])
ceil = np.array([[T_H * min(1, (v / 22.2) ** 1.5) if (i in HANDS and LVL != "none") else min(28.0, 26.5 * (v / 21.3) ** 1.5) for i in range(8)] for v in V])
ratio = fan / ceil
esc = Kc * fan ** 1.5 / V[:, None]
esc_lim = np.array([60.0 if i in HANDS else 45.0 for i in range(8)])
edf_A = 11.2 * edf ** 1.5 / V
air = np.array([f["air"] for f in frames]) > 0.5
tilt = np.array([f["tilt"] for f in frames]); servo = np.array([f["servo_arm"] for f in frames])
if os.environ.get("H3_DUMP"): B4.save(frames, stats, f"h3d_{LVL}_{PROG}")
res = dict(level=LVL, prog=PROG, robot_kg=C.M_ROBOT, hand_T=T_H,
    min_V=float(V.min()), peak_pack_A=float(I.max()), peak_edf_A=float(edf_A.max()), edf_ratio_max=float((edf / edfcap).max()),
    fan_ratio_max=[float(x) for x in ratio.max(0)], fan_ratio_max_air=[float(x) for x in ratio[air].max(0)] if air.any() else None,
    hand_fan_max_N=[float(fan[:, i].max()) for i in HANDS], hand_ceiling_min_N=[float(ceil[:, i].min()) for i in HANDS],
    esc_peak_A=[float(x) for x in esc.max(0)], esc_over=[float((esc[:, i] / esc_lim[i]).max()) for i in range(8)],
    tilt_max=float(tilt.max()), tilt_max_air=float(tilt[air].max()) if air.any() else None,
    servo_max=float(servo.max()), servo_hand_arms=[float(servo[:, i].max()) for i in HANDS],
    sat_max=int(max(f["sat"] for f in frames)), z_max=float(max(f["qpos"][2] for f in frames)),
    soc_end=float(frames[-1]["soc"]), refused=bool(B.refused), t_end=float(t[-1]))
print(json.dumps(res, indent=1))
json.dump(res, open(f"h3fly_{LVL}_{PROG}.json", "w"), indent=1)
np.savez_compressed(f"h3fly_{LVL}_{PROG}.npz", t=t, fan=fan, ceil=ceil, V=V, I=I, edf=edf, tilt=tilt, servo=servo, z=np.array([f["qpos"][2] for f in frames]))
