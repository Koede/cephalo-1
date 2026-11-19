"""Quad mode (shovel + dishes) with hand3 on arms 2 and 6 (both fly in the candle pose).  python3 quad_h3.py <low|mid|high|none> [gains.json]"""
import os, sys, json
LVL = sys.argv[1]
import hand3_params as HP
T_ENV = HP.T[LVL]
os.environ.update(CEPH_PROP="xl", CEPH_PACKS="2", CEPH_HAND_T=str(T_ENV), CEPH_HAND_K=str(HP.K_HAND), CEPH_HAND_ARMS="" if LVL == "none" else "2,6", CEPH_MON_PLOT="0", CEPH_MON_OUT="monitor_out_quad_" + os.environ.get("QUAD_TAG", LVL))
import numpy as np
import cephalo4 as C
import monitor4; monitor4.install()
if not os.environ.get("NOXL"): import xlpod4; xlpod4.install()
import sim4 as S4
import body4h as B4
if os.environ.get("Q_J1"): pass
M_HAND, K_HAND, BO_H, R_H, HANDS = HP.M_HAND, HP.K_HAND, HP.BO_H, HP.R_H, HP.HANDS
FING = os.environ.get('H3_FINGERS', '0') == '1' and LVL != 'none'
if FING:
    import hand3_mj as HM; HM.install(HANDS)
if LVL != "none":
    for i in HANDS: B4.HAND[i] = dict(T=T_ENV, K=K_HAND); B4.BO[i] = BO_H; B4.QT[i] = HP.QT[LVL]
    C.M_ROBOT += len(HANDS) * (M_HAND - C.M_POD); C.W_ROBOT = C.M_ROBOT * C.G
    _o = S4.arm_xml
    def arm_xml(i):
        x = _o(i)
        if i not in HANDS: return x
        a, b = f"{C.M_POD * 0.8:.4f}", f"{C.M_POD * 0.2:.4f}"
        x = x.replace(f'mass="{a}"', f'mass="{M_HAND * (1 - HP.F_ROTOR):.4f}"').replace(f'mass="{b}"', f'mass="{M_HAND * HP.F_ROTOR:.4f}"')
        old = f'fromto="{C.BLADE_OFF - 0.002:.4f} 0 0 {C.BLADE_OFF + 0.002:.4f} 0 0" size="{C.CLAW_R:.4f}"'
        x = x.replace(old, f'fromto="{BO_H - 0.002:.4f} 0 0 {BO_H + 0.002:.4f} 0 0" size="{R_H:.4f}"')
        if FING:                                   # the four fingers as bodies with contact; their mass comes out of the rotor-plane lump
            x = x.replace(f'mass="{M_HAND * HP.F_ROTOR:.4f}"', f'mass="{M_HAND * HP.F_ROTOR - HM.M_FINGERS:.4f}"')
            tip = f'<site name="tip{i}"'; assert tip in x
            x = x.replace(tip, HM.hand_xml(i, float(os.environ.get('H3_CLOCK', '45'))) + tip)
        return x
    S4.arm_xml = arm_xml
if os.environ.get('H3_SPIN', 'opp') == 'opp': B4.SPIN[:] = [1, -1, -1, 1, -1, 1, 1, -1]
sys.argv = ["quad4h.py", "run"] + sys.argv[2:]
os.environ.setdefault("QUAD_TAG", "h3_" + LVL)
import runpy; runpy.run_path(os.environ.get("QUAD_FILE","quad4h.py"), run_name="__main__")
