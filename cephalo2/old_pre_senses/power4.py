"""Cephalo-1 rev 4 power system model: 6S LiPo pack(s) -> EDF + 8 claw-prop ESCs + 12 V servo bus.
Used by body4.py (CEPH_PACKS, CEPH_SOC) so that thrust ceilings follow the loaded battery voltage.

Data and estimates:
  * EDF JP Hobby 90 mm 12-blade 1750 Kv (datasheet): 22.2 V 103 A 3.57 kg / 2286 W; 24.0 V 117 A 4.01 kg / 2808 W;
    25.2 V 126 A 4.32 kg / 3175 W.  Fit: T_max = 42.4 N x (V / 25.2)^1.51; electrical power P = 11.2 W x T^1.5 (T in N)
    (momentum theory shape, matches the three points within 4 %).
  * Claw-prop (6 in, 2207 6S): ceilings 10.1 / 12.6 / 15.1 N (pess/mid/opt, cephalo4, unmeasured) are taken to hold at
    25.2 V, like hobby thrust tables, and scale with (V / 25.2)^1.5 like the EDF.  Power by momentum theory:
    P = T^1.5 / (sqrt(2 rho A) FM eta), A = 0.0182 m^2 (6 in disc), figure of merit 0.55 (est.; folding finger blades),
    motor + ESC efficiency 0.80 (est.) -> P = 10.75 W x T^1.5.
  * Servo bus: 64 Feetech STS on 12 V from a buck (90 %): 0.05 A idle + 2.7 A x load fraction each (est. from the
    STS3215 12 V stall current); avionics 10 W.
  * Pack: 6S 5000 mAh 100C class (e.g. CNHL G+Plus, 811 g, Turnigy Rapid 713 g).  DC resistance 2.5 mOhm per cell
    (est.) + 4 mOhm connector/harness -> 19 mOhm per pack; packs in parallel divide it.  One pack is already inside the
    2.62 kg body mass; each extra pack adds 0.81 kg.
  * Open-circuit voltage per cell vs state of charge: typical LiPo rest curve (est.).
"""
import numpy as np

SOC_PTS = np.array([0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0])
OCV_PTS = np.array([3.27, 3.65, 3.73, 3.77, 3.80, 3.84, 3.88, 3.95, 4.02, 4.10, 4.20])
V_REF = 25.2
EDF_T_REF, EDF_EXP, EDF_K = 42.4, 1.51, 11.2
CLAW_EXP, CLAW_K = 1.5, 10.75
R_PACK, CAP_AH, M_PACK = 0.019, 5.0, 0.81
I_SERVO_IDLE, I_SERVO_STALL, BUS_EFF, P_AVIONICS = 0.05, 2.7, 0.90, 10.0


class Battery:
    def __init__(self, packs=1, soc=1.0):
        self.n = packs; self.soc = soc
        self.R = R_PACK / packs; self.cap = CAP_AH * packs
        self.V = self.ocv(); self.I = 0.0; self.P = 0.0; self.brownout = False; self.ah = 0.0

    def ocv(self):
        return 6 * float(np.interp(self.soc, SOC_PTS, OCV_PTS))

    def step(self, edf_thrust, fan_thrusts, servo_loads, dt, k_claw=CLAW_K):
        P = EDF_K * max(edf_thrust, 0) ** 1.5 + k_claw * np.sum(np.clip(fan_thrusts, 0, None) ** 1.5)
        P += 12.0 * np.sum(I_SERVO_IDLE + I_SERVO_STALL * np.clip(servo_loads, 0, 1)) / BUS_EFF + P_AVIONICS
        E = self.ocv()
        disc = E * E - 4 * P * self.R
        self.brownout = disc < 0
        V = (E + np.sqrt(max(disc, 0))) / 2
        self.V, self.P, self.I = V, P, P / V
        self.ah += self.I * dt / 3600
        self.soc = max(self.soc - self.I * dt / 3600 / self.cap, 0.0)
        return V

    def edf_max(self):
        return EDF_T_REF * (self.V / V_REF) ** EDF_EXP

    def claw_scale(self):
        return (self.V / V_REF) ** CLAW_EXP


# ---------------------------------------------------------------- claw-prop variants
# "std": the current 6 in Snap-Digit on a 2207 (ceilings 10.1 / 12.6 / 15.1 N at 25.2 V, P = 10.75 W x T^1.5).
# "xl":  the overhaul, a 7.5 in Snap-Digit on a 28xx-class motor.  T-Motor's 7.5-inch-class test table at 6S
#        (21.5-21.7 V) gives 2347 g / 481 W at 80 % and 2994 g / 692 W at 100 % throttle (both P = 4.35 W x T^1.5) and
#        0.51 N m of torque at full thrust (torque/thrust 0.0173 m).  The folding finger blades are taken to lose 10 %
#        thrust against a solid prop (est.; the 6 in Snap-Digit was estimated at 3-4 %): T_max = 26.5 N at 21.3 V,
#        scaled with (V / 21.3)^1.5 and capped at 28 N (motor ~900 W);  P = 5.1 W x T^1.5.
#        Pod mass 115 g -> 156 g (2026-11-13: itemised in xlclaw.py on the T-Motor F90 2806.5, 46.6 g; before that a
#        190 g budget with an ~84 g 28xx motor).
PROPS = dict(std=dict(K=CLAW_K, T_ref=None, V_ref=V_REF, cap=99.0, dM_pod=0.0, R=0.0762, q_per_T=0.0150),
             xl=dict(K=5.1, T_ref=26.5, V_ref=21.3, cap=28.0, dM_pod=0.041, R=0.0953, q_per_T=0.0173))


def claw_max(prop, V, std_ref=12.63):
    p = PROPS[prop]
    if p["T_ref"] is None:
        return std_ref * (V / V_REF) ** CLAW_EXP
    return min(p["cap"], p["T_ref"] * (V / p["V_ref"]) ** 1.5)
