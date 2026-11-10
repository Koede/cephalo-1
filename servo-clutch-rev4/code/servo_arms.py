"""Servo-in-joint arm variants for Cephalo-1 (every joint driven by its own servo at the joint; no tendons).

Each variant: joint layout (s along the arm, pitch/yaw, limits), a servo family it may use, a mass model, and meshes.
Sizing: per joint, the smallest servo in the family with  rated >= continuous torque  and  stall/1.25 >= peak torque,
iterated with the resulting masses.  Load cases (same as the base-box study):
  continuous: walk with EDF assist 6.2 N/foot, hover 5.5 N per claw-prop +-10 deg vectoring, arm held out at 1 g
  peak:       walk fans off 12.3 N/foot, full thrust 10.8 N at 20 deg vectoring, arm swung at 2 g
Servo data: STS3032 / STS3215 (12 V) / STS3250 datasheets; STS3250 rated torque = stall/3 (est.); geared and BLDC
entries are estimates.  Meshes are simplified (boxes/cylinders) but sized from the real servo bodies.
"""
import numpy as np
from geom import frustum, sphere, box, tube_path, merge, frame_z, rot
from tentacles import (Variant, J, Tr, Rt, H, X, Y, Z, xform, axis_vec, vertebra, L_ARM,
                       WHITE, DARK, GREY, STEEL, ORANGE, YEL, CARBON, PEACH, BLUE, PA12)
G = 9.81
#            rated  stall  mass   L     W     H   (mm; H = shaft axis)
SV = {"STS3032":     (0.147, 0.441, 0.020, 23.2, 12.1, 28.5),
      "STS3215":     (0.981, 2.94, 0.055, 45.2, 24.7, 35.0),
      "STS3250":     (1.63, 4.90, 0.0745, 45.2, 24.7, 35.0),
      "STS3032+4:1": (0.47, 1.41, 0.036, 23.2, 26.0, 44.0),      # + printed planetary 4:1, eff 0.8 (est.)
      "STS3215+4:1": (3.14, 9.4, 0.090, 45.2, 36.0, 55.0),
      "BLDC-S":      (0.60, 1.80, 0.060, 30.0, 52.0, 52.0),       # ~2804 gimbal + 6:1 printed cycloid (est.)
      "BLDC-L":      (1.60, 4.80, 0.130, 36.0, 64.0, 64.0)}       # ~4108 gimbal + 6:1 printed cycloid (est.)
SCOL = {"STS3032": ORANGE, "STS3215": BLUE, "STS3250": [0.25, 0.26, 0.29], "STS3032+4:1": ORANGE,
        "STS3215+4:1": BLUE, "BLDC-S": GREY, "BLDC-L": [0.35, 0.36, 0.40]}
M_POD = 0.100
BASE = H(p=[0.104, 0, 0.039])


def cross_r(name, pad=2.5):
    L, W, Hh = SV[name][3:]
    return 1e-3 * (np.hypot(W, Hh) / 2 + pad)


class ServoVariant(Variant):
    FAMILY = ["STS3032", "STS3215", "STS3250"]
    FIXED = None             # list of servo names (no sizing)
    HOUSING = 0.010          # kg per joint (brackets, horn, bearing)
    LAM = 0.08               # kg/m link structure

    def __init__(self, sf):
        self.servo = list(self.FIXED) if self.FIXED else None
        n0 = len(self.make_joints())
        if self.servo is None:
            self.servo = [self.FAMILY[0]] * n0
        super().__init__(sf)
        self.sf = sf
        for _ in range(4):
            self.size()
        self.links = [self.make_link(k, self.lens[k], self.edges[k]) for k in range(self.n + 1)]
        self.col_r0 = max(cross_r(self.servo[0]), 0.012); self.col_r1 = max(cross_r(self.servo[-1]) * 0.8, 0.010)

    # ---- mass + torque model
    def masses(self, frames, tip):
        pts, m, s = [], [], []
        for k, j in enumerate(self.joints):
            M = frames[k + 1]
            pts.append(M[:3, 3]); m.append(SV[self.servo[k]][2] + self.HOUSING); s.append(j["s"] - 1e-4)
        for k in range(self.n + 1):
            M = frames[k]; lk = self.lens[k]
            pts.append(M[:3, 3] + M[:3, 0] * lk / 2); m.append(self.LAM * lk); s.append(self.edges[k] + lk / 2)
        pts.append(tip[:3, 3] + tip[:3, 0] * 0.03); m.append(M_POD); s.append(9.0)
        return np.array(pts), np.array(m), np.array(s)

    def torques(self, q, F_tip, gm=1.0, foot=0.0):
        frames, tip = self.fk(q, BASE)
        P, m, s = self.masses(frames, tip)
        F = np.zeros_like(P); F[:, 2] = -G * m * gm
        P = np.vstack([P, tip[:3, 3] + tip[:3, 0] * foot]); F = np.vstack([F, F_tip(tip)]); s = np.append(s, 9.0)
        tau = np.zeros(self.n)
        for k, j in enumerate(self.joints):
            M = frames[k + 1]; c = M[:3, 3]; a = M[:3, 1] if j["typ"] == "p" else M[:3, 2]
            sel = s > j["s"]
            tau[k] = a @ np.cross(P[sel] - c, F[sel]).sum(0)
        return tau

    def cases(self):
        L = L_ARM; z = np.zeros_like(self.sf)
        stand = self.map(z + (np.pi / 2) / L, z)[0]
        tulip = self.map(z - (np.pi / 2) / L, z)[0]
        flat = np.zeros(self.n)
        vec = lambda T, a: (lambda t: (np.cos(np.radians(a)) * t[:3, 0] + np.sin(np.radians(a)) * t[:3, 1]) * T)
        cont = [(stand, lambda t: np.array([0, 0, 6.2]), 1, 0.07), (tulip, vec(5.5, 10), 1, 0.034),
                (tulip, vec(5.5, -10), 1, 0.034), (flat, lambda t: np.zeros(3), 1, 0)]
        peak = [(stand, lambda t: np.array([0, 0, 12.3]), 1, 0.07), (tulip, vec(10.8, 20), 1, 0.034),
                (tulip, vec(10.8, -20), 1, 0.034), (flat, lambda t: np.zeros(3), 2, 0)]
        return cont, peak

    def size(self):
        cont, peak = self.cases()
        self.tc = np.max([np.abs(self.torques(*c)) for c in cont], 0)
        self.tp = np.max([np.abs(self.torques(*c)) for c in peak], 0)
        if not self.FIXED:
            new = []
            for k in range(self.n):
                pick = self.FAMILY[-1]
                for nm in self.FAMILY:
                    r, st = SV[nm][:2]
                    if r >= self.tc[k] and st / 1.25 >= self.tp[k]:
                        pick = nm; break
                new.append(pick)
            self.servo = new
        self.util = np.array([max(self.tc[k] / SV[nm][0], 1.25 * self.tp[k] / SV[nm][1]) for k, nm in enumerate(self.servo)])

    def arm_mass(self):
        return sum(SV[nm][2] + self.HOUSING for nm in self.servo) + self.LAM * self.L + M_POD

    def dia(self):
        return 2 * cross_r(self.servo[0])

    def summary(self):
        from collections import Counter
        c = Counter(self.servo)
        return (f"{self.n} DOF | " + " ".join(f"{v}x{k}" for k, v in c.items()) +
                f" | {self.arm_mass()*1000:.0f} g | root Ø{self.dia()*1000:.0f} | load {self.util.max():.2f}x")


def servo_box(c, Rl, name, ax_shaft):
    """Servo body centred at c (local), long side along local x, shaft axis along ax_shaft (local unit vector)."""
    L, W, Hh = np.array(SV[name][3:]) * 1e-3
    o = np.cross(X, ax_shaft)
    return box(c, np.stack([X, o, ax_shaft], 1), [L, W, Hh], SCOL[name])


# ------------------------------------------------------------------ 1/2 rev 3 baseline (fixed servos)
from tentacles import ServoChain as _SC, ServoShell as _SS


class Rev3Chain(ServoVariant, _SC):
    name = "1  REV 3 U-BRACKET CHAIN (current)"

    def dia(self):
        return 0.065
    FIXED = ["STS3250"] * 3 + ["STS3215"] * 2 + ["STS3032"] * 4
    HOUSING = 0.018; LAM = 0.06

    def make_link(self, k, lk, s0):
        return _SC.make_link(self, k, lk, s0)


class Rev3Shell(ServoVariant, _SS):
    name = "2  SERVO SHELLS (same servos, smooth)"

    def dia(self):
        return 0.048
    FIXED = ["STS3250"] * 3 + ["STS3215"] * 2 + ["STS3032"] * 4
    HOUSING = 0.014; LAM = 0.09

    def make_link(self, k, lk, s0):
        return _SS.make_link(self, k, lk, s0)


# ------------------------------------------------------------------ 3 inline servo + bevel knuckle
class InlineBevel(ServoVariant):
    name = "3  INLINE SERVO + BEVEL KNUCKLE"
    POS = [55, 121, 187, 249, 305, 357, 405]
    LIM = 100; HOUSING = 0.014; LAM = 0.07

    def make_joints(self):
        return [J(s * 1e-3, "py"[k % 2], self.LIM) for k, s in enumerate(self.POS)]

    def rt(self, k):
        return cross_r(self.servo[min(k, self.n - 1)]) if k < self.n else 0.012

    def dia(self):
        return 2 * max(self.rt(k) for k in range(self.n))

    def make_link(self, k, lk, s0):
        parts = []; r = self.rt(k)
        x0 = 0.010 if k > 0 else 0.0
        if k < self.n:
            nm = self.servo[k]; Ls = SV[nm][3] * 1e-3
            a = axis_vec(self.joints[k]["typ"])
            parts.append(servo_box(X * (lk - 0.016 - Ls / 2), np.eye(3), nm, np.cross(a, X)))
            for xr in (x0 + 0.002, lk - 0.012):                      # open frame: two rings + 4 rods
                parts.append(frustum(X * xr, X * (xr + 0.003), r, r, WHITE, n=20))
            for m_ in range(4):
                e = np.cos(m_ * np.pi / 2 + np.pi / 4) * Y + np.sin(m_ * np.pi / 2 + np.pi / 4) * Z
                parts.append(frustum(X * x0 + e * r * 0.97, X * (lk - 0.010) + e * r * 0.97, 0.0018, 0.0018, WHITE, n=6))
            parts.append(frustum(X * (lk - 0.016), X * (lk - 0.006), r * 0.45, r * 0.2, ORANGE, n=14))   # bevel pinion
            parts.append(frustum(X * lk - a * r * 0.75, X * lk + a * r * 0.75, r * 0.42, r * 0.42, DARK, n=16))  # knuckle hub
        else:
            parts.append(frustum(X * x0, X * lk, 0.012, 0.010, WHITE, n=16))
        if k > 0:                                                     # clevis fork driven by joint k-1
            a = axis_vec(self.joints[k - 1]["typ"]); o = np.cross(X, a); rp = self.rt(k - 1)
            for sg in (-1, 1):
                parts.append(box(a * sg * rp * 0.85 + X * 0.004, np.stack([X, a, o], 1), [0.016, 0.003, rp * 0.9], WHITE))
        return merge(parts)


# ------------------------------------------------------------------ 4 cardan nodes (crossed servo pair in a sphere)
class CardanNodes(ServoVariant):
    name = "4  CARDAN NODES (crossed servo pair)"
    NODES = [60, 140, 215, 285, 350, 410]
    HOUSING = 0.016; LAM = 0.05

    def make_joints(self):
        js = []
        for s in self.NODES:
            js += [J(s * 1e-3, "p", 90), J(s * 1e-3 + 0.014, "y", 70)]
        return js

    def dia(self):
        return 2 * self.rn(0)

    def rn(self, k):
        big = SV[self.servo[k]][3] > 30
        return 0.034 if big else 0.022

    def make_link(self, k, lk, s0):
        parts = []
        if k % 2 == 1:      # spider between p and y joint: sphere housing both servos
            r = self.rn(k - 1)
            parts.append(sphere(X * lk / 2, r, WHITE, n=22, m=12))
            parts.append(frustum(X * lk / 2 - Y * (r + 0.002), X * lk / 2 + Y * (r + 0.002), r * 0.38, r * 0.38, SCOL[self.servo[k - 1]], n=16))
            parts.append(frustum(X * lk / 2 - Z * (r + 0.002), X * lk / 2 + Z * (r + 0.002), r * 0.38, r * 0.38, SCOL[self.servo[k]], n=16))
        else:               # carbon tube between nodes with cone fairings
            r0 = self.rn(k - 1) if k > 0 else 0.03; r1 = self.rn(k) if k < self.n else 0.012
            parts.append(frustum([0, 0, 0], X * lk, 0.007, 0.007, CARBON, n=12))
            if k > 0: parts.append(frustum(X * 0.0, X * min(0.03, lk / 2), r0 * 0.7, 0.008, WHITE, n=16))
            if k < self.n: parts.append(frustum(X * max(lk - 0.03, lk / 2), X * lk, 0.008, r1 * 0.7, WHITE, n=16))
        return merge(parts)


# ------------------------------------------------------------------ 5 servo-driven geared double hinge
class ServoDoubleHinge(ServoVariant):
    name = "5  SERVO GEARED DOUBLE HINGES"
    POS = [70, 137, 204, 266, 322, 372, 418, 460]
    RG = 0.011; HOUSING = 0.012; LAM = 0.07

    def make_joints(self):
        return [J(s * 1e-3, "py"[k % 2], 120, R=self.RG) for k, s in enumerate(self.POS)]

    def dia(self):
        L, W, Hh = SV[self.servo[0]][3:]
        return 1e-3 * np.hypot(Hh + 8, W + 8)

    def make_link(self, k, lk, s0):
        parts = []
        x0 = self.RG if k > 0 else 0.0; x1 = lk - self.RG if k < self.n else lk
        nm = self.servo[min(k, self.n - 1)]
        w = SV[nm][4] * 1e-3 + 0.008
        if k < self.n:
            a = axis_vec(self.joints[k]["typ"]); o = np.cross(X, a)
            Ls = SV[nm][3] * 1e-3
            parts.append(servo_box(X * (x1 - Ls / 2 - 0.004), np.eye(3), nm, a))
            for sg in (-1, 1):
                parts.append(box(X * (x0 + x1) / 2 + a * sg * (SV[nm][5] * 1e-3 / 2 + 0.004), np.stack([X, a, o], 1),
                                 [x1 - x0, 0.003, w], WHITE))
        else:
            parts.append(frustum(X * x0, X * x1, 0.012, 0.010, WHITE, n=16))
        for kk, x in ((k - 1, x0), (k, x1)):
            if 0 <= kk < self.n:
                a = axis_vec(self.joints[kk]["typ"]); hw = SV[self.servo[kk]][5] * 1e-3 / 2 + 0.004
                parts.append(frustum(X * x - a * hw, X * x + a * hw, self.RG, self.RG, GREY, n=20, c2=DARK, stripes=10))
        return merge(parts)


# ------------------------------------------------------------------ 6 servo inside each rolling vertebra (geared cams)
class ServoRolling(ServoVariant):
    name = "6  SERVO-IN-VERTEBRA ROLLING JOINTS"
    LENS = np.array([50, 46, 44, 42, 40, 38, 36, 34, 32, 30, 30]) * 1e-3
    HOUSING = 0.010; LAM = 0.10

    def __init__(self, sf):
        self.rv = np.linspace(0.026, 0.015, len(self.LENS))
        self.cum = np.concatenate([[0], np.cumsum(self.LENS)])
        self.L_tip = 0.045 + self.cum[-1]
        super().__init__(sf)

    def dia(self):
        return 2 * self.rv[0]

    def make_joints(self):
        js = [J(0.025, "p", 60)]
        for v in range(len(self.LENS) - 1):
            Rj = 0.9 * 0.5 * (self.rv[v] + self.rv[v + 1])
            js.append(J(0.045 + self.cum[v + 1], "yp"[v % 2], 80, R=Rj))
        return js

    def make_link(self, k, lk, s0):
        if k == 0:
            return merge([frustum([0, 0, 0], X * lk, 0.030, 0.026, GREY, n=24),
                          servo_box(X * lk * 0.4, np.eye(3), self.servo[0], Z)])
        v = k - 1
        nv = len(self.LENS)
        ap = None if v == 0 else "yp"[(v - 1) % 2]; ad = None if v == nv - 1 else "yp"[v % 2]
        Rp = self.joints[k - 1]["R"] if v > 0 else 1; Rd = self.joints[k]["R"] if v < nv - 1 else 1
        off = lk - self.LENS[v]
        parts = []
        if off > 1e-4:
            parts.append(frustum([0, 0, 0], X * (off + 0.002), 0.016, 0.016, DARK, n=16))
        nm = self.servo[k] if k < self.n else None
        col = [0.93, 0.93, 0.90] if v % 2 else PEACH
        Vv, Fv, Cv = vertebra(self.rv[v], self.LENS[v], Rp, ap, Rd, ad, col, n=26)
        parts.append((Vv + X * off, Fv, Cv))
        if nm:
            a = axis_vec(self.joints[k]["typ"])
            parts.append(frustum(X * (off + self.LENS[v] * 0.55) - a * (self.rv[v] + 0.002), X * (off + self.LENS[v] * 0.55) + a * (self.rv[v] + 0.002),
                                 self.rv[v] * 0.35, self.rv[v] * 0.35, SCOL[nm], n=14))
        return merge(parts)


# ------------------------------------------------------------------ 7 micro servo + printed 4:1 planetary, inline
class GearedMicro(InlineBevel):
    name = "7  MICRO SERVO + 4:1 PLANETARY (inline)"
    FAMILY = ["STS3032+4:1", "STS3215+4:1"]
    POS = [75, 150, 215, 272, 325, 375, 420, 460]
    HOUSING = 0.012; LAM = 0.06

    def rt(self, k):
        if k >= self.n: return 0.012
        L, W, Hh = SV[self.servo[k]][3:]
        return 1e-3 * (np.hypot(L, W) / 2 + 2.5)

    def make_link(self, k, lk, s0):
        parts = []; r = self.rt(k); x0 = 0.010 if k > 0 else 0.0
        if k < self.n:
            nm = self.servo[k]; L, W, Hh = np.array(SV[nm][3:]) * 1e-3
            a = axis_vec(self.joints[k]["typ"])
            parts.append(box(X * (lk - 0.016 - Hh / 2), np.stack([Y, Z, X], 1), [L, W, Hh * 0.6], SCOL[nm]))   # shaft along arm
            parts.append(frustum(X * (lk - 0.016 - Hh * 0.4), X * (lk - 0.016), r * 0.6, r * 0.6, STEEL, n=18, c2=DARK, stripes=12))
            for xr in (x0 + 0.002, lk - 0.012):
                parts.append(frustum(X * xr, X * (xr + 0.003), r, r, WHITE, n=20))
            for m_ in range(4):
                e = np.cos(m_ * np.pi / 2 + np.pi / 4) * Y + np.sin(m_ * np.pi / 2 + np.pi / 4) * Z
                parts.append(frustum(X * x0 + e * r * 0.97, X * (lk - 0.010) + e * r * 0.97, 0.0018, 0.0018, WHITE, n=6))
            parts.append(frustum(X * (lk - 0.016), X * (lk - 0.006), r * 0.45, r * 0.2, ORANGE, n=14))
            parts.append(frustum(X * lk - a * r * 0.75, X * lk + a * r * 0.75, r * 0.42, r * 0.42, DARK, n=16))
        else:
            parts.append(frustum(X * x0, X * lk, 0.012, 0.010, WHITE, n=16))
        if k > 0:
            a = axis_vec(self.joints[k - 1]["typ"]); o = np.cross(X, a); rp = self.rt(k - 1)
            for sg in (-1, 1):
                parts.append(box(a * sg * rp * 0.85 + X * 0.004, np.stack([X, a, o], 1), [0.016, 0.003, rp * 0.9], WHITE))
        return merge(parts)

    def dia(self):
        return 2 * max(self.rt(k) for k in range(self.n))


# ------------------------------------------------------------------ 8 snake modules (servo body = link)
class SnakeModules(ServoVariant):
    name = "8  SNAKE MODULES (servo body is the link)"
    POS = [45, 98, 151, 204, 252, 296, 336, 372, 406, 440]
    HOUSING = 0.010; LAM = 0.03

    def make_joints(self):
        return [J(s * 1e-3, "py"[k % 2], 90) for k, s in enumerate(self.POS)]

    def dia(self):
        L, W, Hh = SV[self.servo[0]][3:]
        return 1e-3 * np.hypot(Hh * 0.8 + 15, W * 1.05)

    def make_link(self, k, lk, s0):
        parts = []
        if k < self.n:
            nm = self.servo[k]; a = axis_vec(self.joints[k]["typ"]); o = np.cross(X, a)
            L, W, Hh = np.array(SV[nm][3:]) * 1e-3
            parts.append(box(X * (lk - L / 2 + 0.006), np.stack([X, o, a], 1), [L, W, Hh * 0.8], SCOL[nm]))
            parts.append(frustum(X * lk - a * (Hh * 0.4 + 0.004), X * lk + a * (Hh * 0.4 + 0.004), 0.008, 0.008, DARK, n=12))
        else:
            parts.append(frustum([0, 0, 0], X * lk, 0.011, 0.009, WHITE, n=14))
        if k > 0:
            nm = self.servo[k - 1]; a = axis_vec(self.joints[k - 1]["typ"]); o = np.cross(X, a)
            L, W, Hh = np.array(SV[nm][3:]) * 1e-3
            for sg in (-1, 1):
                parts.append(box(a * sg * (Hh * 0.4 + 0.006) + X * 0.010, np.stack([X, a, o], 1), [0.026, 0.0025, W * 1.05], WHITE))
            parts.append(box(X * 0.023, np.stack([X, a, o], 1), [0.003, Hh * 0.8 + 0.015, W * 1.05], WHITE))
            if k < self.n:
                parts.append(frustum(X * 0.023, X * max(lk - SV[self.servo[k]][3] * 1e-3 + 0.006, 0.024), 0.007, 0.007, WHITE, n=10))
        return merge(parts)


# ------------------------------------------------------------------ 9 cycloidal BLDC modules (FOC servo)
class BLDCCycloid(ServoVariant):
    name = "9  BLDC + PRINTED CYCLOID (FOC servo)"
    FAMILY = ["BLDC-S", "BLDC-L"]
    POS = [45, 85, 205, 245, 345, 385]
    HOUSING = 0.020; LAM = 0.05

    def make_joints(self):
        return [J(s * 1e-3, "pypypy"[k], 120) for k, s in enumerate(self.POS)]

    def dia(self):
        return SV[self.servo[0]][4] * 1e-3

    def make_link(self, k, lk, s0):
        parts = [frustum([0, 0, 0], [lk, 0, 0], 0.0075, 0.0075, CARBON, n=12)]
        if k > 0:
            a = axis_vec(self.joints[k - 1]["typ"]); o = np.cross(X, a)
            parts.append(box(X * 0.014, np.stack([X, a, o], 1), [0.026, 0.030, 0.018], DARK))
        if k < self.n:
            nm = self.servo[k]; a = axis_vec(self.joints[k]["typ"])
            r = SV[nm][4] * 1e-3 / 2; t = SV[nm][3] * 1e-3
            c = X * lk
            parts.append(frustum(c - a * t / 2, c + a * t / 2, r, r, SCOL[nm], n=26))
            parts.append(frustum(c - a * 0.003, c + a * 0.003, r + 0.0015, r + 0.0015, ORANGE, n=26))
        else:
            parts.append(sphere(X * lk, 0.010, DARK, n=12, m=8))
        return merge(parts)


VARIANTS = [Rev3Chain, Rev3Shell, InlineBevel, CardanNodes, ServoDoubleHinge, ServoRolling, GearedMicro, SnakeModules, BLDCCycloid]

if __name__ == "__main__":
    from writhe_grid import SF
    for V in VARIANTS:
        v = V(SF)
        print(f"{v.name:42s} {v.summary()}")
        print("      servos:", v.servo)
        print("      util:  ", np.round(v.util, 2))
