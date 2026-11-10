"""Nine single-tentacle joint variants for the 3x3 writhing comparison (Cephalo-1 arm, 475 mm from the wall face).

Every variant receives the SAME commanded backbone curvature field kappa(s, t).  Each maps it onto its own joints
(each joint takes the integral of the curvature over its share of the arm), couples joints that share tendons
(a tendon section bends all its vertebrae equally), then clamps to its own limits.  Limits are estimates.
Frames: arm local x along the arm, z = arm 'up'.  Pitch +ve = tip down (rotation about local y).
"""
import numpy as np
from geom import frustum, sphere, box, tube_path, merge, frame_z, rot

L_ARM = 0.475
WHITE = [0.92, 0.92, 0.89]; DARK = [0.20, 0.21, 0.23]; GREY = [0.55, 0.57, 0.60]; STEEL = [0.68, 0.70, 0.73]
ORANGE = [0.95, 0.55, 0.20]; TEAL = [0.20, 0.66, 0.60]; YEL = [0.98, 0.80, 0.12]; CARBON = [0.16, 0.16, 0.18]
PEACH = [0.98, 0.78, 0.62]; PINK = [0.93, 0.58, 0.52]; BLUE = [0.30, 0.55, 0.85]; PA12 = [0.66, 0.68, 0.72]
X = np.array([1.0, 0, 0]); Y = np.array([0, 1.0, 0]); Z = np.array([0, 0, 1.0])


def H(R=None, p=None):
    M = np.eye(4)
    if R is not None:
        M[:3, :3] = R
    if p is not None:
        M[:3, 3] = p
    return M


def Tr(d):
    return H(p=[d, 0, 0])


def Rt(ax, a):
    return H(R=rot(ax, a))


def xform(mesh, M):
    V, F, C = mesh
    return V @ M[:3, :3].T + M[:3, 3], F, C


def J(s, typ, lim, R=0.0, grp=None):
    lo, hi = (lim if isinstance(lim, tuple) else (-lim, lim))
    return dict(s=s, typ=typ, lo=np.radians(lo), hi=np.radians(hi), R=R, grp=grp)


class Variant:
    name = ""; sub = ""
    col_r0, col_r1 = 0.016, 0.009          # collision radius root -> tip

    def __init__(self, sf):
        self.joints = self.make_joints()
        n = len(self.joints); self.n = n
        self.s = np.array([j["s"] for j in self.joints])
        self.lo = np.array([j["lo"] for j in self.joints]); self.hi = np.array([j["hi"] for j in self.joints])
        self.typ = np.array([j["typ"] for j in self.joints])
        self.L = getattr(self, "L_tip", L_ARM)
        self.edges = np.concatenate([[0.0], self.s, [self.L]])
        self.lens = np.diff(self.edges)
        # curvature -> joint assignment
        ds = sf[1] - sf[0]
        self.A = np.zeros((n, len(sf)))
        for t in "py":
            idx = np.where(self.typ == t)[0]
            ss = self.s[idx]
            b = np.concatenate([[0.0], 0.5 * (ss[1:] + ss[:-1]), [L_ARM + 1]])
            for m, k in enumerate(idx):
                self.A[k] = ((sf >= b[m]) & (sf < b[m + 1])) * ds
        # tendon/coupling groups
        self.groups = {}
        for k, j in enumerate(self.joints):
            g = j["grp"]
            if g is not None:
                self.groups.setdefault((g, j["typ"]), []).append(k)
        self.links = [self.make_link(k, self.lens[k], self.edges[k]) for k in range(n + 1)]
        self.coll = []
        for k in range(n + 1):
            xs = np.arange(0, self.lens[k] + 1e-9, 0.008)
            for x in xs:
                self.coll.append((k, x, self.edges[k] + x))
        self.coll = np.array(self.coll)

    def radius(self, s):
        return self.col_r0 + (self.col_r1 - self.col_r0) * s / L_ARM

    def map(self, kp, ky):
        q = np.where(self.typ == "p", self.A @ kp, self.A @ ky)
        for idx in self.groups.values():
            q[idx] = q[idx].mean()
        qc = np.clip(q, self.lo, self.hi)
        at = (np.abs(qc - q) > 1e-6) | (np.abs(qc - self.lo) < 1e-4) | (np.abs(qc - self.hi) < 1e-4)
        return qc, at

    def fk(self, q, base):
        M = base.copy(); frames = [M.copy()]; sp = 0.0
        for k, j in enumerate(self.joints):
            ax = Y if j["typ"] == "p" else Z
            R = j["R"]
            M = M @ Tr(j["s"] - sp - R) @ Rt(ax, q[k] / 2) @ Tr(2 * R) @ Rt(ax, q[k] / 2) @ Tr(-R)
            frames.append(M.copy()); sp = j["s"]
        tip = M @ Tr(self.L - sp)
        return frames, tip

    def self_hit(self, frames):
        k = self.coll[:, 0].astype(int); x = self.coll[:, 1]; s = self.coll[:, 2]
        Ms = np.array(frames)[k]
        P = Ms[:, :3, 3] + Ms[:, :3, 0] * x[:, None]
        r = self.radius(s)
        D = np.linalg.norm(P[:, None] - P[None], axis=2)
        far = np.abs(s[:, None] - s[None]) > np.maximum(0.06, 3 * (r[:, None] + r[None]))
        clear = np.where(far, D - r[:, None] - r[None], 1.0)
        return clear.min()

    def extra(self, frames):
        return []

    def meshes(self, frames):
        out = []
        for k, m in enumerate(self.links):
            if m is not None:
                out.append(xform(m, frames[k]))
        return out + self.extra(frames)


def axis_vec(typ):
    return Y if typ == "p" else Z


# ---------------------------------------------------------------- 1. current rev 3 servo chain
SERVO = [(0.046, 0.030, 0.034)] * 3 + [(0.040, 0.027, 0.030)] * 2 + [(0.023, 0.023, 0.013)] * 4   # STS3250/3215/3032


class ServoChain(Variant):
    name = "1  SERVO CHAIN (rev 3, current)"
    sub = "9 Feetech servos in U-brackets | 9 DOF | j0 ±41°, j1 ±19°, rest ±90°"
    col_r0, col_r1 = 0.022, 0.013
    POS = np.array([43.5, 78, 161, 196, 279, 307, 356, 378, 426]) * 1e-3
    LIM = [41, 19, 90, 90, 90, (-80, 90), 90, 90, 90]

    def make_joints(self):
        return [J(s, "pypypypyp"[k], self.LIM[k]) for k, s in enumerate(self.POS)]

    def make_link(self, k, lk, s0):
        r = 0.012 - 0.005 * s0 / L_ARM
        parts = [frustum([0, 0, 0], [lk, 0, 0], r, r * 0.95, WHITE, n=12)]
        if k > 0:      # U-bracket driven by joint k-1 (two side plates)
            t = self.joints[k - 1]["typ"]; sx, sy, sz = SERVO[k - 1]
            a = axis_vec(t); o = np.cross(X, a)
            for sg in (-1, 1):
                parts.append(box(a * sg * (sy / 2 + 0.004) + X * 0.012, np.stack([X, a, o], 1), [0.036, 0.003, sz * 0.9], WHITE))
            parts.append(box(X * 0.031, np.stack([X, a, o], 1), [0.004, sy + 0.011, sz * 0.9], WHITE))
        if k < self.n:  # servo body of joint k
            t = self.joints[k]["typ"]; sx, sy, sz = SERVO[k]
            a = axis_vec(t); o = np.cross(X, a)
            c = X * (lk - sx / 2 + 0.010)
            parts.append(box(c, np.stack([X, a, o], 1), [sx, sy, sz], DARK if t == "p" else GREY))
            parts.append(frustum(X * lk - a * (sy / 2 + 0.003), X * lk + a * (sy / 2 + 0.003), 0.009, 0.009, ORANGE, n=12))
        return merge(parts)


# ---------------------------------------------------------------- 2. servo-as-structure shells
class ServoShell(ServoChain):
    name = "2  SERVO SHELLS"
    sub = "same 9 servos inside smooth printed shells + idler bearing | ±41/±30/±70° (est.)"
    col_r0, col_r1 = 0.024, 0.014
    LIM = [41, 30, 70, 70, 70, 70, 70, 70, 70]

    def make_link(self, k, lk, s0):
        r0 = 0.024 * (1 - 0.45 * s0 / L_ARM); r1 = 0.024 * (1 - 0.45 * (s0 + lk) / L_ARM)
        parts = [frustum([0, 0, 0], [lk, 0, 0], r0 * 0.92, r1 * 0.92, WHITE, n=20)]
        if k < self.n:
            a = axis_vec(self.joints[k]["typ"])
            parts.append(sphere(X * lk, r1, WHITE, n=18, m=10))
            parts.append(frustum(X * lk - a * (r1 + 0.002), X * lk + a * (r1 + 0.002), 0.010, 0.010, BLUE, n=14))
        return merge(parts)


# ---------------------------------------------------------------- 3. printed cycloidal BLDC modules
class Cycloidal(Variant):
    name = "3  CYCLOIDAL BLDC MODULES"
    sub = "6 backdrivable printed-cycloid joints, carbon tubes | 6 DOF | ±120° (est.)"
    col_r0, col_r1 = 0.020, 0.014
    POS = [0.045, 0.085, 0.205, 0.245, 0.345, 0.385]

    def make_joints(self):
        return [J(s, "pypypy"[k], 120) for k, s in enumerate(self.POS)]

    def make_link(self, k, lk, s0):
        parts = [frustum([0, 0, 0], [lk, 0, 0], 0.0075, 0.0075, CARBON, n=12)]
        if k > 0:
            a = axis_vec(self.joints[k - 1]["typ"]); o = np.cross(X, a)
            parts.append(box(X * 0.012, np.stack([X, a, o], 1), [0.024, 0.030, 0.016], DARK))
        if k < self.n:
            a = axis_vec(self.joints[k]["typ"]); r = [0.028, 0.028, 0.024, 0.024, 0.020, 0.020][k]
            c = X * lk
            parts.append(frustum(c - a * 0.013, c + a * 0.013, r, r, GREY, n=24))
            parts.append(frustum(c - a * 0.003, c + a * 0.003, r + 0.0015, r + 0.0015, ORANGE, n=24))
            parts.append(frustum(c + a * 0.013, c + a * 0.016, r * 0.6, r * 0.6, DARK, n=16))
        else:
            parts.append(sphere(X * lk, 0.010, DARK, n=12, m=8))
        return merge(parts)


# ---------------------------------------------------------------- 4. gear-coupled double hinges
class DoubleHinge(Variant):
    name = "4  GEARED DOUBLE HINGES"
    sub = "8 folding-phone knuckles, 2 meshing gears each | 8 DOF | ±90° per knuckle"
    col_r0, col_r1 = 0.018, 0.010
    RG = 0.011

    def make_joints(self):
        return [J(s, "pypypypy"[k], 90, R=self.RG) for k, s in enumerate(np.linspace(0.06, 0.43, 8))]

    def make_link(self, k, lk, s0):
        w = 0.032 - 0.014 * s0 / L_ARM
        x0 = self.RG if k > 0 else 0.0; x1 = lk - self.RG if k < self.n else lk
        parts = [box(X * (x0 + x1) / 2, np.eye(3), [x1 - x0, w * 0.75, w * 0.75], WHITE)]
        for kk, x in ((k - 1, x0), (k, x1)):
            if 0 <= kk < self.n:
                a = axis_vec(self.joints[kk]["typ"])
                parts.append(frustum(X * x - a * w * 0.3, X * x + a * w * 0.3, self.RG, self.RG, GREY, n=20, c2=DARK, stripes=10))
        return merge(parts)


# ---------------------------------------------------------------- 5. cross-axis flexure pivots
class Flexure(Variant):
    name = "5  CROSS-AXIS FLEXURES"
    sub = "17 printed X-leaf pivots, no bearings | 17 DOF | ±25° (fatigue limit, est.)"

    def make_joints(self):
        return [J(s, "py"[k % 2], 25) for k, s in enumerate(np.linspace(0.04, 0.45, 17))]

    def make_link(self, k, lk, s0):
        r0 = 0.017 - 0.009 * s0 / L_ARM; r1 = 0.017 - 0.009 * (s0 + lk) / L_ARM
        g = 0.4 * r1
        parts = [frustum(X * (g if k else 0), X * (lk - (g if k < self.n else 0)), r0, r1, WHITE, n=14)]
        if k < self.n:
            a = axis_vec(self.joints[k]["typ"]); o = np.cross(X, a)
            for sg in (-1, 1):
                Rl = rot(a, sg * np.pi / 4) @ np.stack([X, a, o], 1)
                parts.append(box(X * lk, Rl, [2.2 * g * 1.41, 1.7 * r1, 0.0015], DARK))
        return merge(parts)


# ---------------------------------------------------------------- 6. TPU continuum backbone, 2 tendon sections
class TPUContinuum(Variant):
    name = "6  TPU CONTINUUM, 2 SECTIONS"
    sub = "tapered TPU spine + discs, 2 tendon sections | 4 DOF | 180° per section"
    col_r0, col_r1 = 0.016, 0.009
    NJ = 40

    def make_joints(self):
        return [J(s, "py"[k % 2], 18, grp=0 if k < self.NJ // 2 else 1) for k, s in enumerate(np.linspace(0.012, 0.465, self.NJ))]

    def disc_r(self, s):
        return 0.017 - 0.008 * s / L_ARM

    def make_link(self, k, lk, s0):
        r0 = 0.006 - 0.003 * s0 / L_ARM
        parts = [frustum([0, 0, 0], [lk, 0, 0], r0, r0 * 0.97, TEAL, n=10)]
        if k % 4 == 1:
            rd = self.disc_r(s0)
            parts.append(frustum(X * (lk / 2 - 0.0015), X * (lk / 2 + 0.0015), rd, rd, WHITE, n=20))
        return merge(parts)

    def extra(self, frames):
        tend = [[] for _ in range(3)]
        for k in range(1, self.n + 1, 4):
            M = frames[k]; s0 = self.edges[k]; rd = self.disc_r(s0) - 0.002
            for m in range(3):
                a = np.radians(90 + 120 * m)
                p = np.array([self.lens[k] / 2, rd * np.cos(a), rd * np.sin(a)])
                tend[m].append(M[:3, :3] @ p + M[:3, 3])
        return [tube_path(np.array(t), 0.0009, YEL, n=6) for t in tend]


# ---------------------------------------------------------------- 7-9. rolling-contact tendon vertebrae
def vertebra(r, h, Rp, ap, Rd, ad, col, n=24, rings=4, axial=3):
    """Cylinder r, length h along x; proximal/distal faces are convex rolling cams (cylinder radius Rp/Rd,
    cam axis = local y for 'p', local z for 'y'; None = flat)."""
    def f(ax, R, y, z):
        if ax is None:
            return 0 * y
        c = z if ax == "p" else y
        return R - np.sqrt(np.maximum(R * R - c * c, 0))
    th = np.linspace(0, 2 * np.pi, n, endpoint=False)
    cy, cz = r * np.cos(th), r * np.sin(th)
    fp = f(ap, Rp, cy, cz); fd = f(ad, Rd, cy, cz)
    V = [np.stack([fp + k / axial * (h - fp - fd), cy, cz], 1) for k in range(axial + 1)]
    i = np.arange(n); j = (i + 1) % n
    F = []
    for k in range(axial):
        a = k * n + i; b = k * n + j; c = (k + 1) * n + j; d = (k + 1) * n + i
        F += [np.stack([a, b, c], 1), np.stack([a, c, d], 1)]
    V = [np.concatenate(V)]; off = len(V[0])
    for end in (0, 1):
        ys = np.concatenate([[0.0]] + [q * cy for q in np.linspace(0, 1, rings + 1)[1:]])
        zs = np.concatenate([[0.0]] + [q * cz for q in np.linspace(0, 1, rings + 1)[1:]])
        x = f(ap, Rp, ys, zs) if end == 0 else h - f(ad, Rd, ys, zs)
        Fc = [np.stack([np.zeros(n, int), 1 + i, 1 + j], 1)]
        for m in range(rings - 1):
            a = 1 + m * n + i; b = 1 + m * n + j; c = 1 + (m + 1) * n + j; d = 1 + (m + 1) * n + i
            Fc += [np.stack([a, b, c], 1), np.stack([a, c, d], 1)]
        F.append(np.concatenate(Fc) + off); V.append(np.stack([x, ys, zs], 1)); off += len(ys)
    F = np.concatenate(F)
    return np.concatenate(V), F, np.tile(np.asarray(col, float), (len(F), 1))


class Rolling(Variant):
    NV = 18; SEC = 3; LIM = 45
    COLS = [WHITE, PEACH, PINK]
    name = "7  ROLLING VERTEBRAE, 3 SECTIONS"
    sub = "recommended: root knuckle + 3 tendon sections | 8 DOF | ±45°/vertebra"
    col_r0, col_r1 = 0.020, 0.010

    def __init__(self, sf):
        self.h = np.linspace(0.028, 0.0198, self.NV)
        self.r = np.linspace(0.018, 0.008, self.NV)
        self.cum = np.concatenate([[0], np.cumsum(self.h)])
        self.L_tip = 0.045 + self.cum[-1]
        self.Rj = 1.3 * 0.5 * (self.r[:-1] + self.r[1:])
        super().__init__(sf)

    def make_joints(self):
        js = [J(0.015, "p", 60, grp="rp"), J(0.030, "y", 40, grp="ry")]
        for v in range(self.NV - 1):
            js.append(J(0.045 + self.cum[v + 1], "py"[v % 2], self.LIM, R=self.Rj[v], grp=v * self.SEC // (self.NV - 1)))
        return js

    def sec_col(self, v):
        return self.COLS[min(v * self.SEC // self.NV, self.SEC - 1) % len(self.COLS)]

    def make_link(self, k, lk, s0):
        if k == 0:
            return merge([box(X * -0.062, np.eye(3), [0.058, 0.070, 0.050], [0.18, 0.36, 0.26]),   # base control box
                          box(X * -0.062 + Z * 0.026, np.eye(3), [0.050, 0.060, 0.002], [0.85, 0.70, 0.25]),
                          frustum([-0.004, 0, 0], [0.006, 0, 0], 0.034, 0.034, GREY, n=28),
                          frustum([0.006, 0, 0], [lk, 0, 0], 0.016, 0.014, DARK, n=16)])
        if k == 1:
            return merge([sphere(X * 0.0, 0.019, DARK, n=18, m=10), frustum([0, 0, 0], [lk, 0, 0], 0.014, 0.014, DARK, n=16)])
        v = k - 2
        ap = None if v == 0 else "py"[(v - 1) % 2]
        ad = None if v == self.NV - 1 else "py"[v % 2]
        Rp = self.Rj[v - 1] if v > 0 else 1; Rd = self.Rj[v] if v < self.NV - 1 else 1
        off = 0.015 if v == 0 else 0.0
        parts = []
        if v == 0:
            parts.append(sphere(X * 0.0, 0.019, DARK, n=18, m=10))
            parts.append(frustum([0, 0, 0], [off + 0.002, 0, 0], 0.013, 0.013, DARK, n=16))
        Vv, Fv, Cv = vertebra(self.r[v], self.h[v], Rp, ap, Rd, ad, self.sec_col(v))
        parts.append((Vv + X * off, Fv, Cv))
        for m in range(4):
            a = m * np.pi / 2
            parts.append(sphere(X * (off + self.h[v] / 2) + (self.r[v] + 0.0008) * (np.cos(a) * Y + np.sin(a) * Z),
                                0.0018, DARK, n=8, m=5))
        return merge(parts)

    def extra(self, frames):
        tend = [[] for _ in range(4)]
        for v in range(self.NV):
            M = frames[v + 2]; off = 0.015 if v == 0 else 0.0
            for m in range(4):
                a = m * np.pi / 2
                for fr in (0.2, 0.8):
                    p = X * (off + fr * self.h[v]) + (self.r[v] + 0.0012) * (np.cos(a) * Y + np.sin(a) * Z)
                    tend[m].append(M[:3, :3] @ p + M[:3, 3])
        return [tube_path(np.array(t), 0.0009, YEL, n=6) for t in tend]


class RollingEach(Rolling):
    SEC = 17
    COLS = [WHITE, PEACH]
    name = "8  ROLLING, EVERY VERTEBRA DRIVEN"
    sub = "17 tendon pairs from the base box, one per joint | 19 DOF | ±45°/vertebra"


class CRCJ(Rolling):
    LIM = 30
    COLS = [PA12]
    name = "9  MONOLITHIC CRCJ (printed straps)"
    sub = "rolling cams + flexure straps printed as one part, 3 sections | 8 DOF | ±30° (strap strain)"


VARIANTS = [ServoChain, ServoShell, Cycloidal, DoubleHinge, Flexure, TPUContinuum, Rolling, RollingEach, CRCJ]
