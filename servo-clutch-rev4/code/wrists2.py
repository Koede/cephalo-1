"""Second round of tip-wrist candidates: build-from-hobby-parts versions, compared on range AND on taking a hit.

Common interface (all lengths in metres):
    w.build(p, t, n, u_cmd, dt, defl=None, sep=None, u_force=None) -> (mesh, u, stalk_end, roll_ref, clamped)
      p, t, n : arm tip point, arm direction, arm 'up';   u_cmd : wanted pod (fan) axis
      defl    : rotation vector added after the wrist (impact deflection)
      sep     : translation of the ball + pod off its seat (break-away variants)
      u_force : pod axis forced from outside (pod hanging on its tether)
    w.limit(az) -> max tilt from the arm axis for a fan azimuth az (rad) about the arm
Parts named here are real hobby/hardware parts; every limit and retention force is an estimate from these models,
not a measurement.
"""
import numpy as np
from geom import frustum, sphere, torus, box, tube_path, merge, frame_z, rot

DARK = [0.18, 0.19, 0.21]; STEEL = [0.62, 0.64, 0.67]; RED = [0.90, 0.15, 0.12]; WIRE = [0.85, 0.12, 0.10]
TPU = [0.30, 0.75, 0.55]
DEG = np.pi / 180


def rotvec(v):
    a = np.linalg.norm(v)
    return np.eye(3) if a < 1e-9 else rot(v / a, a)


def sph_dir(Rt, tilt, az):
    return Rt @ np.array([np.sin(tilt) * np.cos(az), np.sin(tilt) * np.sin(az), np.cos(tilt)])


class Wrist:
    accent = [0.5, 0.5, 0.5]
    retention = ""

    def clamp(self, Rt, u_cmd):
        l = Rt.T @ (u_cmd / np.linalg.norm(u_cmd))
        tilt = np.arccos(np.clip(l[2], -1, 1)); az = np.arctan2(l[1], l[0])
        lim = self.limit(az)
        if tilt <= lim:
            return u_cmd / np.linalg.norm(u_cmd), False, tilt, az
        return sph_dir(Rt, lim, az), True, lim, az


# ---------------------------------------------------------------- ball drives
class _Ball(Wrist):
    R_BALL, R_W, OFFSET = 0.025, 0.009, 0.034

    def __init__(self):
        self.roll = 0.0; self.prev_u = None; self.wheel_ang = np.zeros(3)

    def limit(self, az):
        return 115 * DEG

    def extras(self, parts, p, t, Rt, c, u, sep):
        pass

    def build(self, p, t, n, u_cmd, dt=1 / 30, defl=None, sep=None, u_force=None, roll_rate=0.0):
        Rt = frame_z(t, n)
        c0 = p + t * self.OFFSET
        u, clamped, _, _ = self.clamp(Rt, u_cmd)
        if defl is not None:
            u = rotvec(defl) @ u
        if u_force is not None:
            u = u_force / np.linalg.norm(u_force)
        sep = np.zeros(3) if sep is None else sep
        c = c0 + sep
        w = 0.0 if self.prev_u is None else np.arccos(np.clip(np.dot(self.prev_u, u), -1, 1)) / dt
        self.prev_u = u; self.roll += roll_rate * dt
        seated = np.linalg.norm(sep) < 1e-3
        if seated:
            self.wheel_ang += (w + abs(roll_rate)) * dt * self.R_BALL / self.R_W * np.array([1, -0.6, 0.8])
        parts = [frustum(p - t * 0.004, p + t * 0.006, 0.013, 0.012, DARK)]
        for k in range(3):
            az = np.radians(90 + 120 * k)
            e = Rt @ np.array([np.cos(az), np.sin(az), 0])
            pol = np.radians(128)
            nrm = np.cos(pol) * t + np.sin(pol) * e
            wc = c0 + nrm * (self.R_BALL + self.R_W)
            eph = np.cross(t, e)
            ref = rot(eph, self.wheel_ang[k]) @ nrm
            parts.append(frustum(wc - eph * 0.0045, wc + eph * 0.0045, self.R_W, self.R_W, DARK, n=12, ref=ref, c2=self.accent, stripes=6))
            mc = wc + eph * 0.0045
            parts.append(frustum(mc, mc + eph * 0.020, 0.0065, 0.0065, STEEL, n=10))          # N20 gearmotor
            parts.append(frustum(p + e * 0.010, mc + eph * 0.008, 0.0025, 0.0025, DARK, n=6))
        Rb = frame_z(u, n if abs(np.dot(n, u)) < 0.95 else t) @ rot([0, 0, 1], self.roll)
        parts.append(sphere(c, self.R_BALL, self.ball_col, n=18, m=12, R=Rb, col2=self.accent, checker=6))
        stalk_end = c + u * (self.R_BALL + 0.012)
        parts.append(frustum(c + u * (self.R_BALL - 0.003), stalk_end, 0.0055, 0.0055, DARK))
        if not seated:   # tether / wires from the collar to the ball
            parts.append(tube_path(np.array([p + t * 0.006, c]), 0.0012, self.tether_col, n=6))
        self.extras(parts, p, t, Rt, c0, u, sep)
        return merge(parts), u, stalk_end, Rb[:, 0], clamped


class BallOpen(_Ball):
    name = "BALL DRIVE (as drawn)"
    sub = "3 omni wheels on N20 motors push from behind; nothing holds the ball in"
    retention = "pull-out ~0 N (wheels push it OUT)"
    accent = [0.95, 0.55, 0.15]; ball_col = [0.92, 0.92, 0.90]; tether_col = WIRE


class BallCaptive(_Ball):
    name = "CAPTIVE BALL DRIVE"
    sub = "+3 front rollers (MR63 bearings on M3 bolts) ahead of the equator"
    retention = "pull-out >100 N est. (bolts/rollers)   tilt 55-100°"
    accent = [0.85, 0.45, 0.10]; ball_col = [0.92, 0.92, 0.90]; tether_col = WIRE
    RET_AZ = np.radians([30, 150, 270]); RET_POL = np.radians(72)

    def limit(self, az):
        d = np.min(np.abs((az - self.RET_AZ + np.pi) % (2 * np.pi) - np.pi))
        return np.radians(55 + 45 * np.clip((np.degrees(d) - 14) / 30, 0, 1))

    def extras(self, parts, p, t, Rt, c, u, sep):
        for a in self.RET_AZ:
            e = Rt @ np.array([np.cos(a), np.sin(a), 0])
            nrm = np.cos(self.RET_POL) * t + np.sin(self.RET_POL) * e
            rc = c + nrm * (self.R_BALL + 0.004)
            eph = np.cross(t, e)
            parts.append(frustum(rc - eph * 0.003, rc + eph * 0.003, 0.004, 0.004, STEEL, n=10))
            # strut from the collar, outside the ball, to the roller
            outside = c + e * (self.R_BALL + 0.010)
            parts.append(tube_path(np.array([p + e * 0.012, c - t * 0.018 + e * (self.R_BALL + 0.006), outside, rc + e * 0.004]), 0.0026, DARK, n=6))


class BallMagnetic(_Ball):
    name = "MAGNETIC BALL DRIVE"
    sub = "50 mm hollow steel ball, N52 Ø20×10 magnet, badge-reel tether"
    retention = "breaks away ~20 N est. (measure), reels back"
    accent = [0.80, 0.30, 0.55]; ball_col = [0.78, 0.80, 0.83]; tether_col = [0.15, 0.15, 0.15]

    def extras(self, parts, p, t, Rt, c, u, sep):
        parts.append(frustum(c - t * (self.R_BALL + 0.012), c - t * (self.R_BALL + 0.002), 0.010, 0.010, RED, n=16))
        parts.append(frustum(p - t * 0.004, p - t * 0.022, 0.011, 0.011, STEEL, n=14))      # badge reel


# ---------------------------------------------------------------- serial roll + pitch wrists
class _RollPitch(Wrist):
    RATE = 600 * DEG     # STS3032 no-load speed, about 0.1 s / 60 deg

    def __init__(self):
        self.roll = 0.0; self.pitch = 0.0

    def track(self, Rt, u_cmd, dt):
        """Greedy rate-limited tracking: try roll values reachable this frame (servo range +-180 deg, cable wrap),
        give each the best reachable pitch, keep the one that points the fan closest to the command."""
        u_c, clamped, tilt, az = self.clamp(Rt, u_cmd)
        l = Rt.T @ u_c
        m = self.RATE * dt
        lim = self.limit(az)
        best = None
        for r in np.clip(self.roll + np.linspace(-m, m, 25), -np.pi, np.pi):
            xr = np.array([np.cos(r), np.sin(r), 0])
            p = np.arctan2(l @ xr, l[2])
            p = np.clip(np.clip(p, -lim, lim), self.pitch - m, self.pitch + m)
            d = np.array([np.sin(p) * np.cos(r), np.sin(p) * np.sin(r), np.cos(p)])
            err = np.arccos(np.clip(d @ l, -1, 1))
            cost = err + 0.02 * abs(r) + 0.01 * abs(r - self.roll)
            if best is None or cost < best[0]:
                best = (cost, r, p)
        if np.degrees(best[0]) > 12 and tilt > 15 * DEG:
            # stuck (e.g. at the roll end stop): commit to the pose that is quickest to reach overall, and unwind
            opts = []
            for a, pt in ((az, tilt), (az + np.pi, -tilt)):
                for k in (-1, 0, 1):
                    r = (a + np.pi) % (2 * np.pi) - np.pi + 2 * np.pi * k
                    if abs(r) <= np.pi + 1e-9:
                        opts.append((max(abs(r - self.roll), abs(pt - self.pitch)), r, pt))
            _, r_t, p_t = min(opts)
            best = (0, self.roll + np.clip(r_t - self.roll, -m, m), self.pitch + np.clip(p_t - self.pitch, -m, m))
        _, self.roll, self.pitch = best
        Rr = Rt @ rot([0, 0, 1], self.roll)
        u = Rr @ np.array([np.sin(self.pitch), 0, np.cos(self.pitch)])
        return u, Rr, clamped


class Yoke(_RollPitch):
    name = "ROLL + PITCH YOKE"
    sub = "2 × STS3032 bus servos (same as arm tip), M3 bolts, 2 × MR85 bearings"
    retention = "pinned: >300 N est. on M3 pins; servo slips past ~0.4 N·m"
    accent = [0.25, 0.60, 0.35]

    def limit(self, az):
        return 135 * DEG

    def build(self, p, t, n, u_cmd, dt=1 / 30, defl=None, sep=None, u_force=None, roll_rate=0.0):
        Rt = frame_z(t, n)
        u, Rr, clamped = self.track(Rt, u_cmd, dt)
        if defl is not None:
            u = rotvec(defl) @ u
        sep = np.zeros(3) if sep is None else sep
        x, y = Rr[:, 0], Rr[:, 1]
        parts = [box(p + t * 0.016, Rt, [0.024, 0.013, 0.032], STEEL)]                  # roll servo, in line
        parts.append(frustum(p + t * 0.032, p + t * 0.036, 0.012, 0.012, self.accent, n=16, ref=x, c2=DARK, stripes=4))
        c = p + t * 0.058 + sep
        for s in (-1, 1):                                                                 # yoke ears
            parts.append(box(p + t * 0.047 + y * s * 0.017, Rr, [0.008, 0.004, 0.026], DARK))
        parts.append(box(c + y * 0.027, frame_z(y, t), [0.024, 0.023, 0.013], STEEL))    # pitch servo on an ear
        parts.append(frustum(c - y * 0.019, c + y * 0.019, 0.003, 0.003, DARK, n=8))       # pitch pin
        parts.append(frustum(c - y * 0.014, c + y * 0.014, 0.010, 0.010, self.accent, n=14, ref=u))  # pitch hub
        stalk_end = c + u * 0.024
        parts.append(frustum(c, stalk_end, 0.0065, 0.0065, DARK))
        return merge(parts), u, stalk_end, y, clamped


class Knuckle(_RollPitch):
    name = "GEARED KNUCKLE + ROLL"
    sub = "2 × STS3032, two hinges tied by printed m1 spur gears, M3 pins"
    retention = "pinned: >300 N est.; gear teeth are the weak point"
    accent = [0.20, 0.50, 0.85]

    def limit(self, az):
        return 170 * DEG

    def build(self, p, t, n, u_cmd, dt=1 / 30, defl=None, sep=None, u_force=None, roll_rate=0.0):
        Rt = frame_z(t, n)
        u, Rr, clamped = self.track(Rt, u_cmd, dt)
        pitch = self.pitch
        if defl is not None:
            u = rotvec(defl) @ u
        sep = np.zeros(3) if sep is None else sep
        x, y = Rr[:, 0], Rr[:, 1]
        parts = [box(p + t * 0.016, Rt, [0.024, 0.013, 0.032], STEEL)]
        parts.append(frustum(p + t * 0.032, p + t * 0.036, 0.012, 0.012, self.accent, n=16, ref=x, c2=DARK, stripes=4))
        h1 = p + t * 0.046 + sep
        mid = Rr @ np.array([np.sin(pitch / 2), 0, np.cos(pitch / 2)])
        h2 = h1 + mid * 0.024
        parts.append(tube_path(np.array([p + t * 0.036, h1]), 0.006, DARK))
        parts.append(box(0.5 * (h1 + h2), frame_z(mid, x), [0.016, 0.026, 0.024], DARK))   # centre link
        for h, ang in ((h1, pitch / 2), (h2, -pitch / 2)):
            for s in (-1, 1):
                ref = rot(y, ang * 4) @ x
                parts.append(frustum(h + y * s * 0.014, h + y * s * 0.019, 0.011, 0.011, self.accent, n=18, ref=ref, c2=DARK, stripes=9))
            parts.append(frustum(h - y * 0.020, h + y * 0.020, 0.0025, 0.0025, STEEL, n=8))
        parts.append(box(h1 - mid * 0.004 + y * 0.030, frame_z(y, t), [0.024, 0.023, 0.013], STEEL))  # pitch servo
        stalk_end = h2 + u * 0.016
        parts.append(frustum(h2, stalk_end, 0.0065, 0.0065, DARK))
        return merge(parts), u, stalk_end, y, clamped


class Flex(Wrist):
    name = "TPU FLEX WRIST"
    sub = "printed TPU 95A spine, 4 Dyneema tendons, 2 × STS3032 in the tip"
    retention = "bends and springs back; tilt 75°, no roll"
    accent = [0.30, 0.75, 0.55]
    L = 0.048

    def limit(self, az):
        return 75 * DEG

    def build(self, p, t, n, u_cmd, dt=1 / 30, defl=None, sep=None, u_force=None, roll_rate=0.0):
        Rt = frame_z(t, n)
        u, clamped, _, _ = self.clamp(Rt, u_cmd)
        if defl is not None:
            u = rotvec(defl) @ u
        sep = np.zeros(3) if sep is None else sep
        ax = np.cross(t, u); th = np.arccos(np.clip(np.dot(t, u), -1, 1))
        ax = ax / np.linalg.norm(ax) if np.linalg.norm(ax) > 1e-9 else Rt[:, 0]
        k = 7
        pts, Rs = [p + t * 0.004], []
        L = self.L + np.dot(sep, u)          # axial stretch under a yank
        for i in range(k + 1):
            R = rot(ax, th * i / k) @ Rt
            Rs.append(R)
            if i:
                d = rot(ax, th * (i - 0.5) / k) @ t
                pts.append(pts[-1] + d * L / k)
        parts = [box(p - t * 0.012 + Rt[:, 0] * 0.020, Rt, [0.013, 0.024, 0.025], STEEL),
                 box(p - t * 0.012 - Rt[:, 0] * 0.020, Rt, [0.013, 0.024, 0.025], STEEL)]
        parts.append(tube_path(np.array(pts), 0.0055, TPU, n=10))
        for i, (q, R) in enumerate(zip(pts, Rs)):
            parts.append(frustum(q - R[:, 2] * 0.0015, q + R[:, 2] * 0.0015, 0.016, 0.016, TPU if i % 2 else DARK, n=16))
        for a in range(4):
            tp = np.array([q + R @ np.array([0.013 * np.cos(a * np.pi / 2), 0.013 * np.sin(a * np.pi / 2), 0]) for q, R in zip(pts, Rs)])
            parts.append(tube_path(tp, 0.0009, [0.95, 0.95, 0.95], n=5))
        stalk_end = pts[-1] + u * 0.008
        parts.append(frustum(pts[-1], stalk_end, 0.0065, 0.0065, DARK))
        return merge(parts), u, stalk_end, Rs[-1][:, 0], clamped


ALL = [BallOpen, BallCaptive, BallMagnetic, Yoke, Knuckle, Flex]
