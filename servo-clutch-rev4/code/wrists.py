"""The two candidate tip wrists, as kinematics + meshes.

Both take the arm tip pose (point p, unit direction t, reference 'up' vector n for the arm) and a commanded pod
direction u_cmd, clamp it to the wrist's reachable set, and return (mesh_parts, u_actual, pod_base_point, pod_roll_ref, clamped).

BallDrive  - 50 mm hollow ball driven by three 18 mm omni wheels on micro gearmotors, all mounted BEHIND the ball's
             equator on a light tripod collar.  Output stalk leaves the front of the ball.  3 DOF: tilt in any
             direction up to TILT_MAX (stalk meets the wheel tripod), unlimited roll about the stalk.
OmniWrist  - Ross-Hime-style double-spherical parallel linkage: base ring on the arm tip, output ring on the pod,
             4 legs of two curved links each meeting on the bisecting plane (constant-velocity symmetry).
             2 DOF: pitch/yaw up to TILT_MAX (90 deg, hemisphere), roll is fixed by the linkage.
             2 micro servos on the base drive legs 0 and 1.
Limits are geometric estimates from these models, not from a built part.
"""
import numpy as np
from geom import frustum, sphere, torus, box, tube_path, merge, frame_z, rot

DARK = [0.18, 0.19, 0.21]; STEEL = [0.62, 0.64, 0.67]; ACC_B = [0.95, 0.55, 0.15]; ACC_O = [0.20, 0.55, 0.85]
RED = [0.90, 0.15, 0.12]


def clamp_dir(t, u_cmd, max_ang):
    u = u_cmd / np.linalg.norm(u_cmd)
    ang = np.arccos(np.clip(np.dot(t, u), -1, 1))
    if ang <= max_ang:
        return u, False, ang
    ax = np.cross(t, u)
    if np.linalg.norm(ax) < 1e-9:  # pointing straight back: pick any side
        ax = frame_z(t)[:, 0]
    return rot(ax, max_ang) @ t, True, ang


class BallDrive:
    name = "BALL DRIVE"
    sub = "omni-wheel ball  |  3 DOF  |  tilt 115° any way, roll unlimited"
    TILT_MAX = np.radians(115)
    R_BALL = 0.025
    OFFSET = 0.034          # ball centre ahead of arm tip
    R_W = 0.009             # omni wheel radius
    accent = ACC_B

    def __init__(self):
        self.roll = 0.0     # accumulated roll of ball/pod about the stalk
        self.prev_u = None
        self.wheel_ang = np.zeros(3)

    def lead(self):
        return self.OFFSET

    def c_to_stalk(self):
        return self.R_BALL + 0.012

    def build(self, p, t, n, u_cmd, roll_rate=0.0, dt=1 / 30):
        c = p + t * self.OFFSET
        u, clamped, _ = clamp_dir(t, u_cmd, self.TILT_MAX)
        # wheel spin: proportional to ball angular speed (visual)
        w = 0.0
        if self.prev_u is not None:
            w = np.arccos(np.clip(np.dot(self.prev_u, u), -1, 1)) / dt
        self.prev_u = u
        self.roll += roll_rate * dt
        self.wheel_ang += (w + abs(roll_rate)) * dt * self.R_BALL / self.R_W * np.array([1, -0.6, 0.8])
        Rt = frame_z(t, n)
        parts = []
        # tripod collar: short neck from arm tip, three spokes to the motors
        parts.append(frustum(p - t * 0.004, p + t * 0.006, 0.013, 0.012, DARK))
        for k in range(3):
            az = np.radians(90 + 120 * k)
            e = Rt @ np.array([np.cos(az), np.sin(az), 0])          # radial in tip frame
            pol = np.radians(128)                                     # contact polar angle from t
            nrm = np.cos(pol) * t + np.sin(pol) * e                   # outward normal at contact
            wc = c + nrm * (self.R_BALL + self.R_W)
            eph = np.cross(t, e)                                      # azimuthal tangent = wheel axle
            # omni wheel with striped rollers, spinning
            ref = rot(eph, self.wheel_ang[k]) @ nrm
            parts.append(frustum(wc - eph * 0.0045, wc + eph * 0.0045, self.R_W, self.R_W, DARK, n=12, ref=ref, c2=self.accent, stripes=6))
            # gearmotor body along the axle, then spoke back to collar
            mc = wc + eph * 0.0045
            parts.append(frustum(mc, mc + eph * 0.020, 0.0065, 0.0065, STEEL, n=10))
            parts.append(frustum(p + e * 0.010, mc + eph * 0.008, 0.0025, 0.0025, DARK, n=6))
        # ball: checker pattern rotates with the pod so roll is visible
        Rb = frame_z(u, n if abs(np.dot(n, u)) < 0.95 else t)
        Rb = Rb @ rot([0, 0, 1], self.roll)
        parts.append(sphere(c, self.R_BALL, [0.92, 0.92, 0.90], n=18, m=12, R=Rb, col2=self.accent, checker=6))
        base = c + u * self.R_BALL
        stalk_end = base + u * 0.012
        parts.append(frustum(c + u * (self.R_BALL - 0.003), stalk_end, 0.0055, 0.0055, DARK))
        roll_ref = Rb[:, 0]
        return merge(parts), u, stalk_end, roll_ref, clamped


class OmniWrist:
    name = "OMNI-WRIST"
    sub = "parallel spherical linkage  |  2 DOF  |  tilt 90° any way, no roll"
    TILT_MAX = np.radians(90)
    D = 0.025        # base / output ring offset from the virtual centre
    R_RING = 0.030
    R_MID = 0.050    # leg elbow radius from the virtual centre
    OFFSET = 0.028
    accent = ACC_O

    def lead(self):
        return self.OFFSET + self.D

    def c_to_stalk(self):
        return self.D + 0.012

    def build(self, p, t, n, u_cmd, roll_rate=0.0, dt=1 / 30):
        c = p + t * (self.OFFSET + self.D)
        u, clamped, _ = clamp_dir(t, u_cmd, self.TILT_MAX)
        Rt = frame_z(t, n)
        h = t + u
        if np.linalg.norm(h) < 1e-6:
            h = Rt[:, 0]
        nm = h / np.linalg.norm(h)                         # bisecting-plane normal

        def refl(x):
            v = x - c
            return c + v - 2 * np.dot(v, nm) * nm         # mirror across bisecting plane, then
        parts = []
        bc = c - t * self.D
        oc = refl(bc)
        parts.append(frustum(p - t * 0.004, bc, 0.013, 0.012, DARK))
        parts.append(torus(bc, t, self.R_RING, 0.0032, DARK, ref=Rt[:, 0]))
        parts.append(torus(oc, u, self.R_RING, 0.0032, DARK, ref=Rt[:, 0]))
        for k in range(4):
            az = np.radians(45 + 90 * k)
            e = Rt @ np.array([np.cos(az), np.sin(az), 0])
            B = bc + e * self.R_RING
            O = refl(B)
            mid = 0.5 * (B + O)
            dm = mid - c; dm -= np.dot(dm, nm) * nm
            if np.linalg.norm(dm) < 1e-6:
                dm = e
            M = c + dm / np.linalg.norm(dm) * self.R_MID
            def arc(a, b):
                ctrl = 0.5 * (a + b); w = ctrl - c; ctrl = c + w / (np.linalg.norm(w) + 1e-9) * self.R_MID * 0.98
                s = np.linspace(0, 1, 6)[:, None]
                return (1 - s) ** 2 * a + 2 * (1 - s) * s * ctrl + s ** 2 * b
            col = self.accent if k < 2 else STEEL
            parts.append(tube_path(arc(B, M), 0.0034, col))
            parts.append(tube_path(arc(M, O), 0.0034, col))
            parts.append(sphere(B, 0.005, DARK, n=8, m=6))
            parts.append(sphere(M, 0.0055, DARK, n=8, m=6))
            parts.append(sphere(O, 0.005, DARK, n=8, m=6))
            if k < 2:   # actuated legs: micro servo beside the base ring
                sc = bc + e * (self.R_RING + 0.014) - t * 0.014
                parts.append(box(sc, frame_z(t, e), [0.013, 0.024, 0.023], STEEL))
        stalk_end = oc + u * 0.012
        parts.append(frustum(oc, stalk_end, 0.0055, 0.0055, DARK))
        roll_ref = refl(bc + Rt[:, 0]) - oc
        roll_ref -= np.dot(roll_ref, u) * u
        return merge(parts), u, stalk_end, roll_ref / (np.linalg.norm(roll_ref) + 1e-9), clamped
