"""Pure-numpy kinematic model of one arm: joint axes, FK, Jacobian, DOF analysis.

Frames (as pin_cad.py): arm axis z (root -> tip); joint j sits at z = j*pitch with its hinge
axis a_j in the x-y plane at angle j*twist from +x. Rotations compose root -> tip, each one about
the *parent-frame* axis line (pin_cad.pose_frames), so no geometry is needed.
"""
import math
import numpy as np


def axis_dir(j, twist):
    t = math.radians(j * twist)
    return np.array([math.cos(t), math.sin(t), 0.0])


def rot_about_line(p, a, q):
    a = np.asarray(a, float); a = a / np.linalg.norm(a)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    R = np.eye(3) + math.sin(q) * K + (1 - math.cos(q)) * K @ K
    T = np.eye(4); T[:3, :3] = R; T[:3, 3] = p - R @ p
    return T


class Kin:
    def __init__(self, joints=13, pitch=1000.0 / 13, twist=60.0):
        self.J, self.pitch, self.twist = joints, pitch, twist
        self.length = joints * pitch

    @classmethod
    def from_arm(cls, arm):
        return cls(arm.J, arm.pitch, arm.twist)

    def point(self, j):
        return np.array([0.0, 0.0, j * self.pitch])

    def frames(self, q):
        """W[i] = transform carrying link i (straight pose) to its pose; W[0] = I, length J+1"""
        W = [np.eye(4)]
        for j in range(self.J):
            W.append(W[-1] @ rot_about_line(self.point(j), axis_dir(j, self.twist), q[j]))
        return W

    def tip(self, q):
        return (self.frames(q)[-1] @ np.append(self.point(self.J), 1.0))[:3]

    def tip_frame(self, q):
        W = self.frames(q)[-1]
        T = np.eye(4); T[:3, :3] = W[:3, :3]; T[:3, 3] = W[:3, :3] @ self.point(self.J) + W[:3, 3]
        return T

    def world_axes(self, q):
        """(point on axis, unit direction) of every joint in the world frame at pose q"""
        W = self.frames(q)
        out = []
        for j in range(self.J):
            p = (W[j] @ np.append(self.point(j), 1.0))[:3]
            a = W[j][:3, :3] @ axis_dir(j, self.twist)
            out.append((p, a / np.linalg.norm(a)))
        return out

    def jacobian(self, q, point=None):
        """6 x J geometric Jacobian of the tip (or `point` = world point): rows = [v; w]"""
        tip = self.tip(q) if point is None else np.asarray(point, float)
        Jm = np.zeros((6, self.J))
        for j, (p, a) in enumerate(self.world_axes(q)):
            Jm[:3, j] = np.cross(a, tip - p)
            Jm[3:, j] = a
        return Jm

    def ik_position(self, target, q0=None, iters=200, tol=1e-6, damping=1e-2, limit=None):
        """damped-least-squares position IK (3 DOF); returns (q, error mm). limit = rad bound per joint"""
        q = np.zeros(self.J) if q0 is None else np.array(q0, float)
        for _ in range(iters):
            e = np.asarray(target, float) - self.tip(q)
            if np.linalg.norm(e) < tol:
                break
            Jp = self.jacobian(q)[:3]
            dq = Jp.T @ np.linalg.solve(Jp @ Jp.T + damping * np.eye(3), e)
            q = q + dq
            if limit is not None:
                q = np.clip(q, -limit, limit)
        return q, float(np.linalg.norm(np.asarray(target, float) - self.tip(q)))

    def dof(self, q, rtol=1e-6):
        """rank, singular values and condition number of the 6 x J tip Jacobian"""
        s = np.linalg.svd(self.jacobian(q), compute_uv=False)
        rank = int((s > rtol * max(s[0], 1e-12)).sum())
        return rank, s, float(s[0] / s[-1]) if s[-1] > 1e-12 else float("inf")


def named_pose(kin, name, half_deg=40.5):
    """the poses pin_cad.py writes: straight / curl / wave (+- half_deg), plus zigzag and coil"""
    J, h = kin.J, math.radians(half_deg)
    if name == "straight":
        return np.zeros(J)
    if name == "curl":
        return np.full(J, h)
    if name == "wave":
        return np.array([h * math.sin(2 * math.pi * k / J) for k in range(J)])
    if name == "zigzag":
        return np.array([h * (-1) ** k for k in range(J)])
    if name == "coil":
        return np.full(J, math.radians(45.0))
    raise SystemExit(f"unknown pose '{name}' (straight, curl, wave, zigzag, coil)")
