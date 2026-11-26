import math
import numpy as np
import pytest
from kin import Kin, axis_dir, rot_about_line, named_pose
import pin_cad as C


def test_axis_dir_unit_and_helix():
    for j in range(13):
        a = axis_dir(j, 60)
        assert abs(np.linalg.norm(a) - 1) < 1e-12 and a[2] == 0
        assert abs(math.degrees(math.acos(np.clip(axis_dir(j, 60) @ axis_dir(j + 1, 60), -1, 1))) - 60) < 1e-9
    assert np.allclose(axis_dir(6, 60), axis_dir(0, 60))          # 360 deg later


def test_matches_pin_cad():
    for j in range(5):
        assert np.allclose(axis_dir(j, 60), C.axis_dir(j, 60))
        p, a = np.array([0, 0, 10.0 * j]), axis_dir(j, 60)
        assert np.allclose(rot_about_line(p, a, 0.7), C.rot_about_line(p, a, 0.7))


@pytest.mark.parametrize("q", [0.3, -1.2, math.pi])
def test_rot_about_line_fixes_axis_and_is_rigid(q):
    p, a = np.array([1.0, 2.0, 3.0]), np.array([0.0, 1.0, 1.0])
    T = rot_about_line(p, a, q)
    R = T[:3, :3]
    assert np.allclose(R @ R.T, np.eye(3)) and abs(np.linalg.det(R) - 1) < 1e-12
    for t in (-2.0, 0.0, 5.0):                                     # points on the line do not move
        x = p + t * a / np.linalg.norm(a)
        assert np.allclose((T @ np.append(x, 1))[:3], x)
    assert np.allclose(rot_about_line(p, a, 0.0), np.eye(4))


def test_joint_affine_orthonormal():
    for j in range(13):
        M = C.joint_affine(j, 60, 76.9)
        assert np.allclose(M[:, :3].T @ M[:, :3], np.eye(3)) and abs(np.linalg.det(M[:, :3]) - 1) < 1e-12
        assert abs(M[2, 3] - j * 76.9) < 1e-9


def test_straight_tip_at_length():
    k = Kin(13, 1000 / 13, 60)
    assert np.allclose(k.tip(np.zeros(13)), [0, 0, 1000])
    assert abs(k.length - 1000) < 1e-9


def test_fk_preserves_link_length():
    k = Kin(13, 1000 / 13, 60); rng = np.random.default_rng(3)
    for _ in range(5):
        q = rng.uniform(-0.8, 0.8, 13)
        W = k.frames(q)
        for i in range(13):                                         # joint points keep their spacing
            p0 = (W[i] @ np.append(k.point(i), 1))[:3]; p1 = (W[i + 1] @ np.append(k.point(i + 1), 1))[:3]
            assert abs(np.linalg.norm(p1 - p0) - k.pitch) < 1e-6


def test_jacobian_matches_finite_difference():
    k = Kin(13, 1000 / 13, 60); q = np.radians(np.random.default_rng(1).uniform(-30, 30, 13))
    Jm = k.jacobian(q)[:3]; eps = 1e-6
    for j in range(13):
        dq = np.zeros(13); dq[j] = eps
        fd = (k.tip(q + dq) - k.tip(q - dq)) / (2 * eps)
        assert np.allclose(Jm[:, j], fd, atol=1e-3)


def test_dof_ranks():
    k = Kin(13, 1000 / 13, 60)
    assert k.dof(named_pose(k, "straight"))[0] == 4                 # straight arm is singular
    assert k.dof(named_pose(k, "wave"))[0] == 6
    q = np.radians(np.random.default_rng(0).uniform(-45, 45, 13))
    assert k.dof(q)[0] == 6                                         # generic pose: full 6 DOF, 7 redundant


def test_ik_reaches_reachable_target():
    k = Kin(13, 1000 / 13, 60)
    target = k.tip(named_pose(k, "wave"))
    q, err = k.ik_position(target, q0=np.radians(np.random.default_rng(0).uniform(-5, 5, 13)))
    assert err < 1e-3
