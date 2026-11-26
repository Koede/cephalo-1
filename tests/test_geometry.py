import math
import numpy as np
import pytest
import pin_cad as C
import root_gear as R

@pytest.fixture(scope="module")
def arm(): return R.GearedRootArm(1000, 60, 13)

def test_pitch_and_counts(arm):
    assert arm.J == 13 and abs(arm.pitch * 13 - 1000) < 1e-9 and arm.pitch >= arm.pitch_min

def test_involute_gear_tooth_count_and_radius():
    pts = R.involute_gear(40)
    assert len(pts) == 40 * 18                                      # 2 flanks x 8 points + 2 root points per tooth (root circle is inside the base circle)
    rs = [math.hypot(x, y) for x, y in pts]
    assert abs(max(rs) - (20 * R.MOD + R.MOD)) < 0.05               # tip radius = z/2*m + m
    assert abs(min(rs) - (20 * R.MOD - 1.25 * R.MOD)) < 0.05           # root circle = rp - 1.25 m

def test_gear_mesh_centre_distance():
    assert abs(R.CDIST - (R.Z_GEAR + R.Z_PIN) / 2 * R.MOD) < 1e-9

def test_identical_vertebra(arm):
    P = arm.pitch; t = math.radians(60)
    Rz = np.eye(4); Rz[:2, :2] = [[math.cos(t), -math.sin(t)], [math.sin(t), math.cos(t)]]; Rz[2, 3] = P
    d = arm.part(2) - arm.part(1).transform(Rz[:3, :])
    assert d.volume() < 0.5

def test_static_clear(arm):
    b = [arm.body(i) for i in range(arm.J + 1)]
    assert all((b[i] ^ b[i + 1]).volume() < 0.5 for i in range(arm.J))
