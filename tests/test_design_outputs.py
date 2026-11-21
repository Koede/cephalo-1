"""Gates on the committed main design (arm-pin-hinge/out_1m_13): fast, no regeneration."""
import json, os
import pytest
from common import MAIN_OUT

def J(*p): return json.load(open(os.path.join(MAIN_OUT, *p)))

def test_fit_gates():
    f = J("fit.json")
    assert f["static"] < 0.5 and f["pitch"] >= f["pitch_min"]
    for j, lo, hi in f["sweeps"]: assert lo <= -45 and hi >= 45
    assert all(not bad for _, bad in f["pairs"])

def test_root_gear_gates():
    g = J("root_gear.json")
    assert g["thrust_to_weight"] >= 1.15 - 1e-3
    assert g["added_g_per_arm"] <= g["budget_g"]

def test_joint0_sweep_is_the_tight_one():
    s = {j: (lo, hi) for j, lo, hi in J("fit.json")["sweeps"]}
    assert s[0][1] < s[1][1]                                       # joint 0 limited (+-55), the rest free (+-90)

def test_arm_counts():
    f = J("fit.json"); assert f["joints"] == 13 and f["root_servos"] == 5
