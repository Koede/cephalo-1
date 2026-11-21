"""Run every tools/*.py script as a user would (own process) and check the exit code and a key line of output."""
import importlib.util, os, subprocess, sys
import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def run(*args, cwd=ROOT):
    r = subprocess.run([sys.executable, os.path.join(ROOT, "tools", args[0]), *args[1:]], cwd=cwd, capture_output=True, text=True, timeout=300)
    return r.returncode, r.stdout + r.stderr


@pytest.mark.parametrize("script,args,expect", [
    ("check_gates.py", [], "passed"),
    ("mass_budget.py", [], "matches"),
    ("gear_check.py", [], "contact ratio"),
    ("fastener_check.py", [], "horn bolt"),
    ("symmetry_8arm.py", [], "45 deg spacing"),
    ("design_doc_sync.py", [], "passed"),
    ("study_report.py", ["bearings"], "gear brg"),
    ("study_report.py", ["joints"], "joint 1"),
    ("study_report.py", ["servos"], "STS3250"),
    ("study_report.py", ["battery"], "Tattu"),
    ("joint_dof.py", ["--samples", "20"], "revolute"),
    ("joint_range.py", ["--joint", "0", "--step", "5", "--out", "{tmp}/jr.json"], "55"),
    ("collision.py", ["--level", "static"], "passed"),
    ("singularity_scan.py", ["--samples", "50"], "rank<6 0"),
    ("workspace_reach.py", ["--samples", "500"], "max reach"),
    ("variant_compare.py", ["--variant", "1000:13:60"], "ok"),
    ("param_sweep.py", ["joints", "12", "13"], "pitch"),
    ("bom_export.py", ["--csv", "{tmp}/bom.csv"], "STS3250"),
    ("print_prep.py", [], "Parts to print"),
    ("repo_map.py", [], "pin_cad.py"),
    ("stale_outputs.py", [], "generators"),
    ("doctor.py", ["--fast"], "doctor"),
])
def test_tool_runs(script, args, expect, tmp_path):
    rc, out = run(script, *[a.replace("{tmp}", str(tmp_path)) for a in args])
    assert rc == 0, out[-800:]
    assert expect in out, out[-800:]


def test_joint_range_flags_short_travel(tmp_path):
    rc, out = run("joint_range.py", "--joint", "0", "--step", "5", "--need", "60", "--out", str(tmp_path / "x.json"))
    assert rc == 1 and "SHORT" in out and "55" in out          # geared root limits joint 0 to +-55


def test_collision_flags_bad_pose():
    rc, out = run("collision.py", "--pose", "0:70")
    assert rc == 1 and "link 0 vs 1" in out


@pytest.mark.skipif(importlib.util.find_spec("mujoco") is None, reason="mujoco not installed")
def test_mujoco_sanity():
    rc, out = run("mujoco_sanity.py", "--poses", "5")
    assert rc == 0, out[-600:]
