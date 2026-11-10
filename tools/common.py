"""Shared helpers for the cephalo-1 skills (import with `sys.path.insert(0, <repo>/tools)`).

Every tool in this folder imports the CAD modules from arm-pin-hinge/ through here, so that
paths, the default design (1 m, 13 joints, 60 deg helix, geared root, 5 STS3250) and the
report/exit-code conventions live in one place.
"""
import json, math, os, sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PIN_DIR = os.path.join(ROOT, "arm-pin-hinge")
SIM_DIR = os.path.join(ROOT, "joint-study", "sim")
MAIN_OUT = os.path.join(PIN_DIR, "out_1m_13")      # committed main design

DEFAULT = dict(length=1000.0, joints=13, twist=60.0, rng=90.0, root_servos=5)


def add_paths(sim=False):
    for p in (PIN_DIR, os.path.join(ROOT, "spec-sheet")) + ((SIM_DIR,) if sim else ()):
        if p not in sys.path:
            sys.path.insert(0, p)


def build_arm(length=None, joints=None, twist=None, rng=None, geared=True):
    """PinArm (or GearedRootArm, the main design) for the given parameters; None -> main design."""
    add_paths()
    import pin_cad as C
    d = DEFAULT
    length = d["length"] if length is None else length
    joints = d["joints"] if joints is None else joints
    twist = d["twist"] if twist is None else twist
    rng = d["rng"] if rng is None else rng
    if geared:
        import root_gear as R
        return R.GearedRootArm(length, twist, joints, rng)
    return C.PinArm(length, twist, joints, rng)


def build_bodies(arm):
    """link bodies 0..J (printed part + servo + lid + previous horn), in the straight pose"""
    return [arm.body(i) for i in range(arm.J + 1)]


def load_json(path):
    with open(path) as f:
        return json.load(f)


def parse_joint_arg(arg, n):
    """'3' -> [3], 'all' -> range(n), '0,2,5' -> list, '2-6' -> range"""
    if arg in (None, "all"):
        return list(range(n))
    out = []
    for part in str(arg).split(","):
        if "-" in part:
            a, b = part.split("-")
            out += list(range(int(a), int(b) + 1))
        else:
            out.append(int(part))
    bad = [j for j in out if not 0 <= j < n]
    if bad:
        raise SystemExit(f"joint index {bad} out of range 0..{n - 1}")
    return out


def table(rows, header):
    """plain-text table; rows are lists of values"""
    rows = [[str(c) for c in r] for r in rows]
    w = [max(len(str(h)), *(len(r[i]) for r in rows)) if rows else len(str(h)) for i, h in enumerate(header)]
    line = lambda r: "  ".join(str(c).ljust(w[i]) for i, c in enumerate(r))
    return "\n".join([line(header), line(["-" * x for x in w])] + [line(r) for r in rows])


class Checks:
    """collect PASS/FAIL results, print a table and give an exit code"""
    def __init__(self, title=""):
        self.title, self.rows = title, []

    def add(self, name, ok, detail="", warn=False):
        """warn=True: a failing check is reported as WARN and does not change the exit code"""
        self.rows.append((name, bool(ok), detail, warn))
        return bool(ok)

    @property
    def failed(self):
        return [r for r in self.rows if not r[1] and not r[3]]

    @property
    def warned(self):
        return [r for r in self.rows if not r[1] and r[3]]

    def report(self):
        st = lambda ok, w: "PASS" if ok else ("WARN" if w else "FAIL")
        t = table([[st(ok, w), n, d] for n, ok, d, w in self.rows], ["", "check", "detail"])
        s = (f"{self.title}\n{t}\n{len(self.rows) - len(self.failed) - len(self.warned)}/{len(self.rows)} passed"
             + (f", {len(self.warned)} warning(s)" if self.warned else "") + (f", {len(self.failed)} FAILED" if self.failed else ""))
        print(s)
        return s

    def exit(self):
        self.report()
        sys.exit(1 if self.failed else 0)
