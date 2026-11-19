"""Applies the XL-pod hooks (2026-11-13, xlpod4.py) to a restored workspace. Idempotent: run once after restoring the
project code, before any XL full-robot run.   python3 xlpod_patch.py
  cephalo4.py   toe() / TOE / XL_POD (folded-fingertip position as a function; default = the old CLAW_R stick)
  sim4.py       toe contact spheres from cephalo4.toe()
  viz4.py       arm_mesh draws the XL pod from its CAD when cephalo4.XL_POD is set
  power4.py     XL pod mass 190 g budget -> 156 g itemised (dM_pod 0.075 -> 0.041)
  fall_video.py, pen_chk.py   xlpod4.install() instead of setting CLAW_R; fall_video: FV_FILE_<case>, FV_TITLE_<case>
(fall4.py already calls xlpod4.install().)"""
EDITS = [
 ("cephalo4.py", '''def foot_drop(fold=FOLD_STAND):
    return BLADE_OFF + CLAW_R * np.sin(fold) + 0.004''', '''TOE = None      # optional: function fold -> (radial, axial) of the folded fingertip from the hub centre (XL claw: xlpod4.py)
XL_POD = False  # viz4 draws the XL pod from its CAD (xlpod4.py)


def toe(fold=FOLD_STAND):
    """Fingertip of a folded claw finger: (radial, axial) from the hub centre. Default: a straight CLAW_R stick."""
    if TOE is not None:
        return TOE(fold)
    return CLAW_R * np.cos(fold), CLAW_R * np.sin(fold)


def foot_drop(fold=FOLD_STAND):
    return BLADE_OFF + toe(fold)[1] + 0.004'''),
 ("sim4.py", '''rr = C.CLAW_R * np.cos(C.FOLD_STAND); xx = C.BLADE_OFF + C.CLAW_R * np.sin(C.FOLD_STAND)''', '''rr, xx = C.toe(C.FOLD_STAND); xx = C.BLADE_OFF + xx'''),
 ("viz4.py", '''    S.add(box(pt - u * 0.012, Rt, [0.024, 0.024, 0.016], [0.35, 0.36, 0.40]))
''', '''    S.add(box(pt - u * 0.012, Rt, [0.024, 0.024, 0.016], [0.35, 0.36, 0.40]))
    if C.XL_POD:                                          # XL claw-prop drawn from its CAD (xlpod4.py, xl-claw.md)
        __import__("xlpod4").draw(S, pt, Rt, spin, fold, fan_on, warn)
        return
'''),
 ("power4.py", '''cap=28.0, dM_pod=0.075,''', '''cap=28.0, dM_pod=0.041,'''),
 ("fall_video.py", '''C.CLAW_R = PW.PROPS["xl"]["R"]''', '''import xlpod4; xlpod4.install()'''),
 ("pen_chk.py", '''C.CLAW_R = PW.PROPS["xl"]["R"]''', '''import xlpod4; xlpod4.install()'''),
 ("fall_video.py", '''Z = {c: np.load(f"fall4_{c}.npz") for c in CASES}
J = {c: json.load(open(f"fall4_{c}.json")) for c in CASES}''', '''SRC = {c: os.environ.get(f"FV_FILE_{c}", f"fall4_{c}") for c in CASES}     # FV_FILE_hit=fall4_hit_ts30_0 shows another run of that case
Z = {c: np.load(SRC[c] + ".npz") for c in CASES}
J = {c: json.load(open(SRC[c] + ".json")) for c in CASES}'''),
 ("fall_video.py", '''TITLE = {c: f"{CASES.index(c) + 1} / {len(CASES)}" + TITLE[c][5:] for c in CASES}''',
  '''TITLE = {c: f"{CASES.index(c) + 1} / {len(CASES)}" + os.environ.get(f"FV_TITLE_{c}", TITLE[c][5:]) for c in CASES}'''),
]
if __name__ == "__main__":
    for f, a, b in EDITS:
        s = open(f).read()
        if b in s: print("already applied:", f); continue
        assert s.count(a) == 1, f"{f}: expected text not found (file differs from the 2026-11-13 base)"
        open(f, "w").write(s.replace(a, b)); print("patched:", f)
