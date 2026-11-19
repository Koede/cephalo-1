"""body_video.py for a build with hand3 on arms 2 and 6: the hands are drawn from their CAD (hand3_draw.py), fingers straight
and spinning as the 10 in four-blade rotor whenever that arm's thrust is on.
python3 bv.py still <t> body4_h3d_mid_day.npz | chunk <i> <n> <npz> | join <n> <npz>"""
import os, sys
os.environ.update(CEPH_PROP="xl", CEPH_PACKS="2", CEPH_MONITOR="0")
import cephalo4 as C, xlpod4; xlpod4.install()
import hand3_draw as H3
H3.install((2, 6))
src = open("body_video.py").read()
src = src.replace('"Claw-prop max (est.)"', '"XL claw-prop / hand rotor max"')        # HUD wording for this build
src = src.replace('.get(CASE, 12.6)', '.get(CASE, 28.0)')
sys.argv = ["body_video.py"] + sys.argv[1:]
exec(compile(src, "body_video.py", "exec"), {"__name__": "__main__", "__file__": "body_video.py"})
