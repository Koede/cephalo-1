import os, sys, runpy
LV="low"; M_HAND=0.570
os.environ.update(CEPH_PROP="xl", CEPH_PACKS="2", CEPH_MONITOR="0", H3_FINGERS="0")
import cephalo4 as C, xlpod4; xlpod4.install()
import sim4 as S4, body4h as B4
for i in (2,6): B4.HAND[i]=dict(T=21.3,K=10.1); B4.BO[i]=0.092
C.M_ROBOT += 2*(M_HAND-C.M_POD); C.W_ROBOT=C.M_ROBOT*C.G
sys.argv=["quad_video_r.py"]+sys.argv[1:]
runpy.run_path("quad_video_r.py", run_name="__main__")
