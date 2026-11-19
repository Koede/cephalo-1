import os, sys, runpy, numpy as np
sys.argv=["quad_h3.py","low","quad4_g.json"]
os.environ["QUAD_FILE"]="qclear_stub.py"
open("qclear_stub.py","w").write("""
import sys, os, numpy as np, runpy
ns = runpy.run_path('quad4r.py', run_name='notmain')
import body4h as B4, itertools
Q = ns['Quad'](); d = Q.d; FLY = ns['FLY']
Rd = [0.127 if i in B4.HAND else __import__('cephalo4').CLAW_R for i in range(8)]
def disc(i, n=72):
    R = d.xmat[Q.pod[i]].reshape(3,3); c = d.xpos[Q.pod[i]] + R[:,0]*B4.BO[i]
    a = np.linspace(0, 2*np.pi, n, endpoint=False)
    return c[None] + Rd[i]*(np.cos(a)[:,None]*R[:,1] + np.sin(a)[:,None]*R[:,2]), c
worst = (9, None)
for i, j in itertools.combinations(FLY, 2):
    pi, ci = disc(i); pj, cj = disc(j)
    dm = np.linalg.norm(pi[:,None]-pj[None], axis=2).min()
    if dm < worst[0]: worst = (dm, (i, j, np.linalg.norm(ci-cj), Rd[i]+Rd[j]))
print('TASK', ns['TASK'], 'POSE', ns['POSE'], 'min rim-rim %.3f m' % worst[0], worst[1])
""")
runpy.run_path("quad_h3.py", run_name="__main__")
