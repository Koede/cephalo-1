import os, numpy as np
os.environ.setdefault("CEPH_PROP","xl"); os.environ.setdefault("CEPH_PACKS","2")
import cephalo4 as C, body4 as B
from flipkin import Rx, hub, clear, cyl, H0, H1
c=np.zeros(3)
FLY=[0,1,2,3,4,7]
def run(wreg, N=90, seeds=True):
    q=np.tile(B.Q_FLY,(8,1)); out=[]
    for n in range(N+1):
        s=n/N; th=np.pi*B.mj(s); R=Rx(th); qn=B.Q_FLY+(B.Q_STAND-B.Q_FLY)*B.mj(s)
        for i in FLY:
            x0,r0,f0=cyl(H0[i]); x1,r1,f1=cyl(H1[i])
            f=f0+(f1-f0)*s; r=r0+(r1-r0)*s; x=x0+(x1-x0)*s
            tgt=np.array([x,r*np.cos(f),r*np.sin(f)])
            cands=[q[i].copy()]
            if seeds: cands += [q[i]+np.random.default_rng(k).normal(0,0.3,8) for k in range(6)]+[qn.copy()]
            best=None
            for q0 in cands:
                qq=np.clip(q0,C.LIM[:,0],C.LIM[:,1])
                for _ in range(10): qq=B.ik(i,tgt,np.array([0,0,1.]),qq,qn,c,R,lead=C.BLADE_OFF,w_ax=2.0,w_reg=wreg)
                h,a=hub(i,qq,c,R); e=np.linalg.norm(h-tgt)+0.1*(1-a[2])+0.02*np.abs(qq-q[i]).max()
                if clear(i,qq,c,R)<0.005: e+=1
                if best is None or e<best[0]: best=(e,qq)
            q[i]=best[1]; h,a=hub(i,q[i],c,R)
            out.append((n,i,np.linalg.norm(h-tgt),np.degrees(np.arccos(np.clip(a[2],-1,1))),clear(i,q[i],c,R),np.abs(q[i]-qprev[i]).max() if n else 0))
        qprev=q.copy() if n==0 else qprev
        qprev=q.copy()
    return np.array(out),q
res,q=run(0.05)
for i in FLY:
    r=res[res[:,1]==i]
    print(f"arm {i}: pos {r[:,2].max()*1000:.0f} mm axis {r[:,3].max():.1f} deg clr {r[:,4].min()*1000:.0f} mm  maxstep {np.degrees(r[:,5].max()):.0f} deg  end-dq-to-stand {np.degrees(np.abs(q[i]-B.Q_STAND).max()):.0f}")
