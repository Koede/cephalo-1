import numpy as np, sys
from flipkin3 import *
def path2(N=120,wreg=0.05,wax=6.0,wpos=1.0):
    q=np.tile(B.Q_FLY,(8,1)); Q=[q.copy()]; out=[]
    for n in range(1,N+1):
        s=n/N; th=np.pi*B.mj(s); R=Rax(th); qn=B.Q_FLY+(B.Q_STAND-B.Q_FLY)*B.mj(s)
        for i in FLY:
            x0,r0,f0=cyl(H0[i]); x1,r1,f1=cyl(H1[i])
            tgt=fromcyl(x0+(x1-x0)*s,r0+(r1-r0)*s,f0+(f1-f0)*s)
            cands=[q[i]]+[twist(B.Q_FLY,sg*th) for sg in (1,-1)]+[qn]
            best=None
            for q0 in cands:
                qq=np.clip(q0,C.LIM[:,0],C.LIM[:,1])
                for _ in range(12): qq=B.ik(i,tgt,Z,qq,qn,c,R,lead=C.BLADE_OFF,w_ax=wax,w_reg=wreg)
                h,a=hub(i,qq,c,R); e=wpos*np.linalg.norm(h-tgt)+0.3*(1-a[2])+0.05*np.abs(qq-q[i]).max()
                if clear(i,qq,c,R)<0.008: e+=1
                if best is None or e<best[0]: best=(e,qq)
            dq=np.abs(best[1]-q[i]).max(); q[i]=best[1]; h,a=hub(i,q[i],c,R)
            out.append((n,i,np.linalg.norm(h-tgt),np.degrees(np.arccos(np.clip(a[2],-1,1))),clear(i,q[i],c,R),dq))
        for i in UNDER:
            q[i]=qn; out.append((n,i,0,0,clear(i,q[i],c,R),0))
        Q.append(q.copy())
    return np.array(out),np.array(Q)
for wreg,wax in ((0.05,6),(0.1,6),(0.2,6)):
    res,Q=path2(wreg=wreg,wax=wax)
    print("wreg",wreg,"wax",wax)
    for i in FLY:
        r=res[res[:,1]==i]
        print(f"  arm {i}: pos {r[:,2].max()*1000:.0f} mm axis {r[:,3].max():.1f} deg clr {r[:,4].min()*1000:.0f} mm maxstep {np.degrees(r[:,5].max()):.1f} | end-to-stand {np.degrees(np.abs(Q[-1,i]-B.Q_STAND).max()):.0f}")
    np.save(f"flip_path_{wreg}.npy",Q)
