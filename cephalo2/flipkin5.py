import numpy as np
from flipkin3 import *
R_PROP=0.0953
def path3(N=120):
    q=np.tile(B.Q_FLY,(8,1)); Q=[q.copy()]; out=[]; gaps=[]
    for n in range(1,N+1):
        s=n/N; th=np.pi*B.mj(s); R=Rax(th); u=th/np.pi
        qn=B.Q_FLY+(B.Q_STAND-B.Q_FLY)*u
        for i in FLY:
            if i in (0,4):
                x0,r0,f0=cyl(H0[i]); x1,r1,f1=cyl(H1[i])
                tgt=fromcyl(x0+(x1-x0)*s,r0+(r1-r0)*s,f0+(f1-f0)*s); seed=q[i]; nom=B.Q_FLY+(B.Q_STAND-B.Q_FLY)*B.mj(s); wreg=0.05
            else:
                tgt=hub(i,qn,c,R)[0]; seed=qn; nom=qn; wreg=0.05
            qq=seed.copy()
            for _ in range(12): qq=B.ik(i,tgt,Z,qq,nom,c,R,lead=C.BLADE_OFF,w_ax=6.0,w_reg=wreg)
            dq=np.abs(qq-q[i]).max(); q[i]=qq; h,a=hub(i,q[i],c,R)
            out.append((n,i,np.linalg.norm(h-tgt),np.degrees(np.arccos(np.clip(a[2],-1,1))),clear(i,q[i],c,R),dq))
        for i in UNDER:
            q[i]=qn; out.append((n,i,0,0,clear(i,q[i],c,R),0))
        H=[hub(i,q[i],c,R)[0] for i in FLY]
        g=min(np.hypot(*(H[a][:2]-H[b][:2]))-2*R_PROP for a in range(5) for b in range(a+1,5) if abs(H[a][2]-H[b][2])<0.08)
        gaps.append(g)
        Q.append(q.copy())
    return np.array(out),np.array(Q),np.array(gaps)
res,Q,g=path3()
for i in range(8):
    r=res[res[:,1]==i]
    print(f"arm {i}: pos {r[:,2].max()*1000:.0f} mm axis {r[:,3].max():.1f} deg clr {r[:,4].min()*1000:.0f} mm maxstep {np.degrees(r[:,5].max()):.1f} | end-to-stand {np.degrees(np.abs(Q[-1,i]-B.Q_STAND).max()):.0f}")
print("min prop disc gap between flyers", round(g.min()*1000), "mm at step", g.argmin())
# arm-arm clearance (capsules) during the path
def arm_pts(i,q,R):
    segs,tip=C.arm_fk(i,q,c,R); return np.array([C.at_s(segs,s)[0] for s in np.linspace(0.07,C.SPIGOT,25)])
mind=9
for n in range(0,len(Q),4):
    s=n/(len(Q)-1); R=Rax(np.pi*B.mj(s))
    P=[arm_pts(i,Q[n,i],R) for i in range(8)]
    for a in range(8):
        b=(a+1)%8
        d=np.min(np.linalg.norm(P[a][:,None]-P[b][None],axis=2))-0.05
        if d<mind: mind=d; at=(n,a,b)
print("min neighbour arm clearance", round(mind*1000),"mm",at)
np.save("flip_path.npy",Q)
