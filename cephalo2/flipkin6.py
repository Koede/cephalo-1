import numpy as np, itertools
from flipkin3 import *
R_PROP=0.0953
def arm_pts(i,q,R):
    segs,tip=C.arm_fk(i,q,c,R); return np.array([C.at_s(segs,s)[0] for s in np.linspace(0.07,C.SPIGOT,25)])
def path4(N=120,wax=30,wreg=0.3,nomprev=True):
    q=np.tile(B.Q_FLY,(8,1)); Q=[q.copy()]; ax=[];steps=[];gaps=[];ncl=[]
    for n in range(1,N+1):
        s=n/N; th=np.pi*B.mj(s); R=Rax(th); u=th/np.pi
        qn=B.Q_FLY+(B.Q_STAND-B.Q_FLY)*u
        for i in FLY:
            if i in (0,4):
                x0,r0,f0=cyl(H0[i]); x1,r1,f1=cyl(H1[i])
                tgt=fromcyl(x0+(x1-x0)*s,r0+(r1-r0)*s,f0+(f1-f0)*s); nom=B.Q_FLY+(B.Q_STAND-B.Q_FLY)*B.mj(s)
                qq=q[i].copy()
                for _ in range(12): qq=B.ik(i,tgt,Z,qq,nom,c,R,lead=C.BLADE_OFF,w_ax=6.0,w_reg=0.05)
            elif i==2:
                qq=qn.copy()
            else:
                tgt=hub(i,qn,c,R)[0]; nom=(q[i]*0.5+qn*0.5) if nomprev else qn
                qq=q[i].copy()
                for _ in range(12): qq=B.ik(i,tgt,Z,qq,nom,c,R,lead=C.BLADE_OFF,w_ax=wax,w_reg=wreg)
            steps.append(np.abs(qq-q[i]).max()); q[i]=qq; ax.append(np.degrees(np.arccos(np.clip(hub(i,qq,c,R)[1][2],-1,1))))
        for i in UNDER: q[i]=qn
        H=[hub(i,q[i],c,R)[0] for i in FLY]
        gaps.append(min(np.hypot(*(H[a][:2]-H[b][:2]))-2*R_PROP for a in range(5) for b in range(a+1,5) if abs(H[a][2]-H[b][2])<0.08) if True else 0)
        P=[arm_pts(i,q[i],R) for i in range(8)]
        ncl.append(min(np.min(np.linalg.norm(P[a][:,None]-P[(a+1)%8][None],axis=2)) for a in range(8))-0.048)
        Q.append(q.copy())
    return np.array(Q),max(ax),np.degrees(max(steps)),min(gaps),min(ncl),np.degrees(np.abs(np.array(Q)[-1,[1,3]]-B.Q_STAND).max())
for wax,wreg,nomprev in itertools.product((10,30,100),(0.1,0.3,1.0),(True,False)):
    Q,a,st,g,nc,e=path4(wax=wax,wreg=wreg,nomprev=nomprev)
    print(f"wax {wax} wreg {wreg} nomprev {nomprev}: axis {a:.1f} step {st:.1f} gap {g*1000:.0f} armclr {nc*1000:.0f} end-dev {e:.0f}")
