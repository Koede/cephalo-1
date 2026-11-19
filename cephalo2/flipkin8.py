import numpy as np, json
import flipkin3 as K3
from flipkin3 import B, C, Rax, hub, cyl, fromcyl, H0, H1, Z, c, clear, U
R_PROP=0.0953
FLY=[0,1,2,3,4]; UNDER=[5,6,7]
def arm_pts(i,q,R):
    segs,tip=C.arm_fk(i,q,c,R); return np.array([C.at_s(segs,s)[0] for s in np.linspace(0.07,C.SPIGOT,25)])
def path(N=120,bump=0.06,lift=0.0,wreg=1.0):
    q=np.tile(B.Q_FLY,(8,1)); Q=[q.copy()]; ax=[];steps=[]
    for n in range(1,N+1):
        s=n/N; th=np.pi*B.mj(s); R=Rax(th); u=th/np.pi
        qn=B.Q_FLY+(B.Q_STAND-B.Q_FLY)*u
        for i in FLY:
            if i in (0,4):
                x0,r0,f0=cyl(H0[i]); x1,r1,f1=cyl(H1[i])
                sg=np.sign(x0); b=np.sin(np.pi*s)
                tgt=fromcyl(x0+(x1-x0)*s+sg*bump*b,r0+(r1-r0)*s+lift*b,f0+(f1-f0)*s); nom=B.Q_FLY+(B.Q_STAND-B.Q_FLY)*B.mj(s)
                qq=q[i].copy()
                for _ in range(12): qq=B.ik(i,tgt,Z,qq,nom,c,R,lead=C.BLADE_OFF,w_ax=6.0,w_reg=0.05)
            elif i==2: qq=qn.copy()
            else:
                tgt=hub(i,qn,c,R)[0]; qq=q[i].copy()
                for _ in range(12): qq=B.ik(i,tgt,Z,qq,qn,c,R,lead=C.BLADE_OFF,w_ax=30,w_reg=wreg)
            steps.append(np.abs(qq-q[i]).max()); q[i]=qq; ax.append(np.degrees(np.arccos(np.clip(hub(i,qq,c,R)[1][2],-1,1))))
        for i in UNDER: q[i]=qn
        Q.append(q.copy())
    Q=np.array(Q); arm=9; disc=9; bodyc=9
    for n in range(0,N+1,2):
        s=n/N; R=Rax(np.pi*B.mj(s))
        P=[arm_pts(i,Q[n,i],R) for i in range(8)]
        for a in range(8):
            bodyc=min(bodyc,clear(a,Q[n,a],c,R))
            for b2 in range(a+1,8): arm=min(arm,np.min(np.linalg.norm(P[a][:,None]-P[b2][None],axis=2))-0.048)
        H={i:hub(i,Q[n,i],c,R)[0] for i in FLY}
        for a in FLY:
            for b2 in FLY:
                if a<b2 and abs(H[a][2]-H[b2][2])<0.08: disc=min(disc,np.hypot(*(H[a][:2]-H[b2][:2]))-2*R_PROP)
    return Q,max(ax),np.degrees(max(steps)),arm,disc,bodyc
if __name__=="__main__":
    for bump in (0.0,0.05,0.10,0.15):
        for lift in (0.0,0.08):
            Q,a,st,arm,disc,bc=path(bump=bump,lift=lift)
            print(f"bump {bump} lift {lift}: axis {a:.1f} step {st:.1f} armclr {arm*1000:.0f} disc {disc*1000:.0f} body {bc*1000:.0f}")
