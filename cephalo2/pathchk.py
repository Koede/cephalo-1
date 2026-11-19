import numpy as np, sys, flip4 as F4, cephalo4 as C
Q=np.load(sys.argv[1]); N=len(Q)-1; c=np.zeros(3)
dq=np.degrees(np.abs(np.diff(Q,axis=0)).max(axis=2))
print("steps>8deg:", [(n+1, i, round(dq[n,i])) for n in range(N) for i in range(8) if dq[n,i]>8][:20])
def pts(i,q,R):
    segs,tip=C.arm_fk(i,q,c,R); return np.array([C.at_s(segs,s)[0] for s in np.linspace(0.07,C.SPIGOT,20)])
worst=[9,9,9]
for n in range(0,N+1,2):
    R=F4.Rax(np.pi*F4.mj(n/N)); P=[pts(i,Q[n,i],R) for i in range(8)]
    for a in range(8):
        worst[0]=min(worst[0],C.body_clearance((P[a]-c)@R,np.full(20,0.024),np.zeros(3)).min())
        for b in range(a+1,8): worst[1]=min(worst[1],np.min(np.linalg.norm(P[a][:,None]-P[b][None],axis=2))-0.048)
    H=[F4.hub(i,Q[n,i],c,R)[0] for i in F4.FLYERS]
    for a in range(5):
        for b in range(a+1,5):
            if abs(H[a][2]-H[b][2])<0.1: worst[2]=min(worst[2],np.hypot(*(H[a][:2]-H[b][:2]))-2*0.0953)
print("body clr %.0f mm, arm-arm %.0f mm, disc gap %.0f mm"%tuple(np.array(worst)*1000))
for n in (0,40,80,120,160): print(n, np.degrees(Q[n,:5]).round(0).tolist())
