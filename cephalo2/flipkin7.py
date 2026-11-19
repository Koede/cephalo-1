import numpy as np
from flipkin6 import *
import flipkin6 as K
Q,*_=K.path4(wax=30,wreg=1.0,nomprev=False)
Q=np.array(Q); N=len(Q)-1
worst={}
for n in range(0,N+1,2):
    s=n/N; R=Rax(np.pi*B.mj(s))
    P=[arm_pts(i,Q[n,i],R) for i in range(8)]
    H={i:hub(i,Q[n,i],c,R)[0] for i in FLY}
    for a in range(8):
        for b in range(a+1,8):
            d=np.min(np.linalg.norm(P[a][:,None]-P[b][None],axis=2))-0.048
            if d<worst.get((a,b),(9,))[0]: worst[(a,b)]=(d,n)
    for a in FLY:
        for b in FLY:
            if a<b and abs(H[a][2]-H[b][2])<0.08:
                g=np.hypot(*(H[a][:2]-H[b][:2]))-2*R_PROP
                if g<worst.get(("disc",a,b),(9,))[0]: worst[("disc",a,b)]=(g,n)
for k,v in sorted(worst.items(),key=lambda kv:kv[1][0])[:8]: print(k, round(v[0]*1000), "mm at step", v[1])
