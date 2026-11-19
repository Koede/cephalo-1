import numpy as np, sys
from scipy.optimize import lsq_linear
import flip4 as F4, cephalo4 as C
from flip_pathopt import active_w, arm_mass, M_PACK, W, c
Q=np.load(sys.argv[1]); N=len(Q)-1
for n in range(0,N+1,8):
    s=n/N; th=np.pi*F4.mj(s); R=F4.Rax(th); aw=active_w(th)
    on=[i for i in range(8) if (i in F4.FLYERS or aw>0.5)]
    mp=M_PACK*(R@np.array([0,0,0.03]))+C.M_BODY*(R@np.array([0,0,0.05])); mm=C.M_BODY+M_PACK
    for j in range(8):
        a_,b_=arm_mass(j,Q[n,j],R); mp=mp+a_; mm+=b_
    com=mp/mm; H=np.array([F4.hub(i,Q[n,i],c,R)[0] for i in on]); k=len(on)
    A=np.vstack([np.ones(k),(H[:,1]-com[1]),-(H[:,0]-com[0]), 0.05*np.eye(k)])
    b=np.concatenate([[W,0,0],0.05*np.full(k,W/k)])
    A[1:3]*=10; b[1:3]*=10
    Tm=lsq_linear(A,b,bounds=(0,60)).x
    L=max((np.abs(C.joint_torques(i,Q[n,i],c,R,(Tm[on.index(i)] if i in on else 0)*np.array([0,0,1.]),F4.hub(i,Q[n,i],c,R)[0]))/C.STALL).max() for i in range(8))
    res=A[:3]@Tm-b[:3]
    print(f"{s:5.2f} {np.degrees(th):6.1f} | load {L:.2f} | T {Tm.round(1)} | res {np.abs(res).round(2)}")
