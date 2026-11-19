import numpy as np, flip4 as F4
F=F4.Flip(); d=F.d
orig=F.flip_thrust
def ft(t):
    orig(t)
    if abs(t-2.2)<2e-4:
        import inspect
        cz=np.array([d.xmat[F.pod[i]].reshape(3,3)[2,0] for i in range(8)])
        wgt=F.gate_s.copy()
        on=[i for i in range(8)]
        com=d.subtree_com[F.torso]
        A=np.zeros((3,8))
        for j,i in enumerate(on):
            ax=d.xmat[F.pod[i]].reshape(3,3)[:,0]; r=d.xpos[F.pod[i]]+ax*F4.C.BLADE_OFF-com; mo=np.cross(r,ax); A[:,j]=[ax[2],mo[0],mo[1]]
        b=np.array([113.,0,0])
        print("w",wgt.round(3)); print("F",F.allocate_n(A,b,wgt,F.fmax_eff*wgt).round(2))
        print("F w=1 for 5",F.allocate_n(A[:,:5],b,np.ones(5),np.full(5,F.fmax_eff)).round(2), F.fmax_eff)
F.flip_thrust=ft
F4.T_END=2.21
F.run_flip()
