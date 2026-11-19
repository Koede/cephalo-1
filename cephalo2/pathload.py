import numpy as np, flip4 as F4, cephalo4 as C, body4 as B4
P=F4.PATH; N=len(P)-1
c=np.zeros(3)
for n in range(0,N+1,10):
    s=n/N; R=F4.Rax(np.pi*F4.mj(s)); row=[]
    for i in range(8):
        h,a=F4.hub(i,P[n,i],c,R)
        T=20.0 if (i in F4.FLYERS) else 0.0
        tau=C.joint_torques(i,P[n,i],c,R,T*np.array([0,0,1.]),h)
        l=np.abs(tau)/C.STALL; row.append(f"{l.max():.2f}(j{l.argmax()})")
    print(f"s {s:.2f} th {np.degrees(np.pi*F4.mj(s)):5.1f}: "+" ".join(row))
