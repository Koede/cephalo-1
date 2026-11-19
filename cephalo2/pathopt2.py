"""Load-aware roll path for the flyers: claw axis vertical (hard), servo load low, continuity, clearances."""
import numpy as np, time
from scipy.optimize import least_squares
import flip4 as F4, cephalo4 as C, body4 as B4
c=np.zeros(3); T=22.6; RP=0.0953
P0=F4.PATH; N=len(P0)-1
def arm_pts(i,q,R):
    segs,tip=C.arm_fk(i,q,c,R); return np.array([C.at_s(segs,s)[0] for s in np.linspace(0.07,C.SPIGOT,14)])
def resid(q,i,R,qprev,others,hubs_o,qnom=None,T=22.6):
    h,a=F4.hub(i,q,c,R)
    r=[8*(a-np.array([0,0,1.]))]
    tau=C.joint_torques(i,q,c,R,T*np.array([0,0,1.]),h)/C.STALL
    r.append(4.0*np.maximum(np.abs(tau)-0.6,0))
    r.append(1.0*(q-qprev)); r.append(0.4*(q-qnom))
    P=arm_pts(i,q,R)
    bc=C.body_clearance((P-c)@R,np.full(len(P),0.024),np.zeros(3))
    r.append(20*np.maximum(0.015-bc,0))
    for Po in others:
        dmin=np.min(np.linalg.norm(P[:,None]-Po[None],axis=2))-0.048
        r.append([20*max(0.03-dmin,0)])
    for ho in hubs_o:
        g=np.hypot(*(h[:2]-ho[:2]))-2*RP
        r.append([20*max(0.03-g,0)*(abs(h[2]-ho[2])<0.1)])
    return np.concatenate([np.atleast_1d(x) for x in r])
Q=P0.copy()
t0=time.time()
for n in range(1,N+1):
    s=n/N; R=F4.Rax(np.pi*F4.mj(s))
    for i in F4.FLYERS:
        others=[arm_pts(j,Q[n,j] if j<i or j in F4.UNDER else Q[n-1,j],R) for j in range(8) if j!=i]
        hubs_o=[F4.hub(j,Q[n,j] if j<i else Q[n-1,j],c,R)[0] for j in F4.FLYERS if j!=i]
        q0=Q[n-1,i]+(P0[n,i]-P0[n-1,i])
        th=np.pi*F4.mj(s); Tn=14.0+8.6*min(1,np.sin(th)/np.sin(np.radians(40)))
        sol=least_squares(resid,np.clip(q0,C.LIM[:,0]+1e-6,C.LIM[:,1]-1e-6),bounds=(C.LIM[:,0],C.LIM[:,1]),args=(i,R,Q[n-1,i],others,hubs_o,P0[n,i],Tn),max_nfev=80)
        Q[n,i]=sol.x
    if n%20==0: print(n, f"{time.time()-t0:.0f}s", flush=True)
np.save("flip_path_opt2.npy",Q)
# report
for n in range(0,N+1,16):
    s=n/N; R=F4.Rax(np.pi*F4.mj(s)); row=[]
    for i in F4.FLYERS:
        th=np.pi*F4.mj(s); Tn=14.0+8.6*min(1,np.sin(th)/np.sin(np.radians(40)))
        h,a=F4.hub(i,Q[n,i],c,R); l=np.abs(C.joint_torques(i,Q[n,i],c,R,Tn*np.array([0,0,1.]),h))/C.STALL
        row.append(f"{l.max():.2f}(j{l.argmax()}) ax{np.degrees(np.arccos(min(a[2],1))):.0f}")
    print(f"s {s:.2f}: "+"  ".join(row))
print("max step deg", np.degrees(np.abs(np.diff(Q,axis=0)).max()))
