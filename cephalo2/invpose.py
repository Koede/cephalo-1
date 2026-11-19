import os, numpy as np
os.environ.setdefault("CEPH_PROP","xl"); os.environ.setdefault("CEPH_PACKS","2")
import cephalo4 as C, body4 as B
RX=np.diag([1.,-1,-1])
def pose(H, rf=0.36, q0=None):
    bp=np.array([0,0,H]); Q=[]
    for i in range(8):
        a=-np.radians(22.5+45*i)
        tgt=np.array([rf*np.cos(a), rf*np.sin(a), 0.0])
        q=np.radians([5,0,-60,0,-45,0,10,0]) if q0 is None else q0[i].copy()
        for _ in range(15): q=B.ik(i,tgt,B.DOWN,q,np.radians([5,0,-60,0,-45,0,10,0]),bp,RX)
        f,ax=B.foot(i,q,bp,RX); Q.append((q,np.linalg.norm(f-tgt),np.degrees(np.arccos(np.clip(-ax[2],-1,1)))))
    return Q
def load(i,q,H,Fz):
    bp=np.array([0,0,H]); f,_=B.foot(i,q,bp,RX)
    return np.abs(C.joint_torques(i,q,bp,RX,np.array([0,0,Fz]),f))/C.STALL
W=(C.M_ROBOT+0.6+0.81)*C.G
print("W",round(W,1))
for H in (0.17,0.20,0.24,0.28,0.32):
    for rf in (0.30,0.36):
        Q=pose(H,rf)
        q=Q[0][0]; segs,tip=C.arm_fk(0,q,np.array([0,0,H]),RX)
        P=np.array([C.at_s(segs,s)[0] for s in np.linspace(0.07,C.SPIGOT,30)])
        clr=C.body_clearance((P-[0,0,H])@RX,np.full(30,0.024),np.zeros(3)).min()
        print(f"H {H} rf {rf}: err {Q[0][1]*1000:.1f} mm ax {Q[0][2]:.1f} | q {np.degrees(q).round(0)} | clr {clr*1000:.0f} | load 8 feet {load(0,q,H,W/8).max():.2f}  7 feet {load(0,q,H,W/7).max():.2f} | dome gap {(H-0.156)*1000:.0f}")
