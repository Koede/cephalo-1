"""Kinematic feasibility of the gimballed roll and inverted stand (python3 flipkin.py)."""
import os, numpy as np, json
os.environ.setdefault("CEPH_PROP","xl"); os.environ.setdefault("CEPH_PACKS","2")
import cephalo4 as C, body4 as B
D=C.DEG
def Rx(a):
    c,s=np.cos(a),np.sin(a); return np.array([[1,0,0],[0,c,-s],[0,s,c]])
def hub(i,q,bp,bR):
    cs,ax,R,p=B.fk(i,q,bp,bR); return p+R[:,0]*C.BLADE_OFF, R[:,0]
def clear(i,q,bp,bR):
    segs,tip=C.arm_fk(i,q,bp,bR); P=np.array([C.at_s(segs,s)[0] for s in np.linspace(0.07,C.SPIGOT,30)])
    return C.body_clearance((P-bp)@bR,np.full(30,0.024),np.zeros(3)).min()
c=np.zeros(3)
H0=np.array([hub(i,B.Q_FLY,c,np.eye(3))[0] for i in range(8)])
H1=np.array([hub(i,B.Q_STAND,c,Rx(np.pi))[0] for i in range(8)])
print("start hubs z",H0[:,2].round(3)," end hubs z",H1[:,2].round(3))
print("end axis z", [round(hub(i,B.Q_STAND,c,Rx(np.pi))[1][2],2) for i in range(8)])
FLY=[0,1,2,3,4,7]
def cyl(h): return h[0], np.hypot(h[1],h[2]), np.arctan2(h[2],h[1])
worst={}
q=np.tile(B.Q_FLY,(8,1))
N=60
res=[]
for n in range(N+1):
    s=n/N; th=np.pi*B.mj(s); R=Rx(th)
    for i in range(8):
        if i in FLY:
            x0,r0,f0=cyl(H0[i]); x1,r1,f1=cyl(H1[i])
            if i in (4,7):  # phi decreasing through 90
                pass
            f=f0+(f1-f0)*s; r=r0+(r1-r0)*s; x=x0+(x1-x0)*s
            r+= 0.0
            tgt=np.array([x,r*np.cos(f),r*np.sin(f)])
            for _ in range(4):
                q[i]=B.ik(i,tgt,np.array([0,0,1.]),q[i],q[i],c,R,lead=C.BLADE_OFF,w_ax=2.0,w_reg=0.01)
            h,a=hub(i,q[i],c,R)
            res.append((n,i,np.linalg.norm(h-tgt),np.degrees(np.arccos(np.clip(a[2],-1,1))),clear(i,q[i],c,R)))
        else:
            q[i]=B.Q_FLY+(B.Q_STAND-B.Q_FLY)*s
            res.append((n,i,0,0,clear(i,q[i],c,R)))
res=np.array(res)
for i in range(8):
    r=res[res[:,1]==i]
    print(f"arm {i}: pos err max {r[:,2].max()*1000:.0f} mm, axis err max {r[:,3].max():.1f} deg, body clearance min {r[:,4].min()*1000:.0f} mm")
print("end q flyers vs Q_STAND (deg):"); 
for i in FLY: print(i,np.degrees(q[i]).round(0), "lim hit", np.any(np.abs(q[i]-C.LIM[:,0])<1e-3)|np.any(np.abs(q[i]-C.LIM[:,1])<1e-3))
