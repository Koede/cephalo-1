import os, numpy as np
os.environ.setdefault("CEPH_PROP","xl"); os.environ.setdefault("CEPH_PACKS","2")
import cephalo4 as C, body4 as B
D=C.DEG
AX=np.radians(22.5)
def Rax(th):   # roll about the horizontal axis at azimuth 22.5 deg (through arms 0 and 4)
    u=np.array([np.cos(AX),np.sin(AX),0]); K=np.array([[0,-u[2],u[1]],[u[2],0,-u[0]],[-u[1],u[0],0]])
    return np.eye(3)+np.sin(th)*K+(1-np.cos(th))*K@K
def hub(i,q,bp,bR):
    cs,ax,R,p=B.fk(i,q,bp,bR); return p+R[:,0]*C.BLADE_OFF, R[:,0]
def clear(i,q,bp,bR):
    segs,tip=C.arm_fk(i,q,bp,bR); P=np.array([C.at_s(segs,s)[0] for s in np.linspace(0.07,C.SPIGOT,30)])
    return C.body_clearance((P-bp)@bR,np.full(30,0.024),np.zeros(3)).min()
U=np.array([np.cos(AX),np.sin(AX),0]); V=np.array([-np.sin(AX),np.cos(AX),0]); Z=np.array([0,0,1.])
def cyl(h): return h@U, np.hypot(h@V,h@Z), np.arctan2(h@Z,h@V)
def fromcyl(x,r,f): return x*U+r*np.cos(f)*V+r*np.sin(f)*Z
c=np.zeros(3)
H0=np.array([hub(i,B.Q_FLY,c,np.eye(3))[0] for i in range(8)])
H1=np.array([hub(i,B.Q_STAND,c,Rax(np.pi))[0] for i in range(8)])
FLY=[0,1,2,3,4]; UNDER=[5,6,7]
def twist(q,phi):
    o=q.copy()
    for k in (0,2,4,6):
        o[k]=q[k]*np.cos(phi); o[k+1]=q[k+1]+q[k]*np.sin(phi)
    return o
def path(N=90,wreg=0.02):
    q=np.tile(B.Q_FLY,(8,1)); Q=[q.copy()]; out=[]
    for n in range(1,N+1):
        s=n/N; th=np.pi*B.mj(s); R=Rax(th)
        for i in FLY:
            x0,r0,f0=cyl(H0[i]); x1,r1,f1=cyl(H1[i])
            if f1-f0>np.pi: f1-=2*np.pi
            if f0-f1>np.pi: f1+=2*np.pi
            tgt=fromcyl(x0+(x1-x0)*s,r0+(r1-r0)*s,f0+(f1-f0)*s)
            cands=[q[i]]+[twist(B.Q_FLY,sg*th) for sg in (1,-1)]+[B.Q_FLY+(B.Q_STAND-B.Q_FLY)*s]
            best=None
            for q0 in cands:
                qq=np.clip(q0,C.LIM[:,0],C.LIM[:,1])
                for _ in range(12): qq=B.ik(i,tgt,Z,qq,q[i],c,R,lead=C.BLADE_OFF,w_ax=2.0,w_reg=wreg)
                h,a=hub(i,qq,c,R); e=np.linalg.norm(h-tgt)+0.1*(1-a[2])+0.05*np.abs(qq-q[i]).max()
                if clear(i,qq,c,R)<0.008: e+=1
                if best is None or e<best[0]: best=(e,qq)
            dq=np.abs(best[1]-q[i]).max(); q[i]=best[1]; h,a=hub(i,q[i],c,R)
            out.append((n,i,np.linalg.norm(h-tgt),np.degrees(np.arccos(np.clip(a[2],-1,1))),clear(i,q[i],c,R),dq))
        for i in UNDER:
            q[i]=B.Q_FLY+(B.Q_STAND-B.Q_FLY)*B.mj(s); out.append((n,i,0,0,clear(i,q[i],c,R),0))
        Q.append(q.copy())
    return np.array(out),np.array(Q)
if __name__=="__main__":
    res,Q=path()
    for i in range(8):
        r=res[res[:,1]==i]
        print(f"arm {i}: pos {r[:,2].max()*1000:.0f} mm axis {r[:,3].max():.1f} deg clr {r[:,4].min()*1000:.0f} mm maxstep {np.degrees(r[:,5].max()):.1f} deg | end {np.degrees(Q[-1,i]).round(0)}")
    np.save("flip_path.npy",Q)
