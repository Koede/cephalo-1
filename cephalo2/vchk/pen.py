import os, sys, numpy as np, json
os.environ.setdefault("CEPH_PROP","xl"); os.environ.setdefault("CEPH_PACKS","1"); os.environ.setdefault("CEPH_J1","STS3250")
os.environ["CEPH_J1CLUTCH"]="0.9"
import cephalo4 as C, power4 as PW
C.CLAW_R = PW.PROPS["xl"]["R"]
import viz4 as V, states4 as ST
print("states4 model nq", ST._M.nq)
for c in ['cut','hit','inv','drift']:
    z=np.load(f'fall4_{c}.npz'); t=z['t']
    assert z['qpos'].shape[1]==ST._M.nq, (z['qpos'].shape, ST._M.nq)
    zz=[];pp=[];who=[]
    for j in range(0,len(t),3):
        st=ST.state(z['qpos'][j]); fan=z['fan'][j]; fold=np.where(fan>1,0.0,np.radians(60))
        S=V.scene_from_state(st,t[j],np.zeros(8),fold,fan,z['edf'][j])
        zs=[float(np.asarray(p[0])[:,2].min()) for p in S.prims]
        zz.append(min(zs)); pp.append(str(z['phase'][j])); who.append(int(np.argmin(zs)))
    zz=np.array(zz); pp=np.array(pp)
    print('==',c,'lowest drawn point %.0f mm; frames below -10 mm: %d of %d; below -30: %d'%(zz.min()*1000,(zz<-0.010).sum(),len(zz),(zz<-0.030).sum()))
    for ph in dict.fromkeys(pp): print('     %-8s min %.0f mm'%(ph,zz[pp==ph].min()*1000))
