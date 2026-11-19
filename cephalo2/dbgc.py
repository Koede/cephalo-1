import numpy as np, fall4 as FA, mujoco, states4 as ST, cephalo4 as C
F=FA.Fall("hit"); m,d=F.m,F.d
best=[9,None]
orig=F.fall_thrust
def ft(t):
    orig(t)
    if 1.7<t<2.2:
        n=0
        for j in range(d.ncon):
            c=d.contact[j]; b1=m.body(m.geom_bodyid[c.geom1]).name; b2=m.body(m.geom_bodyid[c.geom2]).name
            if b1[0] in "abcp" and b2[0] in "abcp" and c.geom1!=m.geom('floor').id and c.geom2!=m.geom('floor').id: n+=1
        if n: print(round(t,3),"arm-arm contacts",n)
F.fall_thrust=ft
F.run(2.2)
# check actual geom distances at worst frame via mj_geomDistance? use check at t=2.2
p,R,q,cl,_=ST.state(d.qpos)
