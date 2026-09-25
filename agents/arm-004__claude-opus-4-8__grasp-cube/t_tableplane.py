import sys, json, math; sys.path.insert(0,'work'); import arm, time, numpy as np
import run_episodes as R
SX=0.0388
f=R.find_cube(); cube=(f[0],f[1]) if f else (0.05,0.19)
pts=[]
with arm.bus() as b:
    arm.gripper(b,0)
    for a in (-30,0,30,60,85):
        for r in (0.13,0.18,0.22):
            x,y=SX+r*math.cos(math.radians(a)), r*math.sin(math.radians(a))
            if math.hypot(x-cube[0],y-cube[1])<0.07: continue
            arm.goto(b,x,y,0.04,roll=83.4,speed=45,correct=1)
            zc=0.02; last=None
            while zc>-0.05:
                arm.goto(b,x,y,zc,roll=83.4,speed=10,correct=0); time.sleep(0.25)
                zm=float(arm.tool_xyz(b)[2])
                if zm-zc>0.006: break
                last=zm; zc-=0.005
            pts.append((x,y,zm)); print(a,r,round(x,3),round(y,3),'contact z',round(zm,4),flush=True)
            arm.goto(b,x,y,0.05,roll=83.4,speed=30,correct=0)
    arm.home(b)
P=np.array(pts); A=np.c_[P[:,0],P[:,1],np.ones(len(P))]; c,*_=np.linalg.lstsq(A,P[:,2],rcond=None)
print('plane',c,'resid mm',np.round((A@c-P[:,2])*1000,1))
json.dump({'plane':c.tolist(),'pts':pts},open('tools/table_plane.json','w'),indent=1)
