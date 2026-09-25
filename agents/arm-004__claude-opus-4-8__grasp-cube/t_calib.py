import sys, math, json; sys.path.insert(0,'work'); sys.path.insert(0,'tools'); import arm, time, numpy as np
from camd_client import read_jpeg
SX=0.0388
def pitch(r): return float(np.clip((r-0.26)/(0.43-0.26)*68,0,70))
pts=[]
for a in [-50,-25,0,25,50]:
    for r in [0.14,0.20,0.26,0.32,0.38]:
        pts.append((a,r))
out=[]
with arm.bus() as b:
    arm.gripper(b,0)
    for i,(a,r) in enumerate(pts):
        x=SX+r*math.cos(math.radians(a)); y=r*math.sin(math.radians(a))
        try:
            arm.goto(b,x,y,0.06,pitch=pitch(r),speed=50,correct=0)
            arm.goto(b,x,y,0.015,pitch=pitch(r),speed=30,correct=2)
        except ValueError as e:
            print('skip',a,r,e); continue
        time.sleep(0.4)
        p=arm.tool_xyz(b); open(f'tools/cal{i:02d}.jpg','wb').write(read_jpeg('top')[0])
        out.append(dict(i=i,a=a,r=r,xyz=p.tolist())); print(i,a,r,p.round(3),flush=True)
        arm.goto(b,x,y,0.06,pitch=pitch(r),speed=50,correct=0)
    arm.goto(b,0.2,0.0,0.12,speed=40,correct=0); arm.home(b)
json.dump(out,open('tools/cal_pts.json','w'),indent=1)
