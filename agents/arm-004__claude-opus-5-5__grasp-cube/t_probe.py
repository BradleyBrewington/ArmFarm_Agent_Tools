import sys, json; sys.path.insert(0,'work'); sys.path.insert(0,'tools'); import arm, vision, time, numpy as np, cv2
import run_episodes as R
with arm.bus() as b:
    R.lift_clear(b); arm.home(b); time.sleep(0.4)
    f=R.find_cube(); cx,cy=(float(sys.argv[1]),float(sys.argv[2])) if len(sys.argv)>2 else (f[0],f[1]); print('map',cx,cy,'px',f[3]['cx'],f[3]['cy'],flush=True)
    arm.gripper(b,0)
    res=[]
    for dx in np.arange(-0.03,0.0301,0.015):
        for dy in np.arange(-0.03,0.0301,0.015):
            x,y=cx+dx,cy+dy
            arm.goto(b,x,y,0.06,roll=83.4,speed=50,correct=1)
            arm.goto(b,x,y,-0.015,roll=83.4,speed=15,correct=0); time.sleep(0.3)
            z=float(arm.tool_xyz(b)[2]); hit=z>-0.005
            res.append((float(dx),float(dy),z,hit)); print(round(dx,3),round(dy,3),round(z,4),'HIT' if hit else '',flush=True)
            arm.goto(b,x,y,0.06,roll=83.4,speed=40,correct=0)
    json.dump({'map':[cx,cy],'px':[f[3]['cx'],f[3]['cy']],'res':res},open('tools/probe.json','w'))
    arm.home(b)
