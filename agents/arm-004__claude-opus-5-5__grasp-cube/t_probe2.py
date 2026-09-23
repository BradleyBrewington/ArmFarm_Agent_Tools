import sys, json, math; sys.path.insert(0,'work'); import arm, time, numpy as np
cx,cy=float(sys.argv[1]),float(sys.argv[2]); step=float(sys.argv[3]) if len(sys.argv)>3 else 0.02
res=[]
with arm.bus() as b:
    arm.gripper(b,0)
    for dx in np.arange(-2*step,2*step+1e-6,step):
        for dy in np.arange(-2*step,2*step+1e-6,step):
            x,y=cx+dx,cy+dy
            if math.hypot(x-0.0388,y)<0.115: continue
            try:
                arm.goto(b,x,y,0.05,roll=83.4,speed=45,correct=1)
                arm.goto(b,x,y,-0.035,roll=83.4,speed=12,correct=0); time.sleep(0.3)
            except ValueError as e:
                print('skip',x,y,e); continue
            z=float(arm.tool_xyz(b)[2]); hit=z>-0.025
            res.append((float(x),float(y),z,hit)); print(round(x,3),round(y,3),round(z,4),'HIT' if hit else '',flush=True)
            arm.goto(b,x,y,0.05,roll=83.4,speed=40,correct=0)
    arm.home(b)
json.dump(res,open('tools/probe2.json','w'))
h=[r for r in res if r[3]]; print('hits',len(h), np.mean([r[:2] for r in h],0) if h else None)
