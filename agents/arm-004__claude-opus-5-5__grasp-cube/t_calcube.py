import sys, json, math; sys.path.insert(0,'work'); sys.path.insert(0,'tools'); import arm, grasp, vision, cv2, time
from camd_client import read_frame
SX=0.0388
targets=[(a,r) for a,r in [(0,0.17),(35,0.20),(-35,0.20),(60,0.16),(-60,0.16),(0,0.23),(20,0.14),(-20,0.14)]]
import os
out=json.load(open('tools/cube_cal.json')) if os.path.exists('tools/cube_cal.json') else []
roll=40.2
with arm.bus() as b:
    if arm.joints(b)['gripper']>4:
        roll=None
    else:
      p=arm.tool_xyz(b)
      if p[2]<0.07: arm.goto(b,p[0],p[1],0.09,speed=30,correct=0)
      arm.home(b); time.sleep(0.5)
      t,_=read_frame('top'); c=vision.top_cube(t)[0]; m=vision.pix_to_base([(c['cx'],c['cy'])])[0]
      g=grasp.grasp(b,m[0],m[1],vision.top_cube_yaw(t,c)); print('initial',g,flush=True)
      if not g['ok']: sys.exit(1)
    roll=arm.joints(b)['wrist_roll']
    for a,r in targets:
        cx,cy=SX+r*math.cos(math.radians(a)), r*math.sin(math.radians(a))
        grasp.place(b,cx,cy,roll)
        arm.home(b); time.sleep(0.5)
        t,_=read_frame('top'); c=vision.top_cube(t)
        if not c: print('no cube seen'); break
        c=c[0]; out.append(dict(target=[cx,cy],px=[c['cx'],c['cy']])); print(a,r,(round(cx,3),round(cy,3)),c['cx'],c['cy'],flush=True)
        json.dump(out,open('tools/cube_cal.json','w'),indent=1)
        m=vision.pix_to_base([(c['cx'],c['cy'])])[0]; yaw=vision.top_cube_yaw(t,c)
        g=grasp.grasp(b,m[0],m[1],yaw); print(g,flush=True)
        if not g['ok']: break
        roll=g['roll']
