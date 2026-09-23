import sys, json; sys.path.insert(0,'work'); sys.path.insert(0,'tools'); import arm, grasp, servo, time, cv2, numpy as np
from camd_client import read_frame
x,y,roll=map(float,sys.argv[1:4]); z=0.013
out=[]
with arm.bus() as b:
    arm.gripper(b,85)
    for dx,dy in [(0,0),(0.01,0),(0,0.01),(-0.01,0),(0,-0.01),(0,0)]:
        arm.goto(b,x+dx,y+dy,z,roll=roll,speed=30,correct=3); time.sleep(0.3)
        img,_=read_frame('wrist'); d=grasp.dip_cube(img); t=arm.tool_xyz(b)
        out.append((t.tolist(),d)); print(dx,dy,t.round(4),d,flush=True)
        cv2.imwrite(f'tools/jp_{len(out)}.jpg',img)
    A=servo._tool_A(arm.joints(b))
P=np.array([o[1][:2] for o in out]); T=np.array([o[0][:2] for o in out])
dT=T[1:5]-T[0]; dP=P[1:5]-P[0]
Jb,*_=np.linalg.lstsq(dT,dP,rcond=None); Jb=Jb.T
Jt=Jb@np.linalg.inv(A); print('J_base',Jb,'J_tool',Jt)
json.dump({'J_tool_pre':Jt.tolist(),'z':z},open('tools/jpre.json','w'))
