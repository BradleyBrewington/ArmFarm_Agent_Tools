import sys; sys.path.insert(0,'work'); sys.path.insert(0,'tools'); import arm, time, cv2, numpy as np
from camd_client import read_frame
x,y,roll=0.17,0.03,90.0
ims=[]
with arm.bus() as b:
    arm.gripper(b,85)
    arm.goto(b,x,y,0.03,roll=roll,speed=40,correct=2)
    for zc in [0.006,-0.002,-0.010,-0.014,-0.018,-0.022,-0.026,-0.030]:
        arm.goto(b,x,y,zc,roll=roll,speed=8,correct=0); time.sleep(0.6)
        w,_=read_frame('wrist'); z=float(arm.tool_xyz(b)[2])
        crop=w[150:550,300:750].copy(); cv2.putText(crop,f'{zc:.3f}/{z:.3f}',(5,40),0,1.1,(0,0,255),3)
        ims.append(cv2.resize(crop,(338,300))); print(zc, round(z,4), flush=True)
    arm.goto(b,x,y,0.05,roll=roll,speed=30,correct=0)
cv2.imwrite('tools/shadow.jpg',np.vstack([np.hstack(ims[0:4]),np.hstack(ims[4:8])]))
