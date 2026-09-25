import sys; sys.path.insert(0,'work'); sys.path.insert(0,'tools'); import arm, time, cv2, numpy as np
from camd_client import read_frame
x,y,roll=map(float,sys.argv[1:4])
ims=[]
with arm.bus() as b:
    arm.gripper(b,70)
    for z in [0.044,0.037,0.030,0.024,0.018,0.013]:
        arm.goto(b,x,y,z,roll=roll,speed=20,correct=2); time.sleep(0.3)
        w,_=read_frame('wrist'); p=arm.tool_xyz(b); print(z,p.round(4))
        cv2.putText(w,f'{z:.3f}',(20,80),0,2.5,(0,0,255),4); ims.append(cv2.resize(w,(640,360)))
    t,_=read_frame('top'); cv2.imwrite('tools/top.jpg',t)
    arm.goto(b,x,y,0.08,roll=roll,speed=30,correct=0)
cv2.imwrite('tools/desc.jpg',np.vstack([np.hstack(ims[0:2]),np.hstack(ims[2:4]),np.hstack(ims[4:6])]))
