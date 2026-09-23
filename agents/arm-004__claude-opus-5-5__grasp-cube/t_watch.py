import sys; sys.path.insert(0,'work'); sys.path.insert(0,'tools'); import arm, time, cv2, numpy as np
from camd_client import read_frame
x,y,roll,z=map(float,sys.argv[1:5])
ims=[]
with arm.bus() as b:
    arm.gripper(b,85)
    arm.goto(b,x,y,0.06,roll=roll,speed=40,correct=1)
    arm.goto(b,x,y,z,roll=roll,speed=15,correct=2); time.sleep(0.3)
    for g in (85,60,40,25,12,0):
        arm.gripper(b,g,0.4); time.sleep(0.2)
        w,_=read_frame('wrist'); cv2.putText(w,str(g),(30,90),0,3,(0,0,255),5); ims.append(cv2.resize(w,(640,360)))
    t,_=read_frame('top'); ims[-1]=np.hstack([cv2.resize(ims[-1],(320,360)),cv2.resize(t,(320,360))])
    arm.goto(b,x,y,0.08,roll=roll,speed=30,correct=0)
cv2.imwrite('tools/watch.jpg',np.vstack([np.hstack(ims[0:2]),np.hstack(ims[2:4]),np.hstack(ims[4:6])]))
