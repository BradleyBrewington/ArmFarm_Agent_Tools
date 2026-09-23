import sys; sys.path.insert(0,'work'); sys.path.insert(0,'tools'); import arm, time, cv2, numpy as np
from camd_client import read_frame
x,y,roll,z,g=map(float,sys.argv[1:6])
with arm.bus() as b:
    arm.gripper(b,g)
    arm.goto(b,x,y,0.06,roll=roll,speed=40,correct=1)
    arm.goto(b,x,y,z,roll=roll,speed=15,correct=2); time.sleep(0.4)
    print(arm.tool_xyz(b).round(4), arm.joints(b))
    w,_=read_frame('wrist'); t,_=read_frame('top')
    cv2.imwrite('tools/look.jpg',np.hstack([cv2.resize(w,(640,360)),cv2.resize(t,(640,360))]))
    arm.goto(b,x,y,0.07,roll=roll,speed=30,correct=0)
