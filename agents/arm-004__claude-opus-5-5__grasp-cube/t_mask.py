import sys; sys.path.insert(0,'work'); sys.path.insert(0,'tools'); import arm, time, cv2, numpy as np
from camd_client import read_frame
with arm.bus() as b:
    arm.gripper(b,70)
    for i,(x,y) in enumerate([(0.2,-0.1),(0.16,-0.05)]):
        arm.goto(b,x,y,0.08,roll=79.5,speed=40,correct=0)
        arm.goto(b,x,y,0.045,roll=79.5,speed=30,correct=2); time.sleep(0.4)
        w,_=read_frame('wrist'); cv2.imwrite(f'tools/mask_src{i}.jpg',w)
    arm.goto(b,x,y,0.08,roll=79.5,speed=30,correct=0)
