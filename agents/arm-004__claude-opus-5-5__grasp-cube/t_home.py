import sys; sys.path.insert(0,'work'); sys.path.insert(0,'tools'); import arm, time, cv2
from camd_client import read_frame
with arm.bus() as b:
    p=arm.tool_xyz(b)
    if p[2] < 0.07: arm.goto(b,p[0],p[1],0.09,speed=30,correct=0)
    arm.home(b); time.sleep(0.4)
    t,_=read_frame('top'); cv2.imwrite('tools/top.jpg',t)
