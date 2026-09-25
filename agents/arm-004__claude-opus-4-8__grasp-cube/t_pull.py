import sys; sys.path.insert(0,'work'); sys.path.insert(0,'tools'); import arm, time, numpy as np
from camd_client import read_jpeg
x0,y,x1=map(float,sys.argv[1:4])
def pitch(x): return float(np.clip((x-0.29)/(0.455-0.29)*68,0,68))
with arm.bus() as b:
    arm.gripper(b,0)
    arm.goto(b,x0,y,0.07,pitch=pitch(x0),speed=40,correct=2)
    arm.goto(b,x0,y,0.025,pitch=pitch(x0),speed=20,correct=3)
    for x in np.arange(x0,x1-1e-6,-0.01):
        arm.goto(b,x,y,0.02,pitch=pitch(x),speed=25,correct=1)
    print(arm.tool_xyz(b).round(3))
    arm.goto(b,x1,y,0.08,speed=30,correct=1)
    time.sleep(0.3); open('tools/top.jpg','wb').write(read_jpeg('top')[0])
