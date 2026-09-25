import sys; sys.path.insert(0,'work'); sys.path.insert(0,'tools'); import arm, time
from camd_client import read_jpeg
x,y,z=map(float,sys.argv[1:4]); g=float(sys.argv[4]) if len(sys.argv)>4 else None
with arm.bus() as b:
    if g is not None: arm.gripper(b,g)
    print(arm.goto(b,x,y,z,speed=40,correct=4)[1]); time.sleep(0.5)
    print(arm.joints(b), arm.tool_xyz(b))
    open('tools/wrist.jpg','wb').write(read_jpeg('wrist')[0]); open('tools/top.jpg','wb').write(read_jpeg('top')[0])
