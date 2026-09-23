import sys; sys.path.insert(0,'work'); sys.path.insert(0,'tools'); import arm, time
from camd_client import read_jpeg
x,y,z,p=map(float,sys.argv[1:5])
with arm.bus() as b:
    arm.gripper(b,0)
    arm.goto(b,x,y,z+0.04,pitch=p,speed=40,correct=2)
    print(arm.goto(b,x,y,z,pitch=p,speed=20,correct=3)[1], arm.tool_xyz(b).round(3))
    time.sleep(0.3); open('tools/top.jpg','wb').write(read_jpeg('top')[0])
