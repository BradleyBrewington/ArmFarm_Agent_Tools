import sys; sys.path.insert(0,'work'); sys.path.insert(0,'tools'); import arm, time
from camd_client import read_jpeg
x,y=float(sys.argv[1]),float(sys.argv[2]); zg=float(sys.argv[3]) if len(sys.argv)>3 else 0.015
with arm.bus() as b:
    arm.gripper(b,65)
    arm.goto(b,x,y,0.08,speed=40,correct=4)
    arm.goto(b,x,y,zg,speed=25,correct=3); time.sleep(0.3)
    print('at', arm.tool_xyz(b).round(4))
    open('tools/g_low.jpg','wb').write(read_jpeg('wrist')[0])
    for g in [40,20,10,5,0]:
        cw_pos=arm.gripper(b,g,0.4)
        print('grip cmd',g,'pos',round(cw_pos,1))
    arm.goto(b,x,y,0.10,speed=25,correct=1)
    time.sleep(0.3); print('lifted grip', arm.joints(b)['gripper'])
    open('tools/g_lift.jpg','wb').write(read_jpeg('wrist')[0]); open('tools/top.jpg','wb').write(read_jpeg('top')[0])
