import sys; sys.path.insert(0,'work'); sys.path.insert(0,'tools'); import arm, servo, time
from camd_client import read_jpeg
x,y=float(sys.argv[1]),float(sys.argv[2]); zg=float(sys.argv[3])
with arm.bus() as b:
    arm.goto(b,x,y,0.08,speed=40,correct=2)
    arm.gripper(b,65)
    r=servo.servo(b,x,y); print(r[:2])
    arm.goto(b,r[0],r[1],zg+0.03,speed=30,correct=2)
    arm.goto(b,r[0],r[1],zg,speed=20,correct=3); time.sleep(0.3)
    print('low at',arm.tool_xyz(b).round(4))
    open('tools/g_low.jpg','wb').write(read_jpeg('wrist')[0])
    for g in [40,20,10,5,0]:
        print('grip cmd',g,'pos',round(arm.gripper(b,g,0.4),1))
    arm.goto(b,r[0],r[1],0.10,speed=25,correct=1)
    time.sleep(0.3); print('lifted grip', arm.joints(b)['gripper'])
    open('tools/g_lift.jpg','wb').write(read_jpeg('wrist')[0]); open('tools/top.jpg','wb').write(read_jpeg('top')[0])
