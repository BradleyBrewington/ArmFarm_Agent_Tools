import sys; sys.path.insert(0,'work'); sys.path.insert(0,'tools'); import arm, servo, time
from camd_client import read_jpeg
x,y=float(sys.argv[1]),float(sys.argv[2]); zlow=float(sys.argv[3])
with arm.bus() as b:
    r=servo.servo(b,x,y); print(r)
    if r:
        arm.goto(b,r[0],r[1],zlow,speed=25,correct=3); time.sleep(0.3)
        print('low at',arm.tool_xyz(b).round(4))
        open('tools/g_low.jpg','wb').write(read_jpeg('wrist')[0]); open('tools/top.jpg','wb').write(read_jpeg('top')[0])
