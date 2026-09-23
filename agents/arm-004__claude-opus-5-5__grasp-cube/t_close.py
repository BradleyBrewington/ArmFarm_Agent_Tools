import sys; sys.path.insert(0,'work'); sys.path.insert(0,'tools'); import arm, time, cv2
from camd_client import read_frame
x,y,roll,z=map(float,sys.argv[1:5])
with arm.bus() as b:
    arm.gripper(b,85)
    arm.goto(b,x,y,0.06,roll=roll,speed=40,correct=1)
    arm.goto(b,x,y,z,roll=roll,speed=15,correct=2); time.sleep(0.3)
    for g in (45,25,12,5,0):
        pos=arm.gripper(b,g,0.3)
        if pos>g+6: break
    grip=arm.joints(b)['gripper']
    if grip>4: arm.gripper(b,grip-1,0.2)
    arm.goto(b,x,y,0.10,roll=roll,speed=30,correct=1); time.sleep(0.3)
    print('held',arm.joints(b)['gripper'])
    w,_=read_frame('wrist'); cv2.imwrite('tools/g_lift.jpg',w)
