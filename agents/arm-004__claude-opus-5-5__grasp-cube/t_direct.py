import sys; sys.path.insert(0,'work'); sys.path.insert(0,'tools'); import arm, time, cv2
from camd_client import read_frame
x,y,roll=map(float,sys.argv[1:4]); zg=float(sys.argv[4]) if len(sys.argv)>4 else 0.012
with arm.bus() as b:
    arm.gripper(b,85)
    arm.goto(b,x,y,0.08,roll=roll,speed=40,correct=1)
    arm.goto(b,x,y,zg,roll=roll,speed=20,correct=3); time.sleep(0.3)
    print('low',arm.tool_xyz(b).round(4))
    for g in (45,25,12,5,0):
        pos=arm.gripper(b,g,0.3); print(g,round(pos,1))
        if pos>g+6: break
    arm.goto(b,x,y,0.10,roll=roll,speed=30,correct=1); time.sleep(0.3)
    print('held',arm.joints(b)['gripper'])
    w,_=read_frame('wrist'); cv2.imwrite('tools/d_lift.jpg',w); t,_=read_frame('top'); cv2.imwrite('tools/top.jpg',t)
