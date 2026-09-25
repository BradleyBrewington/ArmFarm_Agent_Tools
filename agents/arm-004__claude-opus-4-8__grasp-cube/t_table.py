import sys; sys.path.insert(0,'work'); import arm, time
x,y=0.18,-0.08
with arm.bus() as b:
    arm.gripper(b,0)
    arm.goto(b,x,y,0.05,roll=83.4,speed=40,correct=2)
    for z in [0.03,0.02,0.01,0.0,-0.01,-0.02,-0.03]:
        arm.goto(b,x,y,z,roll=83.4,speed=10,correct=0); time.sleep(0.4)
        j=arm.joints(b); print(z, arm.tool_xyz(b).round(4), round(j['shoulder_lift'],1), round(j['wrist_flex'],1), flush=True)
    arm.goto(b,x,y,0.06,roll=83.4,speed=30,correct=0)
