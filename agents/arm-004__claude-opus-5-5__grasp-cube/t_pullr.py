import sys, math; sys.path.insert(0,'work'); sys.path.insert(0,'tools'); import arm, time, numpy as np
from camd_client import read_jpeg
ang=math.radians(float(sys.argv[1])); r0=float(sys.argv[2]); r1=float(sys.argv[3])
SX=0.0388
def xy(r): return SX+r*math.cos(ang), r*math.sin(ang)
def pitch(r): return float(np.clip((r-0.26)/(0.43-0.26)*68,0,70))
with arm.bus() as b:
    arm.gripper(b,0)
    x,y=xy(r0)
    arm.goto(b,x,y,0.07,pitch=pitch(r0),speed=40,correct=2)
    arm.goto(b,x,y,0.025,pitch=pitch(r0),speed=20,correct=3)
    for r in np.arange(r0,r1-1e-6,-0.01):
        x,y=xy(r); arm.goto(b,x,y,0.02,pitch=pitch(r),speed=25,correct=1)
    print(arm.tool_xyz(b).round(3))
    arm.goto(b,x,y,0.08,speed=30,correct=1)
    time.sleep(0.3); open('tools/top.jpg','wb').write(read_jpeg('top')[0])
