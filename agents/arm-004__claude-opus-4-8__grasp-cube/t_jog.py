import sys; sys.path.insert(0,'work'); sys.path.insert(0,'tools'); import arm, time
from camd_client import read_jpeg
x,y,z=0.30,0.083,0.08
with arm.bus() as b:
    for i,(dx,dy) in enumerate([(0,0),(0.02,0),(0,0.02)]):
        arm.goto(b,x+dx,y+dy,z,speed=40,correct=4); time.sleep(0.4)
        print(i, arm.tool_xyz(b).round(4)); open(f'tools/jog{i}.jpg','wb').write(read_jpeg('wrist')[0])
