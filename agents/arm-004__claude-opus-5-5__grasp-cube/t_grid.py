import sys; sys.path.insert(0,'work'); sys.path.insert(0,'tools'); import arm, json
from camd_client import read_jpeg
import time
pts=[(0.24,0.0),(0.18,-0.10),(0.18,0.10),(0.30,-0.10),(0.30,0.10)]
with arm.bus() as b:
    for i,(x,y) in enumerate(pts):
        arm.goto(b,x,y,0.08,speed=50); arm.goto(b,x,y,0.03,speed=30)
        time.sleep(0.5); open(f'tools/grid{i}.jpg','wb').write(read_jpeg('top')[0])
        print(i,(x,y),arm.tool_xyz(b).round(3).tolist(),flush=True)
        arm.goto(b,x,y,0.08,speed=40)
