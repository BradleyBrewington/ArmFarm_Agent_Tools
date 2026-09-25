import sys; sys.path.insert(0,'work'); sys.path.insert(0,'tools'); import arm, time
from camd_client import read_jpeg
with arm.bus() as b:
    arm.goto(b,0.2,0.0,0.12,speed=40,correct=0)
    arm.home(b); time.sleep(0.5)
    print(arm.joints(b))
    open('tools/ref_home.jpg','wb').write(read_jpeg('top')[0])
