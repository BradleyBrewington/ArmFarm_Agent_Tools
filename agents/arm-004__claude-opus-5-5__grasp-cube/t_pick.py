import sys; sys.path.insert(0,'work'); sys.path.insert(0,'tools'); import arm, pick
from camd_client import read_jpeg
with arm.bus() as b:
    print(pick.pick(b, float(sys.argv[1]), float(sys.argv[2]), save='tools/p'))
    open('tools/top.jpg','wb').write(read_jpeg('top')[0])
