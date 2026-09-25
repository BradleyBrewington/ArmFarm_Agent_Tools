import sys; sys.path.insert(0,'work'); import arm, json, numpy as np
with arm.bus() as b:
    print(arm.joints(b))
    r, ang = arm.goto(b, 0.22, 0.0, 0.10, speed=40)
    print(json.dumps(r), ang, arm.tool_xyz(b))
