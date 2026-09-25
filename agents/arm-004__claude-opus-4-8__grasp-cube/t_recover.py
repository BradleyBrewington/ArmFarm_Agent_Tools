import sys; sys.path.insert(0,'work'); import arm, time
with arm.bus() as b:
    j=arm.joints(b)
    arm.move(b,{'shoulder_lift':-45.0},speed=20); print(arm.joints(b))
    arm.move(b,{'wrist_flex':40.0,'elbow_flex':70.0},speed=20); print(arm.joints(b))
    arm.home(b,speed=30); print(arm.joints(b), arm.tool_xyz(b))
