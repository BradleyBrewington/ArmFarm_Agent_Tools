import sys; sys.path.insert(0,'work'); import arm, grasp
with arm.bus() as b:
    grasp.place(b,0.0388+0.155*0.906,0.155*0.423,arm.joints(b)['wrist_roll']); arm.home(b)
