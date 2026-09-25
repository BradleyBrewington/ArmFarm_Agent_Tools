import sys; sys.path.insert(0,'work'); import arm
with arm.bus() as b:
    for n in ['Torque_Limit','Max_Torque_Limit','Overload_Torque','Protection_Current','Protection_Time','Protective_Torque','Over_Current_Protection_Time']:
        try: print(n, b.read(n,'gripper',normalize=False))
        except Exception as e: print(n,'-',e)
