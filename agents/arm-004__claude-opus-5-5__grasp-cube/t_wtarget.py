"""Cube is held: set it down in place, open, rise and record where it appears in the wrist camera."""
import sys, json; sys.path.insert(0,'work'); sys.path.insert(0,'tools'); import arm, grasp, vision, servo, cv2, time, numpy as np
from camd_client import read_frame
with arm.bus() as b:
    j=arm.joints(b); roll=j['wrist_roll']; p=arm.tool_xyz(b); x,y=float(p[0]),float(p[1])
    arm.goto(b,x,y,grasp.GRASP_Z+0.003,roll=roll,speed=20,correct=2)
    arm.gripper(b,grasp.OPEN,0.5)
    out={'roll':roll,'x':x,'y':y}
    for z in [0.03,0.045,0.06,0.08,0.11]:
        arm.goto(b,x,y,z,roll=roll,speed=15,correct=2); time.sleep(0.4)
        img,_=read_frame('wrist'); cv2.imwrite(f'tools/wt_{int(z*1000)}.jpg',img)
        d=vision.wrist_cube(img); dl=vision.wrist_cube_low(img)
        out[str(z)]={'wrist_cube':d and {k:d[k] for k in ('cx','cy','area','angle','bbox')},
                     'low':dl and {k:dl[k] for k in ('bx','by','area','bbox')}, 'tool':arm.tool_xyz(b).tolist()}
        print(z,out[str(z)],flush=True)
    json.dump(out,open('tools/wrist_targets.json','w'),indent=1)
