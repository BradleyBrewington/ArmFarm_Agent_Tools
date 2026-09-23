import sys, json, math; sys.path.insert(0,'work'); sys.path.insert(0,'tools'); import arm, grasp, vision, cv2, time, numpy as np
from camd_client import read_frame
import run_episodes as R
with arm.bus() as b:
    R.lift_clear(b); arm.home(b)
    for off in [(-0.03,0.0),(-0.03,0.012),(-0.03,-0.012),(-0.04,0.0),(-0.02,0.0)]:
        f=R.reachable_prep(b); cx,cy,yaw,c=f
        # perpendicular shift: apply along tool y after roll chosen inside grasp -> emulate by shifting cube estimate
        roll=grasp.wrist_roll(b,cx,cy)
        q,_,_=grasp.kin.ik_down(cx,cy,0.02); q['wrist_roll']=roll; T=grasp.kin.pose(q)
        ty=T[:2,1]
        g=grasp.grasp(b,cx+off[1]*ty[0],cy+off[1]*ty[1],yaw,off=off[0]); print(off,g,flush=True)
        if g['ok']:
            json.dump({'off':off,'g':{k:float(v) for k,v in g.items()}},open('tools/search_ok.json','w')); break
        R.lift_clear(b); arm.home(b)
