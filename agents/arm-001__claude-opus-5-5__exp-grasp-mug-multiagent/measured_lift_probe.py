"""Bounded outer-loop measured lift at verified clearance; no calibration/force writes."""
import json,sys,time
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares
from episode_motion import bus_open,state,move,snap
from episode_kinematics import matrix
names=['shoulder_pan','shoulder_lift','elbow_flex','wrist_flex']
root=Path('tools/mug_observations')
with bus_open() as b:
 s=state(b);q=s['q'];g=b.sync_read('Goal_Position');xyz=np.array([s['xyz'][k] for k in ['x','y','z']]);target=xyz+np.array([0,0,.01]);pitch=sum(q[j] for j in names[1:])
 def f(v):
  qq={**q,**dict(zip(names,v))};return np.r_[(matrix(qq)[:3,3]-target)*100,(sum(v[1:])-pitch)*.5]
 fit=least_squares(f,[q[j] for j in names],bounds=([-100,-100,-95,-95],[100,100,97,95]))
 delta={j:float(fit.x[i]-q[j]) for i,j in enumerate(names)}
 scale=min(1,2/max(abs(v) for v in delta.values()))
 command={j:g[j]+delta[j]*scale for j in names if abs(delta[j])>.01}
 plan=dict(before=s,old_goals=g,target_xyz=target.tolist(),desired_measured_delta_deg=delta,scale=scale,command=command,predicted_desired_residual=f(fit.x).tolist())
 print(json.dumps(plan),flush=True)
 if np.linalg.norm(f(fit.x)[:3])>.1 or abs(f(fit.x)[3])>.5:raise RuntimeError('Unacceptable desired-pose fit')
 if any(abs(command[j]-q[j])>7 for j in command):raise RuntimeError('Refuse goal more than7deg from measured')
 Path('tools/last_lift_probe_plan.json').write_text(json.dumps(plan,indent=2))
 if '--execute' in sys.argv:
  after=move(b,command,5,settle_tolerance=7);time.sleep(1);after=state(b)
  stamp=str(time.time_ns());prefix=str(root/stamp);snap(prefix);snap(str(root/'latest'));Path(prefix+'.json').write_text(json.dumps(after))
  result={**plan,'after':after,'evidence':prefix,'actual_delta_xyz':{k:after['xyz'][k]-s['xyz'][k] for k in s['xyz']}}
  Path('tools/lift_probe_'+stamp+'.json').write_text(json.dumps(result,indent=2));print(json.dumps(result),flush=True)
