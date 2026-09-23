"""Camera-supervised mug motions; caller inspects each resulting snapshot."""
import json,sys,time
from pathlib import Path
import numpy as np
from episode_motion import bus_open,move,state,snap
from episode_kinematics import solve,matrix
from calibrate_workspace import load_home
import recording
root=Path('tools/mug_observations');root.mkdir(exist_ok=True)
with bus_open() as b:
 mode=sys.argv[1];q=b.sync_read('Present_Position')
 if mode=='pitch':
  from scipy.optimize import least_squares
  xyz=np.array([float(x) for x in sys.argv[2:5]]);pitch=float(sys.argv[5]);names=['shoulder_pan','shoulder_lift','elbow_flex','wrist_flex']
  def fun(v):
   js=dict(zip(names,v));js['wrist_roll']=q['wrist_roll']
   return np.r_[(matrix(js)[:3,3]-xyz)*100,(sum(v[1:])-pitch)*.15]
  fit=least_squares(fun,[q[j] for j in names],bounds=([-100,-100,-95,-95],[100,100,97,95]))
  if np.linalg.norm(fun(fit.x)[:3])>.8:raise RuntimeError('IK position error')
  s=move(b,dict(zip(names,fit.x)),6,settle_tolerance=7)
 elif mode=='xyz':
  xyz=np.array([float(x) for x in sys.argv[2:5]])
  direction=np.array([0.,0.,-1.]) if len(sys.argv)>5 else matrix(q)[:3,2]
  sol=solve(xyz,q,roll=q['wrist_roll'],direction=direction)
  if sol['error']>.008:raise RuntimeError(sol)
  print(json.dumps(sol),flush=True)
  s=move(b,{j:v for j,v in sol['q'].items() if j!='wrist_roll'},5,settle_tolerance=7)
 elif mode=='home':s=move(b,load_home(Path('home_pose.json')),16)
 elif mode=='joint':s=move(b,json.loads(sys.argv[2]),5)
 else:s=state(b)
 name=str(root/str(time.time_ns()));snap(name);snap(str(root/'latest'))
 (Path(name+'.json')).write_text(json.dumps(s));print(json.dumps(s),flush=True)
