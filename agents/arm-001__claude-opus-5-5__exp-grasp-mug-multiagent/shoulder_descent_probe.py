"""Bounded downward shoulder-only increment, validated from measured pose; all other goals untouched."""
import json,time,sys,math
from pathlib import Path
import numpy as np
from episode_motion import bus_open,state,snap
from calibrate_workspace import clamp_target
import camd_client,fk
step=float(sys.argv[1]) if len(sys.argv)>1 else 1
if not 0<step<=1:raise ValueError('Descent increment must be >0 through1 degree')
with bus_open() as b:
 before=state(b);goals=b.sync_read('Goal_Position');j='shoulder_lift';target=clamp_target({j:goals[j]+step},b.calibration)[j]
 if abs(target-before['q'][j])>7:raise RuntimeError('Requested goal outside7deg measured-error bound')
 pred=fk.forward({**before['q'],j:before['q'][j]+step})
 if not -.006<=pred['z']-before['xyz']['z']<0:raise RuntimeError('Predicted descent outside6mm bound')
 if pred['z']<.025:raise RuntimeError('Predicted tool height below25mm diagnostic floor')
 prefix='tools/mug_observations/'+str(time.time_ns());snap(prefix+'_before')
 try:
  for i in range(41):
   if not camd_client.alive():raise RuntimeError('Camera unavailable')
   q=b.sync_read('Present_Position');cmd=goals[j]+(target-goals[j])*(i/40)
   xyz=fk.forward(q)
   if xyz['z']<.023 or before['xyz']['z']-xyz['z']>.008:raise RuntimeError('Measured descent bound exceeded')
   if abs(q[j]-cmd)>8:raise RuntimeError('Shoulder tracking bound')
   b.write('Goal_Position',j,cmd);time.sleep(.05)
  time.sleep(1)
  after=state(b);after_goals=b.sync_read('Goal_Position')
 except BaseException:
  b.write('Goal_Position',j,b.read('Present_Position',j));raise
 stamp=str(time.time_ns());prefix='tools/mug_observations/'+stamp;snap(prefix);snap('tools/mug_observations/latest');Path(prefix+'.json').write_text(json.dumps(after))
 r=dict(before=before,after=after,goals_before=goals,goals_after=after_goals,commanded_increment_deg=step,measured_shoulder_delta_deg=after['q'][j]-before['q'][j],delta_xyz_m={k:after['xyz'][k]-before['xyz'][k] for k in before['xyz']},pitch_delta_deg=sum(after['q'][k]-before['q'][k] for k in ['shoulder_lift','elbow_flex','wrist_flex']),other_goals_unchanged=all(after_goals[k]==goals[k] for k in goals if k!=j),evidence=prefix)
 Path('tools/descent_probe_'+stamp+'.json').write_text(json.dumps(r,indent=2));print(json.dumps(r),flush=True)
