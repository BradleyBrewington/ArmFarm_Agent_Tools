"""Small shoulder-goal increment from the existing goal; all other goals untouched."""
import json,time,sys,math
from pathlib import Path
import numpy as np
from episode_motion import bus_open,state,snap
from calibrate_workspace import clamp_target
import camd_client,fk,recording
import argparse
ap=argparse.ArgumentParser();ap.add_argument('step',type=float,nargs='?',default=-2);ap.add_argument('--baseline-z',type=float,required=True)
args=ap.parse_args();step=args.step
if not -2<=step<0:raise ValueError('Upward diagnostic step must be -2 through <0 degrees')
with bus_open() as b:
 before=state(b);goals=b.sync_read('Goal_Position');j='shoulder_lift';target=clamp_target({j:goals[j]+step},b.calibration)[j]
 if abs(target-before['q'][j])>7:raise RuntimeError('Requested goal outside7deg measured-error bound')
 baseline_z=before['xyz']['z'] if args.baseline_z is None else args.baseline_z
 if not math.isfinite(baseline_z) or before['xyz']['z']-baseline_z>=.018:raise RuntimeError('Cumulative retention rise margin reached')
 pred=fk.forward({**before['q'],j:before['q'][j]+step})
 if not 0<pred['z']-before['xyz']['z']<=.02:raise RuntimeError('Predicted upward arc outside2cm bound')
 if pred['z']-baseline_z>.017:raise RuntimeError('Predicted cumulative rise leaves insufficient response margin')
 prefix='tools/mug_observations/'+str(time.time_ns());snap(prefix+'_before')
 error=None
 def health():
  h=recording.request('status');v=h['latest']
  if h.get('failure') or any(v['fault_bits']) or not all(v['torque']):raise RuntimeError('Motor/recorder health check failed')
 health()
 try:
  for i in range(41):
   if i%4==0:health()
   if not camd_client.alive():raise RuntimeError('Camera unavailable')
   q=b.sync_read('Present_Position');cmd=goals[j]+(target-goals[j])*(i/40)
   if abs(q[j]-cmd)>8:raise RuntimeError('Shoulder tracking bound')
   actual_z=fk.forward(q)['z']
   if actual_z-baseline_z>=.018:raise RuntimeError('Measured cumulative retention rise margin reached; hold')
   if actual_z<before['xyz']['z']-.003:raise RuntimeError('Unexpected downward motion; hold')
   b.write('Goal_Position',j,cmd);time.sleep(.05)
  for _ in range(10):
   health()
   if not camd_client.alive():raise RuntimeError('Camera unavailable during hold')
   if fk.forward(b.sync_read('Present_Position'))['z']-baseline_z>=.018:raise RuntimeError('Cumulative retention margin reached during hold')
   time.sleep(.1)
  after=state(b);after_goals=b.sync_read('Goal_Position')
 except BaseException as exc:
  error=str(exc)
  b.write('Goal_Position',j,b.read('Present_Position',j))
  after=state(b);after_goals=b.sync_read('Goal_Position')
 stamp=str(time.time_ns());prefix='tools/mug_observations/'+stamp;snap(prefix);snap('tools/mug_observations/latest');Path(prefix+'.json').write_text(json.dumps(after))
 r=dict(error=error,before=before,after=after,goals_before=goals,goals_after=after_goals,commanded_increment_deg=step,baseline_z=baseline_z,cumulative_rise_m=after['xyz']['z']-baseline_z,measured_shoulder_delta_deg=after['q'][j]-before['q'][j],delta_xyz_m={k:after['xyz'][k]-before['xyz'][k] for k in before['xyz']},pitch_delta_deg=sum(after['q'][k]-before['q'][k] for k in ['shoulder_lift','elbow_flex','wrist_flex']),other_goals_unchanged=all(after_goals[k]==goals[k] for k in goals if k!=j),evidence=prefix)
 Path('tools/shoulder_probe_'+stamp+'.json').write_text(json.dumps(r,indent=2));print(json.dumps(r),flush=True)

 if error:raise RuntimeError(error)
