"""Explicit single-joint goal increments at visually reviewed empty-jaw clearance.
Does not replace absolute-target motion helpers or certify contact-pose response.
"""
import argparse,json,math,time
from pathlib import Path
from episode_motion import bus_open,state,snap
from calibrate_workspace import clamp_target
import camd_client,recording,fk
p=argparse.ArgumentParser();p.add_argument('joint',choices=['shoulder_pan','wrist_flex','wrist_roll'])
g=p.add_mutually_exclusive_group(required=True);g.add_argument('--delta',type=float);g.add_argument('--restore-report')
a=p.parse_args();j=a.joint
with bus_open() as b:
 before=state(b);goals=b.sync_read('Goal_Position');raw=b.sync_read('Goal_Position',normalize=False)
 if before['xyz']['z']<.10 or before['q']['gripper']<15:raise RuntimeError('Empty-jaw clearance gate failed')
 restore=None
 if a.restore_report:
  old=json.load(open(a.restore_report))
  if old['joint']!=j:raise ValueError('Restoration joint mismatch')
  target=old['goals_before'][j];restore=old['raw_goals_before'][j];step=target-goals[j]
 else:step=a.delta;target=goals[j]+step
 # Restoring a saved raw goal may span one encoder tick beyond requested2deg.
 limit=2+360/4095 if restore is not None else 2.01
 if not math.isfinite(step) or not 0<abs(step)<=limit:raise ValueError('Increment exceeds bounded goal step (including restoration quantization)')
 target=clamp_target({j:target},b.calibration)[j]
 if abs(target-before['q'][j])>7:raise RuntimeError('Target outside measured-error bound')
 if j=='wrist_roll' and not -37<=target<=-24:raise RuntimeError('Outside locally observed roll interval')
 pred=fk.forward({**before['q'],j:before['q'][j]+step})
 if pred['z']<.10 or math.dist(list(pred.values()),list(before['xyz'].values()))>.02:raise RuntimeError('Predicted clearance motion bound')
 stamp=str(time.time_ns());prefix='tools/mug_observations/'+stamp;snap(prefix+'_before')
 r=dict(joint=j,requested_increment_deg=step,before=before,goals_before=goals,raw_goals_before=raw,target_deg=target,first_command_deg=goals[j],evidence=prefix,error=None)
 def health():
  s=recording.request('status');v=s['latest']
  if s.get('failure') or any(v['fault_bits']) or not all(v['torque']):raise RuntimeError('Recorder/motor health gate')
  if not camd_client.alive():raise RuntimeError('Camera unavailable')
 try:
  for i in range(41):
   if i%4==0:health()
   q=b.sync_read('Present_Position');cmd=goals[j]+(target-goals[j])*i/40
   if abs(q[j]-cmd)>8:raise RuntimeError('Tracking bound')
   if fk.forward(q)['z']<.095:raise RuntimeError('Measured clearance bound')
   b.write('Goal_Position',j,cmd)
   if i==0:r['first_written_raw_goal']=b.read('Goal_Position',j,normalize=False)
   time.sleep(.05)
  if restore is not None:b.write('Goal_Position',j,restore,normalize=False)
  time.sleep(1);health()
 except BaseException as e:
  r['error']=str(e);b.write('Goal_Position',j,b.read('Present_Position',j))
 finally:
  r['after']=state(b);r['goals_after']=b.sync_read('Goal_Position');r['raw_goals_after']=b.sync_read('Goal_Position',normalize=False)
  r['other_raw_goals_unchanged']=all(r['raw_goals_after'][k]==raw[k] for k in raw if k!=j)
  r['first_raw_matches_old']=r.get('first_written_raw_goal')==raw[j]
  r['measured_delta_deg']=r['after']['q'][j]-before['q'][j]
  r['delta_xyz_m']={k:r['after']['xyz'][k]-before['xyz'][k] for k in before['xyz']}
  snap(prefix);snap('tools/mug_observations/latest');Path(prefix+'.json').write_text(json.dumps(r['after']))
  path='tools/clearance_increment_'+stamp+'.json';Path(path).write_text(json.dumps(r,indent=2))
  print(json.dumps({'report':path,**{k:r[k] for k in ['error','measured_delta_deg','delta_xyz_m','other_raw_goals_unchanged','first_raw_matches_old']}}))
 if r['error'] or not r['other_raw_goals_unchanged'] or not r['first_raw_matches_old']:raise RuntimeError('Diagnostic failed; inspect report')
