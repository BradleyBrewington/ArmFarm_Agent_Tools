"""Supervised episode stages. Inspect both snapshots before marking success."""
import json,sys,time
from pathlib import Path
import recording
from episode_motion import bus_open,move,snap,state
ROOT=Path('runs/20260918_cube_episodes');ROOT.mkdir(parents=True,exist_ok=True)
def save(name,s):
 snap(str(ROOT/name));(ROOT/(name+'.json')).write_text(json.dumps(s,indent=2));print(name,json.dumps(s),flush=True)
if __name__=='__main__':
 stage=sys.argv[1];name=sys.argv[2]
 with bus_open() as b:
  if stage in ('pickup','grasp'):
   save(name+'_start',state(b))
   if stage=='pickup':print(recording.start_recording('Pick black cube from '+name+'; bring to saved home pose and verify grasp'),flush=True)
   elif not recording.request('status')['recording']:raise RuntimeError('grasp requires an active episode')
   s=move(b,{'gripper':23},3)
   if abs(s['load']['gripper'])>140:
    s=move(b,{'gripper':s['q']['gripper']+.8},1)
   if abs(s['load']['gripper'])>160:raise RuntimeError('Grip load exceeds limit')
   save(name+'_grip',s)
   s=move(b,{'shoulder_lift':s['q']['shoulder_lift']-12,'elbow_flex':s['q']['elbow_flex']+5},6);save(name+'_lift',s)
  elif stage=='home':
   h=json.load(open('home_pose.json'))['joints'];h.pop('gripper');s=move(b,h,16);save(name+'_home',s)
  else:raise ValueError(stage)
