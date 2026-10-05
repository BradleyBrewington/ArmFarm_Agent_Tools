"""Recorded cube cycles, with fresh camera and contact checks and bounded placement grid."""
import json,time,sys,signal,os
from pathlib import Path
import cv2,numpy as np
import calibrate_workspace as c, recording
from camd_client import read_frame
from cube_control import snapshot
HOME=c.load_home(Path('home_pose.json'))
LOG=Path('tools/cube_cycles.jsonl')
STATE=Path('tools/cube_cycle_state.json')
def emit(**d):
 d['time']=time.time()
 print(json.dumps(d),flush=True)
 with LOG.open('a') as f:f.write(json.dumps(d)+'\n')
def move(b,q,t=1.8):
 for role in ('top','wrist'):read_frame(role)
 q=c.clamp_target(q,b.calibration)
 b.enable_torque(list(q));c.move(b,q,t)
 report=c.settle(b,q,.25)
 if report['physical_stops']:raise RuntimeError('Physical stop: '+str(report))
 if max(report['error_degrees'].values(),default=0)>5:raise RuntimeError('Position miss: '+str(report))
 for j in c.JOINTS:
  if c.fault_bits(b,j):raise RuntimeError('Motor fault '+j)
def pose(pan,elbow=20):
 from scipy.optimize import brentq
 from fk import forward
 def height(s):return forward(dict(shoulder_pan=2,shoulder_lift=s,elbow_flex=elbow,wrist_flex=65-s-elbow,wrist_roll=-30))['z']+.002193685765342651
 s=brentq(height,-80,35)
 return dict(shoulder_pan=pan,shoulder_lift=s,elbow_flex=elbow,wrist_flex=65-s-elbow)
def above(q):return {**q,'shoulder_lift':q['shoulder_lift']-30,'wrist_flex':q['wrist_flex']+30}
def home(b):move(b,HOME,2.3)
def grip(b):
 p=b.read('Present_Position','gripper')
 for goal in range(int(p)-1,5,-1):
  c.move(b,{'gripper':goal},.05);time.sleep(.06)
  actual=b.read('Present_Position','gripper')
  if actual-goal>1.3:
   if not 18<actual<35:raise RuntimeError('Unexpected contact width: '+str(actual))
   b.write('Goal_Position','gripper',23.)
   time.sleep(.15)
   return actual
 raise RuntimeError('No cube contact')
def held(b):
 img,meta=read_frame('wrist')
 dark=float(np.mean(cv2.cvtColor(img[500:650,730:1030],cv2.COLOR_BGR2GRAY)<65))
 p=b.read('Present_Position','gripper')
 ok=15<p<35 and dark>.85
 return ok,dict(gripper=p,wrist_dark=dark,frame_seq=meta['seq'])
def run(n):
 if Path('tools/cube_recovery_required.json').exists():
  raise RuntimeError('Re-localize cube and validate pickup before clearing tools/cube_recovery_required.json')
 current=json.loads(STATE.read_text()) if STATE.exists() else pose(17)
 grid=[(pan,e) for e in (20,30,40) for pan in (-20,-5,10,25)]
 offset=int(os.environ.get("CUBE_GRID_OFFSET","0"))
 grid=grid[offset:]+grid[:offset]
 active=False
 with c.connected_bus(recording.serial_port()) as b:
  try:
   move(b,above(current));home(b)
   for i in range(n):
    ep=recording.start_recording('Pick black cube, bring to home, visually confirm retained grasp');active=True
    move(b,above(current));move(b,current,1.5)
    contact=grip(b)
    move(b,above(current),1.5)
    # Retain the seated closing target through lift and home.
    home(b)
    ok,evidence=held(b);snapshot()
    receipt=recording.stop_recording(ok,json.dumps(dict(source_pose=current,contact=contact,home_confirm=evidence)));active=False
    emit(event='episode',success=ok,receipt=receipt,evidence=evidence)
    if not ok:raise RuntimeError('Grasp not verified at home')
    target=pose(*grid[i%len(grid)])
    move(b,above(target));move(b,target,1.5)
    move(b,{'gripper':45},.5)
    current=target;STATE.write_text(json.dumps(current))
    move(b,above(current),1.5);home(b)
    snapshot();emit(event='placed',pose=current)
  except BaseException as e:
   emit(event='stopped',error=str(e))
   if active:
    receipt=recording.stop_recording(False,str(e))
    emit(event='episode',success=False,receipt=receipt,evidence={'error':str(e)})
   raise
if __name__=='__main__':run(int(sys.argv[1]) if len(sys.argv)>1 else 1)
