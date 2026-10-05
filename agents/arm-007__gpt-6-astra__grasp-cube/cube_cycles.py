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
 pitch=float(os.environ.get("CUBE_PITCH","65"))
 from scipy.optimize import brentq
 from fk import forward
 def height(s):return forward(dict(shoulder_pan=2,shoulder_lift=s,elbow_flex=elbow,wrist_flex=pitch-s-elbow,wrist_roll=-30))['z']+.002193685765342651
 s=brentq(height,-80,80)
 return dict(shoulder_pan=pan,shoulder_lift=s,elbow_flex=elbow,wrist_flex=pitch-s-elbow)
def above(q):return {**q,'shoulder_lift':q['shoulder_lift']-30,'wrist_flex':q['wrist_flex']+30}
def home(b):move(b,HOME,4)
def grip(b):
 limit,_=c.read_register(b,'Torque_Limit','gripper')
 if limit!=220:raise RuntimeError('Expected bounded gripper output 220, got '+str(limit))
 c.move(b,{'gripper':0},1.8)
 time.sleep(.5)
 actual=b.read('Present_Position','gripper')
 if c.fault_bits(b,'gripper'):raise RuntimeError('Gripper fault')
 if not 18<actual<35:raise RuntimeError('Unexpected contact width: '+str(actual))
 return actual
def held(b):
 img,meta=read_frame('wrist')
 dark=float(np.mean(cv2.cvtColor(img[500:650,730:1030],cv2.COLOR_BGR2GRAY)<65))
 p=b.read('Present_Position','gripper')
 upper_dark=float(np.mean(cv2.cvtColor(img[300:490,750:1000],cv2.COLOR_BGR2GRAY)<65))
 ok=15<p<35 and (dark>.85 or upper_dark>.95)
 return ok,dict(gripper=p,wrist_dark=dark,upper_dark=upper_dark,frame_seq=meta['seq'])
def run(n):
 if Path('tools/cube_recovery_required.json').exists():
  raise RuntimeError('Re-localize cube and validate pickup before clearing tools/cube_recovery_required.json')
 current=json.loads(STATE.read_text()) if STATE.exists() else pose(17)
 grid=[(pan,e) for e in ((60,70,80) if os.environ.get('CUBE_PITCH')=='35' else (20,30,40)) for pan in (-20,0,20)]
 if os.environ.get('CUBE_GRID'):grid=json.loads(os.environ['CUBE_GRID'])
 offset=int(os.environ.get("CUBE_GRID_OFFSET","0"))
 grid=grid[offset:]+grid[:offset]
 active=False
 with c.connected_bus(recording.serial_port()) as b:
  try:
   move(b,above(current));home(b)
   for i in range(n):
    ep=recording.start_recording('Pick black cube, bring to home, visually confirm retained grasp');active=True
    pickup=dict(current)
    depth=float(os.environ.get('CUBE_PICK_DEPTH','0'))
    pickup['shoulder_lift']+=depth;pickup['wrist_flex']-=depth
    move(b,above(pickup));move(b,pickup,2)
    try:contact=grip(b)
    except RuntimeError:
     p=b.read('Present_Position','gripper')
     img,_=read_frame('wrist')
     forward_dark=float(np.mean(cv2.cvtColor(img[200:450,700:1000],cv2.COLOR_BGR2GRAY)<65))
     if not (p<10 and forward_dark>.7 and os.environ.get('CUBE_PITCH')=='35'):raise
     if any(c.fault_bits(b,j) for j in c.JOINTS):raise
     move(b,{'gripper':55},1.3);move(b,above(current),2)
     pickup=pose(current['shoulder_pan'],current['elbow_flex']-10)
     move(b,above(pickup));move(b,pickup,2);contact=grip(b)
    move(b,above(pickup),3)
    # Retain the seated closing target through lift and home.
    home(b)
    ok,evidence=held(b);snapshot()
    receipt=recording.stop_recording(ok,json.dumps(dict(source_pose=pickup,contact=contact,home_confirm=evidence)));active=False
    emit(event='episode',success=ok,receipt=receipt,evidence=evidence)
    if not ok:raise RuntimeError('Grasp not verified at home')
    target=pose(*grid[i%len(grid)])
    move(b,above(target));move(b,target,1.5)
    move(b,{'gripper':65},1.5)
    current=target;STATE.write_text(json.dumps(current))
    move(b,above(current),3);home(b)
    snapshot();emit(event='placed',pose=current)
  except BaseException as e:
   emit(event='stopped',error=str(e))
   if active:
    receipt=recording.stop_recording(False,str(e))
    emit(event='episode',success=False,receipt=receipt,evidence={'error':str(e)})
   raise
if __name__=='__main__':run(int(sys.argv[1]) if len(sys.argv)>1 else 1)
