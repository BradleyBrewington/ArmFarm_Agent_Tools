"""Bounded cube collection with live home-image and motor-contact verification.
Starts holding cube at home. Stops on first failure; all logs/evidence stay in tools/.
"""
import sys,json,time,math
from pathlib import Path
import cv2
import calibrate_workspace as c,recording
from cube_pose import pose
from camd_client import read_frame
ROOT=Path(__file__).resolve().parent
HOME=c.load_home(ROOT.parent/'home_pose.json')

def verify(b,g,tag,held):
 checks=[]
 for i in range(3):
  if held:g.tick()
  p=b.sync_read('Present_Position')
  errors={k:abs(p[k]-v) for k,v in c.clamp_target(HOME,b.calibration).items()}
  if max(errors.values())>4:raise RuntimeError('Home tolerance exceeded '+str(errors))
  im,meta=read_frame('wrist');top,tm=read_frame('top')
  if abs(p['wrist_roll'])>2:raise RuntimeError('Wrist orientation invalid for image check')
  gray=cv2.cvtColor(im,cv2.COLOR_BGR2GRAY)
  dark=float((gray[540:650,670:850]<65).mean())
  if held:
   s=g.sample()
   if not 12<p['gripper']<45 or not 90<s['load']<240 or s['fault'] or dark<.85:
    raise RuntimeError('Held-object verification failed '+str((p['gripper'],s,dark)))
  elif float((gray[400:500,700:850]<65).mean())>.15 or p['gripper']<50:raise RuntimeError('Empty-home image or open-jaw check failed')
  checks.append({'dark_fraction':dark,'wrist_seq':meta['seq'],'top_seq':tm['seq'],'home_errors':errors,'gripper':p['gripper']})
  if i==2:
   cv2.imwrite(str(ROOT/f'{tag}_wrist.jpg'),im);cv2.imwrite(str(ROOT/f'{tag}_top.jpg'),top)
  time.sleep(.2)
 if len({v['wrist_seq'] for v in checks})!=3:raise RuntimeError('Camera frames did not advance')
 return checks

def run(points):
 with c.connected_bus(recording.serial_port()) as b:
  g=c.AdaptiveGrip(b);g.effort=min(g.effort,150);b.adaptive_grip=g
  def go(t,seconds):
   c.preflight(b,[t]);c.enable_at_current_position(b,list(t));c.move(b,t,seconds);r=c.settle(b,t,.25)
   if r['physical_stops']:raise RuntimeError('Physical stop discovered '+str(r))
   if max(r['error_degrees'].values(),default=0)>5:raise RuntimeError('Position error '+str(r))
   return r
  for n,(x,pan) in enumerate(points):
   started=time.time();tag=f'cube_{int(started)}';episode=None
   release=pose(x,.015,pan);pick=pose(x,.005,pan)
   approach=pose(x,.060,pan)
   for t in (release,pick,approach,HOME):
    clipped=c.clamp_target(t,b.calibration)
    if max(abs(t[k]-clipped[k]) for k in t)>.1:raise RuntimeError('Pose would be clipped')
   try:
    go(approach,2.5);go(release,1.2);go({'gripper':60},.7);go(approach,1.2);go(HOME,2.5)
    verify(b,g,tag+'_placed',False)
    b.disable_torque(['gripper'])
    for rest in range(10):
     temperature,reply_fault=c.read_register(b,'Present_Temperature','gripper')
     if reply_fault or c.fault_bits(b,'gripper') or temperature>=g.temp_limit:
      raise RuntimeError(f'Gripper rest check failed: temperature={temperature}, limit={g.temp_limit}, reply_fault={reply_fault}')
     time.sleep(1)
    go({'gripper':60},.3)
    episode=recording.start_recording('Black cube pickup to home; visual and motor-contact verification')['episode']
    print(json.dumps({'phase':'picking','episode':episode,'x':x,'pan':pan}),flush=True)
    go(approach,2.5);go(pick,1.2);grip=g.acquire()
    go({**approach,'gripper':0},1.2);go(HOME,2.5)
    checks=verify(b,g,tag+'_held',True)
    result=recording.stop_recording(True,'Cube retained at home: three fresh wrist images with black cube between jaws, sustained motor contact, no faults, all four home joints within4deg. '+json.dumps({'x':x,'pan':pan,'checks':checks}))
    episode=None
    row={'phase':'success','x':x,'pan':pan,'seconds':time.time()-started,'result':result,'grip':grip,'evidence':tag+'_held'}
    with (ROOT/'collection_log.jsonl').open('a') as f:f.write(json.dumps(row)+'\n')
    print(json.dumps(row),flush=True)
   except BaseException as exc:
    g.stop()
    if episode:print(recording.stop_recording(False,str(exc)),flush=True)
    print(json.dumps({'phase':'stopped','error':str(exc),'x':x,'pan':pan}),flush=True)
    raise
  print('BATCH_DONE_HOLDING_AT_HOME',flush=True)
if __name__=='__main__':run(json.loads(sys.argv[1]))
