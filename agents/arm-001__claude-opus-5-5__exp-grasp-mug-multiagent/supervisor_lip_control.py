"""Recorded, bounded arm002 test actions. Live calibration and camera guards."""
import sys,json,time,math,fcntl,subprocess,socket
from pathlib import Path
import numpy as np
from calibrate_workspace import JOINTS,connected_bus,clamp_target,preflight,enable_at_current_position
from recording import serial_port,request,start_recording
from camd_client import read_jpeg
try:
 from check_saved_alignment import arm_transform
except ImportError:
 from episode_kinematics import matrix as arm_transform
ROOT=Path(__file__).resolve().parent
OFFSET=np.array([-.004200149,-.025801957,.003846939,1]) if socket.gethostname()=='arm-002' else np.array([0,0,0,1])
def state(b):
 return {'joints':b.sync_read('Present_Position',list(JOINTS)),'goals':b.sync_read('Goal_Position',list(JOINTS)), 'torque':b.sync_read('Torque_Enable',list(JOINTS),normalize=False),'faults':{j:b.read('Status',j,normalize=False) for j in JOINTS},'load':b.sync_read('Present_Load',list(JOINTS))}
def frames(prefix):
 for role in ('top','wrist'):
  im,meta=read_jpeg(role)
  if meta.get('age',0)>.5:raise RuntimeError('Stale camera')
  (ROOT/(prefix+'_'+role+'.jpg')).write_bytes(im)
  (ROOT/('supervisor_current_'+role+'.jpg')).write_bytes(im)
def main(a):
 stamp=str(time.time_ns());prefix='supervisor_lip_'+stamp;report={'time':time.time(),'action':a}
 with open('/tmp/armfarm-workspace-calibration.lock','a') as lock:
  fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
  procs=subprocess.check_output(['ps','-eo','args'],text=True).splitlines()
  if any('codex' in p and 'app-server' in p for p in procs):raise RuntimeError('Agent app-server active')
  frames(prefix+'_before')
  if a['op'] in ('init','start') and not request('status')['recording']:
   report['episode']=start_recording('User test: simulated lip grasp transferred to physical '+socket.gethostname()+', supervised bounded motions; active supervisor test, do not close externally')
  if a['op']!='snap' and not request('status')['recording']:raise RuntimeError('Recording required')
  with connected_bus(serial_port()) as b:
   before=state(b);report['before']=before
   if any(before['faults'].values()):raise RuntimeError('Motor fault')
   try:
    if a['op']=='init':
     if any(before['torque'].values()):raise RuntimeError('Init expects all motors disabled')
     raw=b.sync_read('Present_Position',list(JOINTS),normalize=False)
     if any(not b.calibration[j].range_min<=v<=b.calibration[j].range_max for j,v in raw.items()):raise RuntimeError('Outside calibration')
     preflight(b,[before['joints']]);enable_at_current_position(b);time.sleep(.5)
    elif a['op']=='delta':
     delta=a['delta']
     if not delta or any(j not in JOINTS or not math.isfinite(v) or abs(v)>10 for j,v in delta.items()):raise ValueError('At most ten degrees/percent per action')
     if not all(before['torque'].values()):raise RuntimeError('Torque disabled')
     old=before['goals'];target={j:old[j]+v for j,v in delta.items()}
     clipped=clamp_target(target,b.calibration)
     if any(abs(target[j]-clipped[j])>1e-6 for j in target):raise ValueError('Would exceed live calibration')
     preflight(b,[target]);duration=max(2.,float(a.get('seconds',4)),max(map(abs,delta.values()))/2)
     samples=[];start=time.monotonic()
     while True:
      f=min(1.,(time.monotonic()-start)/duration);alpha=f*f*(3-2*f)
      command={j:old[j]+alpha*delta[j] for j in delta}
      s=state(b)
      if any(s['faults'].values()) or not all(s['torque'].values()):raise RuntimeError('Fault or torque lost')
      for role in ('top','wrist'):read_jpeg(role)
      if any(abs(s['joints'][j]-command[j])>8 for j in delta if j!='gripper'):raise RuntimeError('Tracking error')
      b.sync_write('Goal_Position',command);samples.append({'time':time.time(),'command':command,'state':s})
      if f>=1:break
      time.sleep(.10)
     time.sleep(.5);report['samples']=samples
    elif a['op'] not in ('snap','start'):raise ValueError('Unknown operation')
    after=state(b);report['after']=after
    report['diagnostic_xyz']=(arm_transform(after['joints'])@OFFSET)[:3].tolist()
   except BaseException as e:
    report['error']=str(e)
    if a['op']=='delta':
     raw=b.sync_read('Present_Position',list(a['delta']),normalize=False);b.sync_write('Goal_Position',raw,normalize=False)
     report['recovery']='Held requested joints at measured raw position'
    raise
   finally:(ROOT/(prefix+'.json')).write_text(json.dumps(report,indent=2))
  frames(prefix+'_after')
  print(json.dumps({'report':prefix+'.json','after':report['after'],'diagnostic_xyz':report['diagnostic_xyz']}))
if __name__=='__main__':main(json.loads(Path(sys.argv[1]).read_text()))
