import sys,json,time
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares
from calibrate_workspace import connected_bus,clamp_target,preflight,enable_at_current_position,load_home,JOINTS
from episode_motion import move_checked
from recording import serial_port
from camd_client import read_jpeg
from check_saved_alignment import arm_transform
ROOT=Path(__file__).resolve().parent
OFFSET=np.array([-.004200149,-.025801957,.003846939,1])
def snap():
 stamp=str(time.time_ns()); obs=ROOT/'mug_observations';obs.mkdir(exist_ok=True)
 with connected_bus(serial_port()) as b:
  j=b.sync_read('Present_Position',list(JOINTS));print(json.dumps({'joints':j,'xyz':(arm_transform(j)@OFFSET)[:3].tolist(),'load':b.sync_read('Present_Load',list(JOINTS))}),flush=True)
 (obs/(stamp+'.json')).write_text(json.dumps({'time':time.time(),'joints':j,'estimated_tool_xyz':(arm_transform(j)@OFFSET)[:3].tolist()},indent=2))
 for c in ('top','wrist'):
  im,m=read_jpeg(c);(ROOT/('current_'+c+'.jpg')).write_bytes(im);(obs/(stamp+'_'+c+'.jpg')).write_bytes(im)
 print('observation_stem='+str(obs/stamp),flush=True)
def step(target,secs=5):
 for c in ('top','wrist'):read_jpeg(c)
 with connected_bus(serial_port()) as b:
  target=clamp_target(target,b.calibration);preflight(b,[target]);enable_at_current_position(b,list(target))
 move_checked(target,secs);snap()
def ik(xyz,pitch=90,roll=None):
 if roll is None:
  with connected_bus(serial_port()) as b:roll=b.read("Present_Position","wrist_roll")
 def f(q):
  j=dict(zip(JOINTS[:4],q));j['wrist_roll']=roll
  return np.r_[((arm_transform(j)@OFFSET)[:3]-xyz)*100,(sum(q[1:])-pitch)*.1]
 fit=least_squares(f,[0,0,40,50],bounds=([-100,-110,-92,-107],[100,110,92,107]))
 if np.linalg.norm(f(fit.x))>.1:raise ValueError('Unreachable')
 return dict(zip(JOINTS[:4],fit.x))
if __name__=='__main__':
 if sys.argv[1]=='snap':snap()
 elif sys.argv[1]=='home':step(load_home(Path('home_pose.json')),15)
 elif sys.argv[1]=='xyz':step(ik(json.loads(sys.argv[2]),float(sys.argv[3]) if len(sys.argv)>3 else 90),8)
 else:step(json.loads(sys.argv[1]),float(sys.argv[2]) if len(sys.argv)>2 else 5)
