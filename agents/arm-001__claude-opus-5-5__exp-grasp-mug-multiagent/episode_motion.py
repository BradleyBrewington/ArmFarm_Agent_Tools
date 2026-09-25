"""Supervised, bounded joint moves with camera and feedback checks."""
import json,sys,time,math
from pathlib import Path
from contextlib import contextmanager
from lerobot.motors import Motor,MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus
import recording,camd_client,fk
from calibrate_workspace import clamp_target
JOINTS=(*fk.JOINTS,'gripper')
class VerifiedBus(FeetechMotorsBus):
 def ping(self,motor,num_retry=0,raise_on_error=False):
  return super().ping(motor,num_retry=max(num_retry,3),raise_on_error=raise_on_error)
@contextmanager
def bus_open():
 b=VerifiedBus(port=recording.serial_port(),motors={j:Motor(i+1,'sts3215',MotorNormMode.RANGE_0_100 if j=='gripper' else MotorNormMode.DEGREES) for i,j in enumerate(JOINTS)})
 try:
  b.connect();b.calibration=b.read_calibration()
  if not all(b.sync_read('Torque_Enable',normalize=False).values()):raise RuntimeError('Torque not enabled')
  yield b
 finally:
  if b.is_connected:b.disconnect(disable_torque=False)
def state(b):
 q=b.sync_read('Present_Position');return {'q':q,'xyz':fk.forward(q),'load':b.sync_read('Present_Load'),'temperature':b.sync_read('Present_Temperature')}
def move(b,changes,seconds=3,settle_tolerance=4):
 if not 0<settle_tolerance<=7:raise ValueError("Invalid settle tolerance")
 if not camd_client.alive():raise RuntimeError('Camera unavailable')
 changes=clamp_target(changes,b.calibration)
 present=b.sync_read('Present_Position')
 start={j:present[j] for j in changes};target=dict(changes)
 for j,v in target.items():
  c=b.calibration[j];lo,hi=((0,85) if j=='gripper' else (-(c.range_max-c.range_min)*180/4095,(c.range_max-c.range_min)*180/4095))
  if not math.isfinite(v) or not lo<=v<=hi:raise ValueError((j,v,lo,hi))
 seconds=max(seconds,max(abs(target[j]-start[j]) for j in target)*1.5/12)
 t=time.monotonic();lastcheck=0
 try:
  while True:
   elapsed=time.monotonic()-t;f=min(elapsed/seconds,1);a=f*f*(3-2*f)
   cmd={j:start[j]+a*(target[j]-start[j]) for j in target}
   if elapsed-lastcheck>.2:
    if not camd_client.alive():raise RuntimeError('Camera stale during movement')
    q=b.sync_read('Present_Position')
    if any(abs(q[j]-cmd[j])>8 for j in target if j!='gripper'):raise RuntimeError('Arm tracking error')
    lastcheck=elapsed
   b.sync_write('Goal_Position',cmd)
   if f==1:break
   time.sleep(.025)
  time.sleep(.3)
 except BaseException:
  q=b.sync_read('Present_Position');b.sync_write('Goal_Position',{j:q[j] for j in target});raise
 q=b.sync_read('Present_Position')
 if any(abs(q[j]-target[j])>settle_tolerance for j in target if j!='gripper'):raise RuntimeError('Arm did not settle')
 return state(b)
def snap(prefix):
 for role in ('top','wrist'):
  jpg,meta=camd_client.read_jpeg(role);Path(f'{prefix}_{role}.jpg').write_bytes(jpg)
if __name__=='__main__':
 with bus_open() as b:
  r=move(b,json.loads(sys.argv[1]),float(sys.argv[2]) if len(sys.argv)>2 else 3) if len(sys.argv)>1 else state(b)
  snap('/tmp/arm');print(json.dumps(r))
  with open('/tmp/arm_moves.jsonl','a') as f:f.write(json.dumps({'time':time.time(),**r})+'\n')
