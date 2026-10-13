"""Small observed moves using live calibration and the supplied stall guard."""
import json,sys,time
from pathlib import Path
import calibrate_workspace as c
import recording
from camd_client import read_jpeg

def snapshot():
 for role in ('top','wrist'):
  data,meta=read_jpeg(role)
  Path(f'tools/current_{role}.jpg').write_bytes(data)
if __name__=='__main__':
 with c.connected_bus(recording.serial_port()) as b:
  target=json.loads(sys.argv[1]) if len(sys.argv)>1 else {}
  if target:
   target=c.clamp_target(target,b.calibration)
   b.enable_torque(list(target))
   c.move(b,target,float(sys.argv[2]) if len(sys.argv)>2 else 2)
   print(c.settle(b,target,.35))
  print('positions',b.sync_read('Present_Position'))
  print('faults',{j:c.fault_bits(b,j) for j in c.JOINTS})
  snapshot()
