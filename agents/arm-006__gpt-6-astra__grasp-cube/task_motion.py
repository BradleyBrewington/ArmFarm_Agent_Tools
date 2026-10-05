import sys,json,time
from pathlib import Path
import calibrate_workspace as c, recording
from camd_client import read_jpeg

def snap():
 for role in ('top','wrist'):
  jpg,m=read_jpeg(role);Path('tools/'+role+'.jpg').write_bytes(jpg)
if __name__=='__main__':
 target=json.loads(sys.argv[1])
 with c.connected_bus(recording.serial_port()) as b:
  c.preflight(b,[target]);c.enable_at_current_position(b,list(target));c.move(b,target,float(sys.argv[2]) if len(sys.argv)>2 else 2);print(json.dumps(c.settle(b,target,.4)));print(b.sync_read('Present_Position'))
 time.sleep(.15);snap()
