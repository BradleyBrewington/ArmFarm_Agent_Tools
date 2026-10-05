"""Read-only cooldown observation; never enables torque or moves the arm."""
import json,time,sys
from pathlib import Path
import calibrate_workspace as c,recording
rows=[];cool_since=None;ready=False
with c.connected_bus(recording.serial_port()) as b:
 if c.read_register(b,'Torque_Enable','gripper')[0]!=0:raise RuntimeError('Gripper must already be torque-off')
 started=time.monotonic()
 while time.monotonic()-started<float(sys.argv[1]):
  temp,f=c.read_register(b,'Present_Temperature','gripper');fault=c.fault_bits(b,'gripper')
  row={'time':time.time(),'temperature':temp,'fault':fault|f};rows.append(row)
  if fault or f:raise RuntimeError(str(row))
  if temp<=43:
   cool_since=cool_since or time.monotonic()
   ready=time.monotonic()-cool_since>=30
  else:cool_since=None
  if len(rows)%10==1:print(row,flush=True)
  if ready:break
  time.sleep(1)
result={'ready_for_supervised_retry':ready,'restart_target_C':43,'required_stable_seconds':30,'samples':rows}
Path('tools/cooldown_latest.json').write_text(json.dumps(result,indent=2))
print({'ready':ready,'samples':len(rows),'last':rows[-1]},flush=True)
