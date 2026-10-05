import time
import calibrate_workspace as c,recording
from cube_control import snapshot
with c.connected_bus(recording.serial_port()) as b:
 p=b.read('Present_Position','gripper')
 for goal in range(int(p)-1,4,-1):
  c.move(b,{'gripper':goal},.08)
  time.sleep(.1)
  actual=b.read('Present_Position','gripper')
  if actual-goal>1.3:
   b.write('Goal_Position','gripper',actual-1.0)
   print('contact',actual,'hold',actual-1.0,flush=True)
   break
 else:print('no contact',flush=True)
 time.sleep(.3)
 print('fault',c.fault_bits(b,'gripper'))
 snapshot()
