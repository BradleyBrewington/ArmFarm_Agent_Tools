"""Supervised gradual jaw closure; motor contact is not task success."""
import json,time,sys
from episode_motion import bus_open,state,snap
import camd_client,recording
contact_load=int(sys.argv[1]) if len(sys.argv)>1 else 100
if not 100<=contact_load<=120:raise ValueError('Contact target must be 100 through 120')
with bus_open() as b:
 if not recording.request('status')['recording']:raise RuntimeError('Active episode required')
 # A contact deflection makes measured position more open than the goal.
 # Preserve existing pressure when checking a grasp after lifting.
 goal=min(b.read('Present_Position','gripper'),b.read('Goal_Position','gripper'))
 while goal>0:
  if not camd_client.alive():raise RuntimeError('Camera unavailable')
  goal=max(0,goal-.2)
  b.write('Goal_Position','gripper',goal)
  time.sleep(.16)
  load=abs(b.read('Present_Load','gripper'))
  if load>=140:
   q=b.read('Present_Position','gripper');b.write('Goal_Position','gripper',q+.5)
   raise RuntimeError('Grip load limit reached; relieved jaw pressure')
  if load>=contact_load:
   time.sleep(.2)
   if abs(b.read('Present_Load','gripper'))>=contact_load:break
 s=state(b);snap('tools/mug_observations/grip');print(json.dumps({'goal':goal,'contact_target':contact_load,**s}))
 if abs(s['load']['gripper'])<contact_load:raise RuntimeError('No sustained contact detected; inspect images before moving')
