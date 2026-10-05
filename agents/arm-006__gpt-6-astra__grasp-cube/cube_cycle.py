import sys,json,time
from pathlib import Path
import calibrate_workspace as c,recording
from task_motion import snap
home=c.load_home(Path('home_pose.json'))
mode=sys.argv[1]
if len(sys.argv)==5:
 from cube_pose import pose
 pick=pose(*map(float,sys.argv[2:]))
else: pick=json.loads(sys.argv[2])
with c.connected_bus(recording.serial_port()) as b:
 g=c.AdaptiveGrip(b);g.effort=min(g.effort,150);b.adaptive_grip=g
 def go(p,t=2):
  c.preflight(b,[p]);c.enable_at_current_position(b,list(p));c.move(b,p,t);return c.settle(b,p,.3)
 approach={**pick,'shoulder_lift':pick['shoulder_lift']-5,'elbow_flex':pick['elbow_flex']-12,'wrist_flex':pick['wrist_flex']+17}
 if mode=='place':
  go(approach,3);go(pick,1.5);go({'gripper':60},.8);go(approach,1.5);go(home,3)
  print('PLACED_AND_HOME',flush=True)
 elif mode=='pick':
  print(recording.start_recording('Pick up black cube and bring to home; verify held object'),flush=True)
  try:
   go(approach,3);go(pick,1.5);print(g.acquire(),flush=True)
   go({**approach,'gripper':0},1.5)
   result=go(home,3)
   if not result['within_tolerance']:raise RuntimeError('Home not reached')
   print(result,flush=True);print(g.sample(),flush=True)
  except Exception:
   g.stop();snap();print(recording.stop_recording(False,'Controller failed before verified home grasp'),flush=True);raise
  print('VERIFY_HOME_IMAGES_BEFORE_SUCCESS',flush=True)
 snap()
