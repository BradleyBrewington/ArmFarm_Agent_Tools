"""Save a three-frame stationary check, with arm state and camera metadata."""
import time,json
from pathlib import Path
from episode_motion import bus_open,state
import camd_client
with bus_open() as b:
 p=Path('tools/pose_match_'+str(time.time_ns()));p.mkdir()
 r={'before':state(b),'frames':[]}
 for i in range(3):
  m={}
  for role in ('top','wrist'):
   jpg,meta=camd_client.read_jpeg(role);(p/f'{i}_{role}.jpg').write_bytes(jpg);m[role]=meta
  r['frames'].append(m);time.sleep(.35)
 r['after']=state(b);(p/'state.json').write_text(json.dumps(r,indent=2));print(p)
