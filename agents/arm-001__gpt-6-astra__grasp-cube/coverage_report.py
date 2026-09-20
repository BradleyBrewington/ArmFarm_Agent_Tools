"""Summarize observed episodes without changing their labels."""
import json,time
from pathlib import Path
from datetime import datetime,timezone
root=Path('runs/20260918_cube_episodes')
collection_start=datetime.strptime(root.name[:8],'%Y%m%d').replace(tzinfo=timezone.utc).timestamp()
episodes=[]
for p in sorted(Path('../episodes/completed').glob('*/episode.json')):
 d=json.loads(p.read_text())
 if d.get('task_id')!='grasp-cube' or d.get('started',0)<collection_start:continue
 row={k:d.get(k) for k in ('id','task','started','finished','success','training_ready','quality_flags','notes')}
 row['folder']=str(p.parent.resolve())
 if d['task'].startswith('Pick black cube from '):
  name=d['task'].split('Pick black cube from ',1)[1].split(';',1)[0]
  f=root/(name+'_start.json')
  if f.exists():row['start_observation']=json.loads(f.read_text())
 episodes.append(row)
successes=sum(d['success'] is True for d in episodes)
failures=sum(d['success'] is False for d in episodes)
now=time.time();elapsed=(now-min(d['started'] for d in episodes))/3600 if episodes else 0
report={'updated':now,'successes':successes,'failures':failures,'elapsed_hours_since_first_episode':elapsed,'observed_successes_per_hour':successes/elapsed if elapsed else 0,'coverage_note':'Sampled positions only. Full workspace boundary, table height, and robot-camera alignment have not been calibrated. XYZ values are FK estimates, not measured table coordinates.','episodes':episodes}
(root/'coverage.json').write_text(json.dumps(report,indent=2))
print(json.dumps({k:v for k,v in report.items() if k!='episodes'},indent=2))
