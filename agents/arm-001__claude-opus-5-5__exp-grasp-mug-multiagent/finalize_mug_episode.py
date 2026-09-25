"""Finalize the active mug episode with explicit reviewed outcome and append-only history."""
import sys,json,time,datetime
from pathlib import Path
import recording
report=json.load(open(sys.argv[1]));cur=json.load(open('tools/current_mug_episode.json'));eid=cur['episode']
p=Path('tools/mug_episode_ledger.json');ledger=json.loads(p.read_text())
if any(e['id']==eid for e in ledger):raise RuntimeError('Episode already recorded; refuse duplicate finalization')
if not isinstance(report['success'],bool):raise TypeError('Explicit boolean outcome required')
r=recording.stop_recording(success=report['success'],notes=report['notes']);print(r)
if r['episode']!=eid:raise RuntimeError('Recorder episode does not match expected episode')
now=time.time();obs=[]
for f in sorted(Path('tools/mug_observations').glob('*.json')):
 if f.stem.isdigit() and cur['started']<=int(f.stem)/1e9<=now:
  obs.append(dict(time=int(f.stem)/1e9,state=str(f),top=str(f.with_name(f.stem+'_top.jpg')),wrist=str(f.with_name(f.stem+'_wrist.jpg'))))
entry=dict(id=eid,started=cur['started'],finished=now,success=report['success'],notes=report['notes'],observations=obs,report=sys.argv[1],quality_flags=r.get('quality_flags',[]),training_ready=r.get('training_ready'),**report.get('episode_details',{}));ledger.append(entry);p.write_text(json.dumps(ledger,indent=2))
Path('tools/last_mug_stop.json').write_text(json.dumps(r,indent=2))
s=json.load(open('tools/supervisor_status.json'));s.update(report['status']);s.update(updated_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),successes=sum(e['success'] for e in ledger),unsuccessful_episodes=sum(not e['success'] for e in ledger),verified_successes_trailing_hour=sum(e['success'] and e.get('finished',0)>=now-3600 for e in ledger),latest_episode_id=eid,latest_result=report['notes'],experiment_ledger=sys.argv[1],history_reconciliation='Prior history retained by episode_id, including uploaded records. Appended one completed episode; no denominator reset.')
s['success_to_failure_ratio']=s['successes']/s['unsuccessful_episodes'] if s['unsuccessful_episodes'] else None
Path('tools/supervisor_status.json').write_text(json.dumps(s,indent=2));print({k:s[k] for k in ['successes','unsuccessful_episodes','verified_successes_trailing_hour']})
