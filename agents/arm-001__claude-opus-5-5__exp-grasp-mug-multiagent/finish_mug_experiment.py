"""Append this completed controlled experiment, preserving all earlier episodes."""
import json,time,datetime,sys
from pathlib import Path
import recording
note='Handle approach nudged mug; lateral correction then closure reached empty endpoint (goal 0, measured 1.417, load 96). No opposing contact or lift established. Retreated and returned home with open gripper.'
r=recording.stop_recording(success=False,notes=note)
print(r)
Path('tools/last_mug_stop.json').write_text(json.dumps(r,indent=2))
id=json.load(open('tools/current_mug_episode.json'))['episode']
p=Path('tools/mug_episode_ledger.json');ledger=json.loads(p.read_text())
obs=[]
for f in sorted(Path('tools/mug_observations').glob('*.json')):
 if f.stem.isdigit() and int(f.stem)/1e9>=1790195434:
  obs.append(dict(time=int(f.stem)/1e9,state=str(f),top=str(f.with_name(f.stem+'_top.jpg')),wrist=str(f.with_name(f.stem+'_wrist.jpg'))))
entry=dict(id=id,started=1790195434,finished=time.time(),success=False,notes=note,observations=obs,grasp_location_frame='base_link measured FK of attempted contact, not mug footprint',grasp_location_m=[.284615794,.035740912,.046060079],grasp_q=dict(shoulder_pan=-7.5164835,shoulder_lift=17.4065934,elbow_flex=-10.1538462,wrist_flex=71.8681319,wrist_roll=-24.7472527,gripper=1.417004),table_contact_location=dict(status='unknown: contact underside obscured; no coverage credit',pre_attempt_body_footprint_pixel_estimate=[580,385],estimated_table_m=[.03397,.03199],uncertainty_m=.03,method='Approximate body-support location from top view before approach; not elevated handle projection',evidence=obs[0]['top'] if obs else None))
if not any(e['id']==id for e in ledger):ledger.append(entry)
p.write_text(json.dumps(ledger,indent=2))
s=json.load(open('tools/supervisor_status.json'));now=time.time()
s.update(updated_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),successes=sum(e['success'] for e in ledger),unsuccessful_episodes=sum(not e['success'] for e in ledger),verified_successes_trailing_hour=sum(e['success'] and e.get('finished',0)>=now-3600 for e in ledger),latest_episode_id=id,latest_result=note,grid_ledger='tools/mug_table_grid.json',blocker='No opposing contact at measured FK (.284616,.035741,.046060); projected handle alignment was above/outside the jaw contact depth.',transferable_discovery='Wrist-image handle overlap is insufficient: closure measured 1.417 at goal0 and load96, identical to empty endpoint. No lift attempted.',next_hypothesis='Approach the outer handle arc with fixed jaw inside loop at measured depth; use the previously verified more extended elbow configuration as a geometry reference, not an absolute target.',turn_boundary='Recorder stopped; four-joint home reached; mug on table; gripper open; torque retained.')
s['success_to_failure_ratio']=s['successes']/s['unsuccessful_episodes']
s['coverage_proposal']=dict(frame='checkerboard_table',bounds_m={'x':[-.35,.60],'y':[-.30,.30]},cell_size_m=.05,candidate_cells=228,accepted_usable_mask=None,verified_cells=[],untested_or_uncertified_cells='All 228 cells in tools/mug_table_grid.json; geometry, reach and obstacles pending.',status='Broad calibrated visible-table survey envelope, not accepted usable workspace. No coverage claim.')
Path('tools/supervisor_status.json').write_text(json.dumps(s,indent=2))
