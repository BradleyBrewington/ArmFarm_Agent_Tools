"""Persist the bounded image-feedback experiment without rewriting prior outcomes."""
import recording,json,time,datetime,math
from pathlib import Path
cur=json.load(open('tools/current_mug_episode.json'));eid=cur['episode']
note='Unsuccessful alignment experiment; no grasp or lift. Initial nominal-clearance probes changed mug image pose; retracted. At measured FK z .198 m fixed jaw visible. Two small joint probes reduced estimated image error but target drift and pitch error invalidate stationary image-Jacobian fit. Returned home open, torque retained.'
r=recording.stop_recording(success=False,notes=note);print(r)
Path('tools/last_mug_stop.json').write_text(json.dumps(r,indent=2))
obs=[]
for p in sorted(Path('tools/mug_observations').glob('*.json')):
 if p.stem.isdigit() and int(p.stem)/1e9>=cur['started']:
  obs.append(dict(time=int(p.stem)/1e9,state=str(p),top=str(p.with_name(p.stem+'_top.jpg')),wrist=str(p.with_name(p.stem+'_wrist.jpg'))))
points=[('1790195873964954130',[625,281],[650,392]),('1790195906142568121',[643,282],[645,389]),('1790195920525383460',[640,302],[637,386])]
measure=[]
for stamp,jaw,target in points:
 s=json.load(open('tools/mug_observations/'+stamp+'.json'));error=[target[i]-jaw[i] for i in range(2)]
 measure.append(dict(state='tools/mug_observations/'+stamp+'.json',top='tools/mug_observations/'+stamp+'_top.jpg',fixed_jaw_tip_px=jaw,handle_opening_target_px=target,error_px=error,error_norm_px=math.hypot(*error),manual_pixel_uncertainty=6,q=s['q'],xyz=s['xyz']))
experiment=dict(episode_id=eid,hypothesis='Small measured motions at verified clearance yield local image-relative correction before descent.',measurements=measure,fit_valid=False,bounded_correction_applied=False,reason='Target moved ~13 px between baseline and second probe; cause unresolved. Pitch changed ~1.4 degrees on coordinated probe, commanded cancellation was imperfect. Initial low probes also changed target pose. No descent based on contaminated fit.',method_deviation='At first pose fixed top-view jaw hidden by camera mount. Opening gripper at clearance identified visible top right jaw as moving; after higher retraction fixed top left jaw visible.',next='First verify stationary target with repeated no-motion snapshots and stable background features. Use clear configuration away from wrist-flex solver bound and monitor measured pitch before fitting local response. Do not infer clearance from FK alone.')
epath=Path('tools/image_alignment_'+eid+'.json');epath.write_text(json.dumps(experiment,indent=2))
p=Path('tools/mug_episode_ledger.json');ledger=json.loads(p.read_text())
if not any(e['id']==eid for e in ledger):ledger.append(dict(id=eid,started=cur['started'],finished=time.time(),success=False,notes=note,observations=obs,experiment_ledger=str(epath),grasp_location_m=None,grasp_location_status='No grasp established; measured probe poses in experiment ledger.',table_contact_location={'status':'Unknown; no coverage credit. Target drift observed; no elevated handle projected onto table.'}))
p.write_text(json.dumps(ledger,indent=2))
s=json.load(open('tools/supervisor_status.json'));now=time.time();s.update(updated_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),successes=sum(e['success'] for e in ledger),unsuccessful_episodes=sum(not e['success'] for e in ledger),verified_successes_trailing_hour=sum(e['success'] and e.get('finished',0)>=now-3600 for e in ledger),latest_episode_id=eid,latest_result=note,experiment_ledger=str(epath),blocker=experiment['reason'],next_hypothesis=experiment['next'],transferable_discovery='Fixed jaw is hidden by top camera mount near downward pitch; top-view visible right jaw is moving. Higher retraction exposes fixed left jaw. Estimated error 114→107→84 px is not verified improvement because target drifted; no valid fit or retained grasp.',turn_boundary='Recorder stopped; four-joint home reached, open gripper and torque retained.',history_reconciliation='Prior four episodes retained by episode_id including uploaded historical failure; appended current unsuccessful experiment.')
s['success_to_failure_ratio']=s['successes']/s['unsuccessful_episodes'];Path('tools/supervisor_status.json').write_text(json.dumps(s,indent=2))
print({k:s[k] for k in ['successes','unsuccessful_episodes','verified_successes_trailing_hour']})
