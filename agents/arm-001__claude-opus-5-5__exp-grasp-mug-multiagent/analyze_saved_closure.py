"""Read-only saved-frame alignment analysis; no robot commands."""
import json,cv2,numpy as np
from pathlib import Path
root=Path('/var/lib/armfarm/stations/armfarm/episodes/completed/20260923T171728-eb3ce7d2');out=Path('tools/closure_review_20260923T171728-eb3ce7d2')
rows=json.load(open(out/'frames.json'));allrows=[json.loads(x) for x in (root/'frames.jsonl').read_text().splitlines()]
def frame(r,role):
 m=r['images'][role]
 with (root/m['file']).open('rb') as f:f.seek(m['offset']);a=f.read(m['length'])
 return cv2.imdecode(np.frombuffer(a,np.uint8),cv2.IMREAD_GRAYSCALE)
rois={'wrist_upper_handle':('wrist',(315,203,32,29)),'wrist_rim':('wrist',(315,104,231,75)),'wrist_fixed_tip':('wrist',(334,233,15,20)),'top_body_edge':('top',(258,136,29,63)),'top_arm':('top',(302,39,30,49))}
refs={k:frame(rows[0],role)[y:y+h,x:x+w] for k,(role,(x,y,w,h)) in rois.items()};tracks=[]
for row in rows:
 ims={role:frame(row,role) for role in ['top','wrist']};d={'t_wall':row['t_wall'],'frame_index':row['frame_index'],'gripper_measured':row['observation.state'][-1],'gripper_goal':row['action'][-1],'features':{}}
 for k,(role,(x,y,w,h)) in rois.items():
  radius=15;im=ims[role];win=im[y-radius:y+h+radius,x-radius:x+w+radius];res=cv2.matchTemplate(win,refs[k],cv2.TM_CCOEFF_NORMED);_,score,_,loc=cv2.minMaxLoc(res);d['features'][k]={'shift_px':[loc[0]-radius,loc[1]-radius],'score':score}
 tracks.append(d)
summary={}
for k in rois:
 a=np.array([d['features'][k]['shift_px'] for d in tracks]);scores=[d['features'][k]['score'] for d in tracks];summary[k]={'shift_min_px':a.min(0).tolist(),'shift_max_px':a.max(0).tolist(),'range_px':np.ptp(a,axis=0).tolist(),'minimum_score':min(scores),'roi':rois[k][1]}
relative=np.array([np.array(d['features']['wrist_upper_handle']['shift_px'])-np.array(d['features']['wrist_fixed_tip']['shift_px']) for d in tracks]);state=np.array([r['observation.state'] for r in rows]);goals=np.array([r['action'] for r in rows]);times=np.array([r['t_wall'] for r in rows]);start=1790202051.236;end=1790202073.781
r={'episode_id':root.name,'operator_call_interval_epoch':[start,end],'saved_interval_epoch':[float(times[0]),float(times[-1])],'uncovered_call_edges_seconds':[float(times[0]-start),float(end-times[-1])],'frame_count':len(rows),'max_saved_interval_gap_seconds':float(np.diff(times).max()),'frame_dimensions':[640,360],'coordinates':'Recorded640x360 pixels, half the saved standalone1280x720 views.','feature_summary':summary,'handle_relative_to_tip_range_px':np.ptp(relative,axis=0).tolist(),'arm_measured_range_deg':np.ptp(state[:,:5],axis=0).tolist(),'arm_goal_range_deg':np.ptp(goals[:,:5],axis=0).tolist(),'gripper_start_end':[float(state[0,-1]),float(state[-1,-1])],'camera_unique_sequences':{role:len(set(x['images'][role]['seq'] for x in rows)) for role in ['top','wrist']},'camera_unique_hashes':{role:len(set(x['images'][role]['sha256'] for x in rows)) for role in ['top','wrist']},'limitations':'Template shifts measure visible projected features, not 3D insertion. Jaw occlusion can contaminate fixed-tip/handle matching; inspect correlation and representative frames. Unrecorded call-edge intervals are unknown.'}
(out/'tracking.json').write_text(json.dumps(tracks));(out/'result.json').write_text(json.dumps(r,indent=2));print(json.dumps(r,indent=2))
