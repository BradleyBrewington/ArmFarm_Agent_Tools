"""Read-only local fit of manually annotated fixed fingertip; no motor writes."""
import json
from pathlib import Path
import numpy as np,cv2
from scipy.optimize import least_squares
from check_saved_alignment import arm_transform
R=Path(__file__).resolve().parent
samples=[('1790195792546453841',[780,337]),('1790195817511762893',[750,414]),('1790195843405510262',[750,472]),('1790195869971917509',[748,497]),('1790195915611854702',[757,499]),('1790195939506835459',[779,527]),('1790195962961146620',[747,539]),('1790196030563028959',[730,558]),('1790196142505385493',[375,270])]
c=json.loads(Path('calibration_checks/live-table-20260922/table.json').read_text());C=np.array(c['camera_from_base']);profile=json.loads(Path(c['intrinsics_source']).read_text())['cameras']['top'];K=np.array(profile['camera_matrix']);D=np.array(profile['dist_coeffs']);A=[C@arm_transform(json.loads((R/'mug_observations'/f'{s}.json').read_text())['joints']) for s,p in samples]
def project(t,v):
 q=(t@np.r_[v,1])[:3];return cv2.projectPoints(q.reshape(1,3),np.zeros(3),np.zeros(3),K,D)[0].ravel()
def residual(v,indices):return np.concatenate([project(A[i],v)-samples[i][1] for i in indices])
train=[0,1,2,3,5,6,8];fit=least_squares(lambda v:residual(v,train),c['tool_offset']);rows=[]
for i,(stem,px) in enumerate(samples):rows.append(dict(stem=stem,fixed_tip_px=px,predicted=project(A[i],fit.x).tolist(),error_px=float(np.linalg.norm(project(A[i],fit.x)-px)),held_out=i not in train))
r=dict(reference='Fixed fingertip distal tip manually identified from top image at roll~-25; wrist fixed finger is right, moving finger left.',candidate_offset=fit.x.tolist(),original_midpoint_offset=c['tool_offset'],training_rms_px=float(np.sqrt(np.mean(residual(fit.x,train)**2))),samples=rows,annotation_uncertainty_px=5,accepted_calibration_modified=False,motor_writes=False,caveat='Local diagnostic only, tip vs pad distinct. Saved camera alignment uncertainty remains; no contact-depth certification or roll63 validation.')
(R/'fixed_tip_diagnostic.json').write_text(json.dumps(r,indent=2));print(json.dumps(r,indent=2))
