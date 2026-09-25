"""Read-only paired camera and encoder evidence during a stationary hold."""
import json,time
from pathlib import Path
from camd_client import read_jpeg
from recording import request
out=Path(__file__).parent/('supervisor_hold_'+str(time.time_ns()));out.mkdir()
rows=[];start=time.monotonic()
for i in range(26):
 row={'time':time.time(),'elapsed':time.monotonic()-start,'status':request('status')}
 for role in ('top','wrist'):
  im,meta=read_jpeg(role);(out/f'{i:03}_{role}.jpg').write_bytes(im);row[role]=meta
 rows.append(row)
 time.sleep(.2)
(out/'evidence.json').write_text(json.dumps(rows,indent=2))
print(json.dumps({'folder':str(out),'duration_s':rows[-1]['elapsed'],'motor_writes':False,'recording':rows[-1]['status']['recording']}))
