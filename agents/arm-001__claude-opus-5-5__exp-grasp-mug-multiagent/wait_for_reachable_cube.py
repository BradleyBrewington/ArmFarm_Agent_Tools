"""Poll the top camera and exit(0) when the black cube enters the reachable zone.

Reachable zone: top-camera pixel y <= REACH_Y (proven rest positions are y<=286;
far/unreachable cube sits at y~410). Requires 2 consecutive stable reads so we
resume only after the operator has finished repositioning. Read-only: never
touches torque or Goal_Position. Exits 0 (reachable, prints position), or 2
(timeout). Re-invokes the agent on exit.
"""
import sys, time, json
sys.path.insert(0, 'tools')
import camd_client, cv2, numpy as np

REACH_Y = 288          # cube centroid y at/above this = reachable band
POLL_S = 45
MAX_MIN = 90
stable_need = 2


def cube_xy():
    jpg, _ = camd_client.read_jpeg('top')
    arr = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
    g = cv2.cvtColor(arr, cv2.COLOR_BGR2GRAY)
    m = (g < 60).astype(np.uint8) * 255
    m[:210, :] = 0; m[:, :120] = 0; m[:, 1150:] = 0
    n, lab, st, ce = cv2.connectedComponentsWithStats(m, 8)
    best = None
    for i in range(1, n):
        a = st[i, cv2.CC_STAT_AREA]
        if a > 3000 and (best is None or a > best[2]):
            best = (float(ce[i][0]), float(ce[i][1]), int(a))
    return best


deadline = time.time() + MAX_MIN * 60
stable = 0
last = None
while time.time() < deadline:
    try:
        c = cube_xy()
    except Exception as e:
        print(json.dumps({'t': time.time(), 'err': str(e)}), flush=True)
        time.sleep(POLL_S); continue
    if c is None:
        print(json.dumps({'t': time.time(), 'cube': None, 'note': 'no cube detected (removed/held)'}), flush=True)
        stable = 0
    else:
        x, y, a = c
        reachable = y <= REACH_Y
        print(json.dumps({'t': time.time(), 'cube_px': [round(x), round(y)], 'area': a, 'reachable': reachable}), flush=True)
        if reachable:
            stable += 1
            if stable >= stable_need:
                print(json.dumps({'RESULT': 'REACHABLE', 'cube_px': [round(x), round(y)]}), flush=True)
                sys.exit(0)
        else:
            stable = 0
        last = c
    time.sleep(POLL_S)
print(json.dumps({'RESULT': 'TIMEOUT', 'last': last}), flush=True)
sys.exit(2)
