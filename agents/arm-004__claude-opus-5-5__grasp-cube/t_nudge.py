import sys, math; sys.path.insert(0,'tools'); import arm, grasp, run_episodes as R
dr=float(sys.argv[1]); dt=float(sys.argv[2]) if len(sys.argv)>2 else 0.0
with arm.bus() as b:
    f=R.find_cube(b); cx,cy,yaw,_=f
    a=math.atan2(cy,cx-0.0388)
    cx2=cx+dr*math.cos(a)-dt*math.sin(a); cy2=cy+dr*math.sin(a)+dt*math.cos(a)
    g=grasp.grasp(b,cx2,cy2,yaw); print(dr,dt,g)
    if not g['ok']: R.lift_clear(b); arm.home(b)
