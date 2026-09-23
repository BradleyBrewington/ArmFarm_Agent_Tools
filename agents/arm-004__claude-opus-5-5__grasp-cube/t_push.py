import sys; sys.path.insert(0,'work'); import arm, grasp, run_episodes as R
a=list(map(float,sys.argv[1:5]))
with arm.bus() as b:
    grasp.push_line(b,a[0:2],a[2:4]); arm.home(b); print(R.find_cube())
