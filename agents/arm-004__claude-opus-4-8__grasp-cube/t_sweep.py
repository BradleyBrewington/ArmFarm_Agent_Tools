import sys; sys.path.insert(0,'work'); import arm, grasp, run_episodes as R
r,a0,a1=map(float,sys.argv[1:4])
with arm.bus() as b:
    grasp.sweep_arc(b,r,a0,a1); arm.home(b); print(R.find_cube())
