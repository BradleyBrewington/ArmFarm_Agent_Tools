import sys; sys.path.insert(0,'work'); import arm, run_episodes as R
with arm.bus() as b:
    R.lift_clear(b); arm.home(b); print(R.reachable_prep(b))
