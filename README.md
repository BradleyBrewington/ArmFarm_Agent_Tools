# ArmFarm Agent Tools

Current tools written or used by each ArmFarm agent:

```
agents/<agent_id>/
```

Each folder mirrors that agent's tools folder, including edits and deletions.
Commits record the agent ID, designation, model and task metadata. Agent IDs stay
stable when a model or task changes. Git history preserves earlier revisions.

The first station is `protopi5`:
[`af-4b0311b565574e80b3def13b55293597`](agents/af-4b0311b565574e80b3def13b55293597/).

Robot software is experimental. Inspect tools and adapt them to your hardware
before running them. Calibration and motion limits are specific to each arm.

Recordings, conversations, images, credentials and deployment configuration live
outside this repository. A background publisher on each Pi checks once a minute;
GitHub access is provisioned from the operator's laptop with a separate deploy key
for each station. Keys grant access to this repository, not the operator's other
repositories. They are repository-wide, not folder-scoped; the publisher scopes
normal writes to its own agent folder.
