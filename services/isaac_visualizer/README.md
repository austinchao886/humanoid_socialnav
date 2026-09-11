# Isaac visualizer

The visualizer is explicitly non-authoritative. It renders snapshots written by
the Isaac runtime and never publishes LowCmd or determines execution safety.
The renderer implementation currently lives in the pinned
`unitree_sim_isaaclab` workspace; this directory records service ownership and
keeps that distinction visible in the integration repository.
