# Isaac runtime

Owns Social Motion helpers for the authoritative Isaac simulation. The actual
simulator, action provider, and DDS bridge are maintained only in the pinned
`unitree_sim_isaaclab` workspace recorded in `versions.lock.yaml`.

Stale copies of those upstream files were deliberately removed from this
repository. Changes to the simulator must be committed in the Unitree fork and
then pinned here; they must not be copied back as an untracked overlay.
