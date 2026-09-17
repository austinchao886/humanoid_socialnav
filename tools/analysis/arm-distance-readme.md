# Offline arm surface queries (experimental)

`arm_geometry_inventory.py --include-geometry ASSET.usd` exports authored visual
and collision surfaces in rigid-link-local metres. `arm_surface_distance.py`
loads explicit triangle meshes and queries two column-vector rigid world poses.
Install `requirements-arm-distance.txt` only in an isolated offline environment.
Neither tool sends commands or changes a live scene.

The query reports unsigned **surface** distance, world nearest points, surface
contact/intersection, or containment. Containment is checked per connected
component only when both meshes are watertight and winding-consistent. Open or
inconsistent meshes cannot establish solid separation. No automatic hole fill,
simplification, convex replacement, or repair is performed; Trimesh's normal
vertex-processing step merges repeated vertices for topology analysis.

All results deliberately have `safe=false`: no physical safety margin has been
established. This field means "not qualified safe", not "a collision is proven".
The numerical tolerance is not a safety clearance. Mesh triangle contact depths
are not a solid minimum translation distance; `penetration_depth_m` stays null.
Self-intersection validity, continuous collision, live cooked colliders, planner
reference synchronization and measured robot-shell tolerances are not verified.
Thus this module is a measurement component, **not a completed collision monitor**.

Seven offline tests cover known box separation/contact/crossing, full containment,
disconnected components, open topology, invalid transforms and world-frame nearest
points. These tests do not qualify G1 locomotion.

Actual deployment visual meshes loaded successfully: pelvis contour 36,102
triangles; torso 51,410; left elbow 1,774; left hand base 28,960. Only pelvis passed
the combined watertight-plus-consistent-winding condition in this check. The other
three need topology investigation before solid containment can be trusted. No
claim about their real robot shells follows from this exported USD check.

Measurement provenance: authored USD SHA-256
`86047174b87b4df485e996232fb4d2ece5901a9bcd6f9a54b78f961ce664730e`;
exporter main commit `27d14bf45179e5ccd3aeecb9fa8bae76815bfcff`;
simulator observation commit `39ac1c37f00b8a74bc4f21f3ad1f389a18d6ac65`.
No controller checkpoint participates in these offline geometry fixtures. No live
trace was enabled, no image rebuilt and no automatic or manual motion initiated
by this analysis batch.

Reference API: https://github.com/BerkeleyAutomation/python-fcl
