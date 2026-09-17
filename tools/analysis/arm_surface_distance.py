"""Offline triangle-surface queries, never a controller or safety gate.

FCL triangle contact depth is not a solid minimum-translation depth. Keep it
separate and report solid penetration depth as unavailable, not as zero.
"""
import numpy as np
import fcl
import trimesh


class SurfaceMesh:
    def __init__(self, vertices, faces, name):
        vertices = np.asarray(vertices, dtype=np.float64)
        faces = np.asarray(faces)
        if (vertices.ndim != 2 or vertices.shape[1] != 3 or not len(vertices)
                or not np.isfinite(vertices).all() or faces.ndim != 2
                or faces.shape[1] != 3 or not len(faces)
                or not np.issubdtype(faces.dtype, np.integer)
                or faces.min() < 0 or faces.max() >= len(vertices)):
            raise ValueError("finite vertices and valid nonempty triangle indices required")
        # Merge duplicated vertices for topology queries without simplifying surfaces.
        self.mesh = trimesh.Trimesh(vertices=vertices, faces=faces, process=True)
        if np.any(self.mesh.area_faces <= 1e-18):
            raise ValueError("degenerate triangles need explicit repair before measurement")
        self.name = name
        self.closed = bool(self.mesh.is_watertight and self.mesh.is_winding_consistent)
        self.representatives = np.asarray([
            part.vertices[0] for part in self.mesh.split(only_watertight=False)
        ]) if self.closed else None
        self.geometry = fcl.BVHModel()
        self.geometry.beginModel(len(self.mesh.vertices), len(self.mesh.faces))
        self.geometry.addSubModel(self.mesh.vertices, self.mesh.faces)
        self.geometry.endModel()

    @classmethod
    def from_export(cls, shape):
        geometry = shape["geometry"]
        if (not geometry.get("available") or geometry.get("type") != "Mesh"
                or geometry.get("frame") != "rigid_link_local_m"
                or geometry.get("hole_indices")
                or geometry.get("subdivision_scheme") != "none"
                or any(n != 3 for n in geometry["face_vertex_counts"])):
            raise ValueError("requires explicit unholed non-subdivided triangle surface")
        indices = geometry["face_vertex_indices"]
        if len(indices) != 3 * len(geometry["face_vertex_counts"]):
            raise ValueError("triangle index count mismatch")
        return cls(geometry["vertices_link_m"], np.asarray(indices).reshape(-1, 3), shape["path"])


def rigid_pose(matrix):
    matrix = np.asarray(matrix, dtype=np.float64)
    if (matrix.shape != (4, 4) or not np.isfinite(matrix).all()
            or not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-9, rtol=0)
            or not np.allclose(matrix[:3, :3].T @ matrix[:3, :3], np.eye(3), atol=1e-6, rtol=0)
            or not np.isclose(np.linalg.det(matrix[:3, :3]), 1, atol=1e-6, rtol=0)):
        raise ValueError("pose must be a finite column-vector rigid transform")
    return matrix


def contained(inner, outer, inner_pose, outer_pose):
    relative = np.linalg.inv(outer_pose) @ inner_pose
    points = trimesh.transform_points(inner.representatives, relative)
    return bool(outer.mesh.contains(points).any())


def query(a, b, pose_a, pose_b, tolerance_m=1e-7):
    if not np.isfinite(tolerance_m) or tolerance_m <= 0:
        raise ValueError("positive finite numerical tolerance required")
    pa, pb = rigid_pose(pose_a), rigid_pose(pose_b)
    oa = fcl.CollisionObject(a.geometry, fcl.Transform(pa[:3, :3], pa[:3, 3]))
    ob = fcl.CollisionObject(b.geometry, fcl.Transform(pb[:3, :3], pb[:3, 3]))
    collision = fcl.CollisionResult()
    fcl.collide(oa, ob, fcl.CollisionRequest(num_max_contacts=64, enable_contact=True), collision)
    result = dict(schema_version=1, shape_a=a.name, shape_b=b.name,
        scope="authored_triangle_surfaces_not_physx_or_hardware",
        numerical_tolerance_m=tolerance_m, safety_margin_m=None,
        closed_meshes=[a.closed, b.closed],
        penetration_depth_m=None,
        penetration_depth_reason="solid_minimum_translation_depth_not_computed",
        continuous_collision_checked=False, safe=False)
    if collision.is_collision:
        result.update(classification="surface_contact_or_intersection",
            surface_distance_m=0.0,
            triangle_contact_depths_m=[float(c.penetration_depth) for c in collision.contacts],
            contacts_truncated_or_at_limit=len(collision.contacts) >= 64)
        return result
    distance = fcl.DistanceResult()
    value = float(fcl.distance(oa, ob, fcl.DistanceRequest(enable_nearest_points=True), distance))
    if not np.isfinite(value) or value < 0:
        raise ValueError("FCL returned invalid unsigned surface distance")
    classification = "near_surface_contact" if value <= tolerance_m else "surface_separated_solid_unknown"
    if value > tolerance_m and a.closed and b.closed:
        classification = "solid_overlap_containment" if (
            contained(a, b, pa, pb) or contained(b, a, pb, pa)
        ) else "separated_closed_surfaces"
    result.update(classification=classification, surface_distance_m=value,
        nearest_points_world_m=[np.asarray(point).tolist() for point in distance.nearest_points])
    return result
