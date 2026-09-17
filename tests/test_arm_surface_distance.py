import importlib.util
from pathlib import Path
import unittest
from unittest.mock import patch

try:
    import numpy as np
    import fcl
    import trimesh
    import scipy
    import rtree
except ImportError as exc:
    raise unittest.SkipTest("requires isolated offline geometry dependencies") from exc

path = Path(__file__).resolve().parents[1] / "tools/analysis/arm_surface_distance.py"
spec = importlib.util.spec_from_file_location("arm_surface_distance", path)
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)


def cube(size, name):
    mesh = trimesh.creation.box(extents=[size] * 3)
    return m.SurfaceMesh(mesh.vertices, mesh.faces, name)


class SurfaceDistanceTests(unittest.TestCase):
    def test_topology_never_repairs_holes(self):
        mesh = trimesh.creation.box()
        mesh.update_faces(np.arange(len(mesh.faces) - 1))
        original_faces = mesh.faces.copy()
        with patch.object(trimesh.Trimesh, "fill_holes", side_effect=AssertionError("repair forbidden")):
            report = m.topology(mesh)
            closed = cube(1, "closed")
            self.assertEqual(len(closed.representatives), 1)
        self.assertEqual(report["boundary_edges"], 3)
        self.assertFalse(report["watertight"])
        self.assertTrue(np.array_equal(mesh.faces, original_faces))

    def test_separation_and_world_points(self):
        a, b = cube(1, "a"), cube(1, "b")
        pose = np.eye(4)
        pose[0, 3] = 1.3
        result = m.query(a, b, np.eye(4), pose)
        self.assertAlmostEqual(result["surface_distance_m"], .3)
        self.assertEqual(result["classification"], "separated_closed_surfaces")
        self.assertFalse(result["safe"])
        self.assertAlmostEqual(np.linalg.norm(np.diff(result["nearest_points_world_m"], axis=0)), .3)

    def test_touch_and_crossing_never_report_separated(self):
        for x in (1., .8):
            pose = np.eye(4)
            pose[0, 3] = x
            result = m.query(cube(1, "a"), cube(1, "b"), np.eye(4), pose)
            self.assertIn(result["classification"], ("surface_contact_or_intersection", "near_surface_contact"))
            self.assertIsNone(result["penetration_depth_m"])

    def test_containment_is_not_missed_by_triangle_contact(self):
        result = m.query(cube(.2, "inner"), cube(2, "outer"), np.eye(4), np.eye(4))
        self.assertEqual(result["classification"], "solid_overlap_containment")
        self.assertAlmostEqual(result["surface_distance_m"], .9)
        self.assertFalse(result["safe"])

    def test_invalid_pose_rejected(self):
        matrix = np.eye(4)
        matrix[0, 0] = 2
        with self.assertRaises(ValueError):
            m.rigid_pose(matrix)

    def test_open_surface_never_claims_solid_separation(self):
        mesh = trimesh.creation.box()
        a = m.SurfaceMesh(mesh.vertices, mesh.faces[:-1], "open")
        pose = np.eye(4)
        pose[0, 3] = 3
        result = m.query(a, cube(1, "b"), np.eye(4), pose)
        self.assertEqual(result["classification"], "surface_separated_solid_unknown")

    def test_disconnected_component_containment(self):
        first = trimesh.creation.box(extents=[.2] * 3)
        second = first.copy()
        second.apply_translation([5, 0, 0])
        mesh = trimesh.util.concatenate([first, second])
        a = m.SurfaceMesh(mesh.vertices, mesh.faces, "two_components")
        self.assertEqual(m.query(a, cube(2, "body"), np.eye(4), np.eye(4))[
            "classification"], "solid_overlap_containment")

    def test_rotated_translated_nearest_points_lie_on_world_surfaces(self):
        a, b = cube(1, "a"), cube(1, "b")
        pa = trimesh.transformations.rotation_matrix(.6, [0, 0, 1])
        pb = trimesh.transformations.rotation_matrix(.3, [0, 1, 0])
        pa[:3, 3] = [2, 3, 4]
        pb[:3, 3] = [4, 3, 4]
        result = m.query(a, b, pa, pb)
        for mesh, pose, point in zip((a.mesh, b.mesh), (pa, pb), result["nearest_points_world_m"]):
            world = mesh.copy().apply_transform(pose)
            _, distances, _ = trimesh.proximity.closest_point_naive(world, [point])
            self.assertLess(distances[0], 1e-7)


if __name__ == "__main__":
    unittest.main()
