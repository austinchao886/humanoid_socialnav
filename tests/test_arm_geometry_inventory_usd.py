"""Offline USD tests. No simulator application or control transport is started."""
import importlib.util
from pathlib import Path
import tempfile
import unittest

try:
    from pxr import Gf, Usd, UsdGeom, UsdPhysics
except ImportError as exc:
    raise unittest.SkipTest("Requires Isaac USD Python bindings") from exc

MODULE = Path(__file__).resolve().parents[1] / "tools/analysis/arm_geometry_inventory.py"
spec = importlib.util.spec_from_file_location("arm_geometry_inventory", MODULE)
audit = importlib.util.module_from_spec(spec)
spec.loader.exec_module(audit)


class InventoryTests(unittest.TestCase):
    def test_geometry_is_in_link_frame_with_stage_units(self):
        stage = Usd.Stage.CreateInMemory()
        UsdGeom.SetStageMetersPerUnit(stage, 0.01)
        body = UsdGeom.Xform.Define(stage, "/link")
        body.AddTranslateOp().Set((100., 200., 300.))
        body.AddRotateZOp().Set(90.)
        UsdPhysics.RigidBodyAPI.Apply(body.GetPrim())
        mesh = UsdGeom.Mesh.Define(stage, "/link/surface")
        mesh.AddTranslateOp().Set((10., 0., 0.))
        mesh.CreatePointsAttr([(0., 0., 0.), (10., 0., 0.), (0., 10., 0.)])
        mesh.CreateFaceVertexCountsAttr([3])
        mesh.CreateFaceVertexIndicesAttr([0, 1, 2])
        result = audit.surface_geometry(stage, mesh.GetPrim(), "/link")
        for got, expected in zip(result["vertices_link_m"],
                                 [(0.1, 0., 0.), (0.2, 0., 0.), (0.1, 0.1, 0.)]):
            for a, b in zip(got, expected):
                self.assertAlmostEqual(a, b)
        self.assertEqual(result["face_vertex_indices"], [0, 1, 2])

    def test_primitive_affine_scale_is_retained(self):
        stage = Usd.Stage.CreateInMemory()
        UsdGeom.SetStageMetersPerUnit(stage, 1.)
        UsdPhysics.RigidBodyAPI.Apply(UsdGeom.Xform.Define(stage, "/link").GetPrim())
        sphere = UsdGeom.Sphere.Define(stage, "/link/sphere")
        sphere.CreateRadiusAttr(0.1)
        sphere.AddScaleOp().Set((2., 1., 1.))
        result = audit.surface_geometry(stage, sphere.GetPrim(), "/link")
        self.assertEqual(result["radius_local_m"], 0.1)
        self.assertEqual(result["local_to_link_row_major"][0][0], 2.)
        self.assertFalse(audit.surface_geometry(stage, sphere.GetPrim(), None)["available"])

    def inspect(self, stage):
        with tempfile.TemporaryDirectory() as directory:
            filename = Path(directory) / "fixture.usda"
            stage.GetRootLayer().Export(str(filename))
            return audit.inventory(filename)

    def test_instance_geometry_is_not_missing(self):
        stage = Usd.Stage.CreateInMemory()
        UsdGeom.Xform.Define(stage, "/template")
        UsdGeom.Cube.Define(stage, "/template/surface")
        instance = UsdGeom.Xform.Define(stage, "/robot/collision").GetPrim()
        instance.GetReferences().AddInternalReference("/template")
        instance.SetInstanceable(True)
        UsdPhysics.CollisionAPI.Apply(instance)
        result = self.inspect(stage)
        collider = result["collision_apis"][0]
        self.assertTrue(collider["instance"])
        self.assertFalse(collider["missing_geometry_descendants"])
        self.assertEqual(collider["geometry_descendants"], ["/robot/collision/surface"])
        self.assertTrue(next(s for s in result["shapes"]
                             if s["path"] == "/robot/collision/surface")["instance_proxy"])

    def test_empty_collision_is_not_surface(self):
        stage = Usd.Stage.CreateInMemory()
        prim = UsdGeom.Xform.Define(stage, "/empty").GetPrim()
        UsdPhysics.CollisionAPI.Apply(prim)
        result = self.inspect(stage)
        self.assertTrue(result["collision_apis"][0]["missing_geometry_descendants"])
        self.assertEqual(result["shapes"], [])

    def test_joint_and_explicit_filter_are_separate(self):
        stage = Usd.Stage.CreateInMemory()
        for path in ("/a", "/b"):
            UsdPhysics.RigidBodyAPI.Apply(UsdGeom.Xform.Define(stage, path).GetPrim())
        joint = UsdPhysics.RevoluteJoint.Define(stage, "/joint")
        joint.CreateBody0Rel().SetTargets(["/a"])
        joint.CreateBody1Rel().SetTargets(["/b"])
        joint.CreateCollisionEnabledAttr(False)
        UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath("/a")).CreateFilteredPairsRel().SetTargets(["/b"])
        result = self.inspect(stage)
        self.assertEqual(result["joints"][0]["body1"], ["/b"])
        self.assertFalse(result["joints"][0]["collision_enabled"])
        self.assertEqual(result["filtered_relationships"][0]["targets"], ["/b"])
        self.assertEqual(result["scope"], "authored_usd_not_live_physx")
        hashes = [layer["sha256"] for layer in result["layers"] if layer["sha256"]]
        self.assertEqual(len(hashes), 1)
        self.assertEqual(len(hashes[0]), 64)


if __name__ == "__main__":
    unittest.main()
