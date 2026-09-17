#!/usr/bin/env python3
"""Read-only authored USD inventory; does not inspect a live PhysX scene.

Run with Isaac's USD Python bindings. JSON goes to stdout. No stage is saved.
Collision APIs on a transform alone are not proof of a cooked collider.
"""
import argparse
import hashlib
import json
from pathlib import Path


def inventory(asset):
    from pxr import Usd, UsdGeom, UsdPhysics

    stage = Usd.Stage.Open(str(asset))
    if stage is None:
        raise ValueError(f"Cannot open USD: {asset}")
    predicate = Usd.TraverseInstanceProxies()
    prims = list(Usd.PrimRange(stage.GetPseudoRoot(), predicate))
    shapes = []
    colliders = []
    joints = []
    filters = []
    articulations = []
    for prim in prims:
        path = str(prim.GetPath())
        if prim.IsA(UsdGeom.Gprim):
            owner = prim
            rigid_body = None
            collision_ancestors = []
            while owner and not owner.IsPseudoRoot():
                if owner.HasAPI(UsdPhysics.CollisionAPI):
                    collision_ancestors.append(str(owner.GetPath()))
                if rigid_body is None and owner.HasAPI(UsdPhysics.RigidBodyAPI):
                    rigid_body = str(owner.GetPath())
                owner = owner.GetParent()
            record = dict(path=path, type=str(prim.GetTypeName()),
                          rigid_body=rigid_body,
                          collision_api_ancestors=collision_ancestors,
                          instance_proxy=prim.IsInstanceProxy())
            if prim.IsA(UsdGeom.Mesh):
                mesh = UsdGeom.Mesh(prim)
                record.update(point_count=len(mesh.GetPointsAttr().Get() or []),
                              face_count=len(mesh.GetFaceVertexCountsAttr().Get() or []))
            shapes.append(record)
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            children = [str(p.GetPath()) for p in Usd.PrimRange(prim, predicate)
                        if p.IsA(UsdGeom.Gprim)]
            colliders.append(dict(path=path, type=str(prim.GetTypeName()),
                enabled=UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get(),
                approximation=prim.GetAttribute("physics:approximation").Get(),
                geometry_descendants=children,
                missing_geometry_descendants=not children,
                instance=prim.IsInstance()))
        if prim.IsA(UsdPhysics.Joint):
            joint = UsdPhysics.Joint(prim)
            joints.append(dict(path=path,
                body0=[str(p) for p in joint.GetBody0Rel().GetTargets()],
                body1=[str(p) for p in joint.GetBody1Rel().GetTargets()],
                collision_enabled=joint.GetCollisionEnabledAttr().Get()))
        for relationship in prim.GetRelationships():
            if "filtered" in relationship.GetName().lower():
                filters.append(dict(path=path, relationship=relationship.GetName(),
                    targets=[str(p) for p in relationship.GetTargets()]))
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            articulations.append(dict(path=path,
                authored_self_collision=prim.GetAttribute(
                    "physxArticulation:enabledSelfCollisions").Get()))
    layers = []
    for layer in stage.GetUsedLayers():
        filename = Path(layer.realPath) if layer.realPath else None
        layers.append(dict(identifier=layer.identifier,
            sha256=hashlib.sha256(filename.read_bytes()).hexdigest()
            if filename and filename.is_file() else None))
    return dict(schema_version=1, scope="authored_usd_not_live_physx",
        asset=str(asset), meters_per_unit=UsdGeom.GetStageMetersPerUnit(stage),
        up_axis=str(UsdGeom.GetStageUpAxis(stage)), layers=layers,
        shapes=shapes, collision_apis=colliders, joints=joints,
        filtered_relationships=filters, articulations=articulations,
        limitations=["No surface-distance or penetration measurement yet.",
            "API inheritance is inventoried, not assumed to create a PhysX shape.",
            "Runtime overrides and cooked geometry require separate verification.",
            "Joint adjacency is evidence for review, not an automatic pair exclusion."])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("asset", type=Path)
    args = parser.parse_args()
    print(json.dumps(inventory(args.asset), indent=2))


if __name__ == "__main__":
    main()
