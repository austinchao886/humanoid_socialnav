#!/usr/bin/env python3
"""Read-only authored USD inventory; does not inspect a live PhysX scene.

Run with Isaac's USD Python bindings. JSON goes to stdout. No stage is saved.
Collision APIs on a transform alone are not proof of a cooked collider.
"""
import argparse
import hashlib
import json
from pathlib import Path


def surface_geometry(stage, prim, rigid_body):
    """Export authored surface, not PhysX convex cooking, in rigid-link coordinates."""
    from pxr import Gf, Usd, UsdGeom

    if rigid_body is None:
        return {"available": False, "reason": "no_rigid_body_owner"}
    cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    body = stage.GetPrimAtPath(rigid_body)
    # Gf uses row vectors: mesh -> world -> rigid link.
    transform = cache.GetLocalToWorldTransform(prim) * cache.GetLocalToWorldTransform(body).GetInverse()
    units = UsdGeom.GetStageMetersPerUnit(stage)
    matrix = [[float(transform[i][j]) for j in range(4)] for i in range(4)]
    for j in range(3):
        matrix[3][j] *= units
    result = dict(available=True, representation="authored_surface_not_cooked_collider",
                  frame="rigid_link_local_m", local_to_link_row_major=matrix,
                  orientation=str(UsdGeom.Gprim(prim).GetOrientationAttr().Get()),
                  transform_determinant=float(transform.GetDeterminant()))
    if prim.IsA(UsdGeom.Mesh):
        mesh = UsdGeom.Mesh(prim)
        points = mesh.GetPointsAttr().Get()
        counts = list(mesh.GetFaceVertexCountsAttr().Get() or [])
        indices = list(mesh.GetFaceVertexIndicesAttr().Get() or [])
        if points is None or sum(counts) != len(indices) or any(
                i < 0 or i >= len(points) for i in indices):
            return {"available": False, "reason": "missing_or_invalid_mesh_topology"}
        result.update(type="Mesh", vertices_link_m=[
            [float(v) * units for v in transform.Transform(Gf.Vec3d(*point))]
            for point in points], face_vertex_counts=counts, face_vertex_indices=indices,
            hole_indices=list(mesh.GetHoleIndicesAttr().Get() or []),
            subdivision_scheme=str(mesh.GetSubdivisionSchemeAttr().Get()))
    elif prim.IsA(UsdGeom.Sphere):
        result.update(type="Sphere", radius_local_m=float(UsdGeom.Sphere(prim).GetRadiusAttr().Get()) * units)
    elif prim.IsA(UsdGeom.Cylinder):
        cylinder = UsdGeom.Cylinder(prim)
        result.update(type="Cylinder", radius_local_m=float(cylinder.GetRadiusAttr().Get()) * units,
                      height_local_m=float(cylinder.GetHeightAttr().Get()) * units,
                      axis=str(cylinder.GetAxisAttr().Get()))
    else:
        return {"available": False, "reason": f"unsupported_surface_type:{prim.GetTypeName()}"}
    return result


def inventory(asset, include_geometry=False):
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
            if include_geometry:
                record["geometry"] = surface_geometry(stage, prim, rigid_body)
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
    parser.add_argument("--include-geometry", action="store_true",
                        help="Export visual and collision authored surfaces in link-local metres; potentially large")
    args = parser.parse_args()
    print(json.dumps(inventory(args.asset, args.include_geometry), indent=2))


if __name__ == "__main__":
    main()
