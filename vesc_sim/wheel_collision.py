"""Select tire-only collision geometry before Play, in the caller's edit layer.

This changes no wheel pose, visual, rigid-body mass/inertia, or ground geometry.
The convex alternative is an inscribed regular prism, not a smooth tire model.
"""
import math


def apply_wheel_angular_limits(stage, root, runtime):
    """Author USD degrees/s from a YAML rad/s limit, on spinning links only.

    PhysX's schema default is about 5729.58 degrees/s (100 rad/s), which is
    only 2.794 m/s at this vehicle's radius. That solver cap is not a tire model.
    """
    from pxr import PhysxSchema
    limit = float(runtime.get('max_wheel_angular_velocity_rad_s', 1000.))
    if not math.isfinite(limit) or limit <= 0:
        raise ValueError('Wheel angular velocity limit must be finite and positive')
    for name in ('FL','FR','RL','RR'):
        wheel = stage.GetPrimAtPath(root+'/Wheel_'+name)
        if not wheel:
            raise ValueError(f'Missing physical wheel {name}')
        PhysxSchema.PhysxRigidBodyAPI.Apply(wheel).CreateMaxAngularVelocityAttr(math.degrees(limit))
    return {'max_wheel_angular_velocity_rad_s': limit}


def cylinder_prism(radius, height, segments):
    """Closed outward-wound Y-axis prism; return points/counts/indices."""
    if not math.isfinite(radius) or not math.isfinite(height) or min(radius, height) <= 0:
        raise ValueError('Cylinder radius and height must be finite and positive')
    if isinstance(segments, bool) or not isinstance(segments, int) or not 8 <= segments <= 127:
        raise ValueError('wheel_collision_segments must be an integer from 8 to 127')
    points = [(radius*math.cos(2*math.pi*i/segments), y,
               radius*math.sin(2*math.pi*i/segments))
              for y in (-height*.5, height*.5) for i in range(segments)]
    indices = []
    for i in range(segments):
        j = (i+1) % segments
        indices.extend((i, i+segments, j+segments, j))
    indices.extend(range(segments))  # lower cap, outward -Y
    indices.extend(i+segments for i in reversed(range(segments)))  # upper +Y
    return points, [4]*segments+[segments, segments], indices


def prepare_wheel_collisions(stage, root, configruntime):
    """Prepare all four tires while stopped; caller owns timeline and edit layer.

    Call only before PhysX attaches the articulation (before Play), never while
    paused with an attached scene. Repeated calls and switching modes are safe.
    No stage layer is saved here.
    """
    from pxr import Gf, UsdGeom, UsdPhysics, UsdShade, PhysxSchema

    mode = configruntime.get('wheel_collision_approximation', 'analytic')
    if mode not in ('analytic', 'convex_hull'):
        raise ValueError('wheel_collision_approximation must be analytic or convex_hull')
    segments = configruntime.get('wheel_collision_segments', 64)
    cylinder_prism(1., 1., segments)  # validate before any scene edits

    def require_identity(prim):
        matrix = UsdGeom.Xformable(prim).GetLocalTransformation()
        if any(abs(float(matrix[i][j])-(1. if i == j else 0.)) > 1e-10
               for i in range(4) for j in range(4)):
            raise ValueError(f'Expected identity tire collision transform: {prim.GetPath()}')

    originals = []
    for name in ('FL', 'FR', 'RL', 'RR'):
        wheel = stage.GetPrimAtPath(root+'/Wheel_'+name)
        original = stage.GetPrimAtPath(str(wheel.GetPath())+'/TireCollision') if wheel else None
        if not original or not original.IsA(UsdGeom.Cylinder):
            raise ValueError(f'Missing Cylinder tire collider for {name}')
        require_identity(original)
        cylinder = UsdGeom.Cylinder(original)
        if cylinder.GetAxisAttr().Get() != 'Y':
            raise ValueError(f'Expected Y-axis tire cylinder for {name}')
        radius, height = float(cylinder.GetRadiusAttr().Get()), float(cylinder.GetHeightAttr().Get())
        points, counts, indices = cylinder_prism(radius, height, segments)
        mass = UsdPhysics.MassAPI(wheel)
        if not mass or not mass.GetMassAttr().HasAuthoredValueOpinion() or not mass.GetDiagonalInertiaAttr().HasAuthoredValueOpinion():
            raise ValueError(f'Explicit wheel mass/inertia required to preserve dynamics: {name}')
        if float(mass.GetMassAttr().Get()) <= 0 or any(float(v) <= 0 for v in mass.GetDiagonalInertiaAttr().Get()):
            raise ValueError(f'Positive wheel mass/inertia required: {name}')
        mesh_path = str(wheel.GetPath())+'/TireCollisionConvex'
        existing = stage.GetPrimAtPath(mesh_path)
        if existing:
            if not existing.IsA(UsdGeom.Mesh):
                raise ValueError(f'Existing convex tire path is not a Mesh: {mesh_path}')
            require_identity(existing)
        originals.append((name, original, mesh_path, radius, height, points, counts, indices))

    angular_limits = apply_wheel_angular_limits(stage, root, configruntime)
    summary = {'mode': mode, 'segments': segments, 'wheels': {}, 'mass_inertia_modified': False,
               **angular_limits}
    for name, original, mesh_path, radius, height, points, counts, indices in originals:
        UsdPhysics.CollisionAPI(original).GetCollisionEnabledAttr().Set(mode == 'analytic')
        mesh_prim = stage.GetPrimAtPath(mesh_path)
        if mode == 'convex_hull':
            mesh = UsdGeom.Mesh.Define(stage, mesh_path)
            mesh_prim = mesh.GetPrim()
            mesh.CreatePointsAttr([Gf.Vec3f(*p) for p in points])
            mesh.CreateFaceVertexCountsAttr(counts)
            mesh.CreateFaceVertexIndicesAttr(indices)
            mesh.CreateSubdivisionSchemeAttr('none')
            mesh.CreateOrientationAttr('rightHanded')
            mesh.CreateExtentAttr([Gf.Vec3f(-radius, -height*.5, -radius),
                                   Gf.Vec3f(radius, height*.5, radius)])
            UsdGeom.Imageable(mesh_prim).CreateVisibilityAttr('invisible')
            UsdPhysics.CollisionAPI.Apply(mesh_prim).CreateCollisionEnabledAttr(True)
            UsdPhysics.MeshCollisionAPI.Apply(mesh_prim).CreateApproximationAttr('convexHull')
            PhysxSchema.PhysxConvexHullCollisionAPI.Apply(mesh_prim).CreateHullVertexLimitAttr(2*segments)
            source = PhysxSchema.PhysxCollisionAPI(original)
            destination = PhysxSchema.PhysxCollisionAPI.Apply(mesh_prim)
            for getter, creator in (('GetContactOffsetAttr', 'CreateContactOffsetAttr'),
                                    ('GetRestOffsetAttr', 'CreateRestOffsetAttr')):
                value = getattr(source, getter)().Get()
                if value is not None:
                    getattr(destination, creator)(value)
            UsdShade.MaterialBindingAPI.Apply(mesh_prim)
            for rel in original.GetRelationships():
                if rel.GetName().startswith('material:binding'):
                    copied = mesh_prim.CreateRelationship(rel.GetName(), custom=False)
                    copied.SetTargets(rel.GetTargets())
                    strength = rel.GetMetadata('bindMaterialAs')
                    if strength is not None:
                        copied.SetMetadata('bindMaterialAs', strength)
        elif mesh_prim:
            UsdPhysics.CollisionAPI(mesh_prim).GetCollisionEnabledAttr().Set(False)
        summary['wheels'][name] = {'original': str(original.GetPath()), 'convex': mesh_path,
            'radius_m': radius, 'height_m': height, 'vertex_count': len(points),
            'max_radial_error_m': radius*(1-math.cos(math.pi/segments)) if mode == 'convex_hull' else 0.}
    return summary
