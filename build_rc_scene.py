"""Build a self-contained RC-car USD inside Isaac Sim/Kit (no SimulationApp launch).

Example in Isaac Sim's Script Editor::

    exec(open('/path/build_rc_scene.py').read())
    build_scene('/path/car.obj', '/home/shin/Desktop/RC_Car_Isaac/rc_car_scene.usda',
                visual_transform={'scale': .001, 'axes': ['x','y','z'],
                                  'origin': [0,0,0], 'translation': [0,0,0]})

Visual transform: subtract ``origin`` in source OBJ coordinates; permute/flip
with ``axes`` (for example ['x','-z','y']); multiply by ``scale``; add
``translation`` in meters. The result is in vehicle coordinates: X forward,
Y left, Z up, origin at the nominal whole-vehicle COM. The chassis is initially
placed at world Z=.070m plus ``spawn_lift``. All mesh vertices and colors are
authored into USD; no OBJ or MTL dependency remains. MTL textures are intentionally
not referenced, so the result is portable. Only Kd/d/Tr colors are imported.

INI values: total 3.5kg; COM height .070m; yaw inertia .04712kgm2; axles
X=.15/-.10m; front/rear wheel-center tracks .20/.23m; gravity9.81; friction.9.
User dimensions: tire outside diameter.05588m (2.2in), width.040m.
Consequently tire outside width is .24/.27m, not the INI planner width.22m.

Assumptions: four .075kg homogeneous-cylinder tires and four .025kg carriers;
3.1kg chassis; 20kN/m,200N*s/m suspension,3mm travel; transverse chassis
inertias estimated from the visual bounding-box aspect ratio. Eight joints:
four Z-prismatic springs and four freely rotating Y-revolute wheel axles.
No steering actuator, ROS, actual sensors, tire Magic Formula, or rolling-drag
model is configured. Sensor-shaped CAD components remain ordinary visuals.
"""

from __future__ import annotations

import json
import math
from pathlib import Path


def _read_mtl(path):
    materials = {}
    current = None
    try:
        lines = path.read_text(encoding='utf-8', errors='replace').splitlines()
    except OSError:
        return materials
    for line in lines:
        fields = line.strip().split()
        if not fields or fields[0].startswith('#'):
            continue
        if fields[0] == 'newmtl':
            current = ' '.join(fields[1:])
            materials[current] = {'color': (.55, .57, .60), 'opacity': 1.0}
        elif current and fields[0] == 'Kd' and len(fields) >= 4:
            materials[current]['color'] = tuple(float(v) for v in fields[1:4])
        elif current and fields[0] in ('d', 'Tr') and len(fields) >= 2:
            value = float(fields[1])
            materials[current]['opacity'] = value if fields[0] == 'd' else 1-value
    return materials


def read_obj(path):
    """Preserve OBJ object/group/material boundaries and polygon face normals."""
    path = Path(path)
    vertices, normals, chunks, materials = [], [], [], {}
    object_name, material_name = 'CAD', 'default'
    chunk = None
    with path.open(encoding='utf-8', errors='replace') as stream:
        for line in stream:
            fields = line.strip().split()
            if not fields or fields[0].startswith('#'):
                continue
            key = fields[0]
            if key == 'v':
                vertices.append(tuple(float(x) for x in fields[1:4]))
            elif key == 'vn':
                normals.append(tuple(float(x) for x in fields[1:4]))
            elif key in ('o', 'g'):
                object_name, chunk = ' '.join(fields[1:]) or 'CAD', None
            elif key == 'usemtl':
                material_name, chunk = ' '.join(fields[1:]), None
            elif key == 'mtllib':
                # Fusion normally writes a single sibling MTL, possibly with spaces.
                sibling = path.parent / ' '.join(fields[1:])
                if sibling.exists():
                    materials.update(_read_mtl(sibling))
                else:
                    for name in fields[1:]:
                        materials.update(_read_mtl(path.parent / name))
            elif key == 'f':
                if chunk is None:
                    chunk = {'name': object_name, 'material': material_name, 'faces': []}
                    chunks.append(chunk)
                face = []
                for item in fields[1:]:
                    idx = item.split('/')
                    vi = int(idx[0])
                    vi = vi-1 if vi > 0 else len(vertices)+vi
                    ni = int(idx[2]) if len(idx) > 2 and idx[2] else None
                    if ni is not None:
                        ni = ni-1 if ni > 0 else len(normals)+ni
                    if not 0 <= vi < len(vertices):
                        raise ValueError(f'Invalid OBJ vertex index {item!r}')
                    face.append((vi, ni))
                if len(face) >= 3:
                    chunk['faces'].append(face)
    if not vertices or not any(c['faces'] for c in chunks):
        raise ValueError(f'OBJ contains no polygon mesh: {path}')
    return vertices, normals, chunks, materials


def _transform_components(vector, transform, normal=False):
    origin = transform.get('origin', (0, 0, 0))
    axes = transform.get('axes', ('x', 'y', 'z'))
    if sorted(a.lstrip('-+') for a in axes) != ['x', 'y', 'z']:
        raise ValueError('axes must be a signed permutation of x,y,z')
    source = list(vector) if normal else [vector[i]-origin[i] for i in range(3)]
    result = [(-1 if axis.startswith('-') else 1) * source['xyz'.index(axis[-1])]
              for axis in axes]
    if normal:
        length = math.sqrt(sum(x*x for x in result)) or 1
        return tuple(x/length for x in result)
    scale = float(transform.get('scale', .001))
    if scale <= 0:
        raise ValueError('scale must be positive')
    translation = transform.get('translation', (0, 0, 0))
    return tuple(result[i]*scale+translation[i] for i in range(3))


def suggest_collision_proxies_from_obj(obj_path, visual_transform=None, *, minimum_size=.002):
    """Return inspectable per-CAD-group AABBs without authoring any colliders.

    OBJ material chunks with an identical object/group name are combined before
    measuring bounds, so paint/material changes do not split collision proxies.
    Bounds use only vertices referenced by polygon faces. Groups whose largest
    dimension is below ``minimum_size`` are omitted; thin plates are retained
    and thickened symmetrically to that dimension on each thin axis.

    Example::

        suggestion = suggest_collision_proxies_from_obj(obj_path, transform)
        # Inspect suggestion['proxies'] before choosing useful boxes.
        build_scene(obj_path, usd_path, transform,
                    collision_proxies=suggestion['proxies'])

    Per-group AABBs approximate the complete extents, including voids in frames.
    The caller should inspect or split large boxes before using them.
    """
    minimum_size = float(minimum_size)
    if minimum_size <= 0:
        raise ValueError('minimum_size must be positive')
    transform = dict(visual_transform or {'scale': .001})
    vertices, _normals, chunks, _materials = read_obj(obj_path)
    points = [_transform_components(p, transform) for p in vertices]
    grouped = {}
    for chunk in chunks:
        entry = grouped.setdefault(chunk['name'], {
            'vertices': set(), 'materials': set(), 'face_count': 0, 'chunk_count': 0})
        entry['materials'].add(chunk['material'])
        entry['face_count'] += len(chunk['faces'])
        entry['chunk_count'] += 1
        for face in chunk['faces']:
            entry['vertices'].update(vi for vi, _ni in face)
    proxies, skipped, all_referenced = [], [], set()
    for name, entry in grouped.items():
        used = entry['vertices']
        all_referenced.update(used)
        if not used:
            skipped.append({'source_name': name, 'reason': 'no polygon vertices'})
            continue
        lo = tuple(min(points[v][i] for v in used) for i in range(3))
        hi = tuple(max(points[v][i] for v in used) for i in range(3))
        raw_size = tuple(hi[i]-lo[i] for i in range(3))
        if max(raw_size) < minimum_size:
            skipped.append({'source_name': name, 'reason': 'largest dimension below minimum',
                            'bounds': [lo, hi], 'raw_size': raw_size})
            continue
        size = tuple(max(x, minimum_size) for x in raw_size)
        proxies.append({'source_name': name,
                        'center': tuple((lo[i]+hi[i])/2 for i in range(3)),
                        'size': size, 'raw_bounds': [lo, hi], 'raw_size': raw_size,
                        'thickened_axes': ['xyz'[i] for i in range(3) if raw_size[i] < minimum_size],
                        'source_materials': sorted(entry['materials']),
                        'source_vertex_count': len(used), 'source_face_count': entry['face_count'],
                        'source_material_chunk_count': entry['chunk_count']})
    referenced_bounds = ([tuple(min(points[v][i] for v in all_referenced) for i in range(3)),
                          tuple(max(points[v][i] for v in all_referenced) for i in range(3))]
                         if all_referenced else None)
    return {'source_obj': str(Path(obj_path).resolve()), 'visual_transform': transform,
            'minimum_dimension_m': minimum_size, 'referenced_visual_bounds_m': referenced_bounds,
            'source_group_count': len(grouped), 'proxy_count': len(proxies),
            'proxies': proxies, 'skipped_groups': skipped,
            'warning': 'Suggested group AABBs include empty space in frames; inspect before applying.'}


def build_scene(obj_path, output_path, visual_transform=None, *, stage=None,
                spawn_lift=.002, collision_proxies=None, chassis_inertia_xy=None,
                physics_hz=480, open_in_viewport=False, tire_visual_segments=256):
    """Author, save and return (Usd.Stage, report); optionally open in running Kit.

    collision_proxies: list of {'center': xyz, 'size': xyz} in vehicle coordinates.
    Default proxy is the lower chassis deck. Supply CAD-informed proxies when
    upper equipment must collide. A proxy never controls mass/inertia.
    """
    from pxr import Gf, Sdf, Tf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade, PhysxSchema

    transform = dict(visual_transform or {'scale': .001})
    vertices, normals, chunks, source_materials = read_obj(obj_path)
    points = [_transform_components(p, transform) for p in vertices]
    lo = tuple(min(p[i] for p in points) for i in range(3))
    hi = tuple(max(p[i] for p in points) for i in range(3))
    dimensions = tuple(hi[i]-lo[i] for i in range(3))
    if max(dimensions) > 2.0 or max(dimensions) < .05:
        raise ValueError(f'Unexpected vehicle mesh size {dimensions}m; check OBJ units/transform')
    axes = transform.get('axes', ('x', 'y', 'z'))
    basis = [_transform_components(v, transform, normal=True)
             for v in ((1,0,0), (0,1,0), (0,0,1))]
    determinant = (basis[0][0]*(basis[1][1]*basis[2][2]-basis[1][2]*basis[2][1])
                   -basis[0][1]*(basis[1][0]*basis[2][2]-basis[1][2]*basis[2][0])
                   +basis[0][2]*(basis[1][0]*basis[2][1]-basis[1][1]*basis[2][0]))
    output_path = Path(output_path).resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    stage = stage or Usd.Stage.CreateNew(str(output_path))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdPhysics.SetStageKilogramsPerUnit(stage, 1.0)
    stage.SetTimeCodesPerSecond(60)
    world = UsdGeom.Xform.Define(stage, '/World')
    stage.SetDefaultPrim(world.GetPrim())
    stage.GetRootLayer().documentation = __doc__

    scene = UsdPhysics.Scene.Define(stage, '/World/PhysicsScene')
    scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
    scene.CreateGravityMagnitudeAttr(9.81)
    physx_scene = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
    physx_scene.CreateTimeStepsPerSecondAttr(int(physics_hz))
    physx_scene.CreateEnableCCDAttr(True)
    physx_scene.CreateSolverTypeAttr('TGS')
    physx_scene.CreateEnableGPUDynamicsAttr(False)

    def material(path, color, roughness=.6, opacity=1):
        mat = UsdShade.Material.Define(stage, path)
        shader = UsdShade.Shader.Define(stage, path+'/Shader')
        shader.CreateIdAttr('UsdPreviewSurface')
        shader.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
        shader.CreateInput('roughness', Sdf.ValueTypeNames.Float).Set(roughness)
        shader.CreateInput('opacity', Sdf.ValueTypeNames.Float).Set(opacity)
        shader.CreateOutput('surface', Sdf.ValueTypeNames.Token)
        mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), 'surface')
        return mat

    def bind(prim, mat):
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat)

    rubber = material('/World/Looks/Rubber', (.025, .029, .035), .9)
    ground_mat = material('/World/Looks/Ground', (.20, .23, .27), .85)
    contact_mat = UsdShade.Material.Define(stage, '/World/Looks/TireContact')
    contact = UsdPhysics.MaterialAPI.Apply(contact_mat.GetPrim())
    contact.CreateStaticFrictionAttr(.9)
    contact.CreateDynamicFrictionAttr(.9)
    contact.CreateRestitutionAttr(0)
    pmat = PhysxSchema.PhysxMaterialAPI.Apply(contact_mat.GetPrim())
    pmat.CreateFrictionCombineModeAttr('min')
    pmat.CreateRestitutionCombineModeAttr('min')

    def collision(prim, convex=False):
        UsdPhysics.CollisionAPI.Apply(prim)
        if convex:
            UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr('convexHull')
            # Keep all 128 vertices of the 64-sided tire. A low cooking limit
            # can turn it into a 16-sided section and lose ~.537mm of radius.
            # PhysX accepts at most 255 hull vertices; this leaves ample room.
            hull = PhysxSchema.PhysxConvexHullCollisionAPI.Apply(prim)
            hull.CreateHullVertexLimitAttr(255)
        pc = PhysxSchema.PhysxCollisionAPI.Apply(prim)
        pc.CreateContactOffsetAttr(.001)
        pc.CreateRestOffsetAttr(0)
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(contact_mat, materialPurpose='physics')

    def cube(path, center, size, hidden=False):
        shape = UsdGeom.Cube.Define(stage, path)
        shape.CreateSizeAttr(1)
        shape.AddTranslateOp().Set(Gf.Vec3d(*center))
        shape.AddScaleOp().Set(Gf.Vec3f(*size))
        if hidden:
            shape.CreateVisibilityAttr(UsdGeom.Tokens.invisible)
        return shape

    # A native plane gives stable analytic cylinder contact at this small scale.
    # In Isaac 6.0.1-rc.7 a 20m cube floor destabilized an isolated 55.88mm wheel.
    floor = UsdGeom.Plane.Define(stage, '/World/Ground')
    floor.CreateAxisAttr(UsdGeom.Tokens.z)
    floor.CreateWidthAttr(20)
    floor.CreateLengthAttr(20)
    collision(floor.GetPrim())
    bind(floor.GetPrim(), ground_mat)

    vehicle = UsdGeom.Xform.Define(stage, '/World/RC_Car')
    vehicle.AddTranslateOp().Set(Gf.Vec3d(0, 0, .07+spawn_lift))
    radius, width, mw, mh = .02794, .04, .075, .025
    mc = 3.5-4*(mw+mh)
    cx = -2*(mw+mh)*(.15-.10)/mc
    cz = (3.5*.07-4*(mw+mh)*radius)/mc-.07
    wheel_inertia = (mw*(3*radius**2+width**2)/12,
                     .5*mw*radius**2,
                     mw*(3*radius**2+width**2)/12)
    carrier_i = .4*mh*.012**2
    unsprung_izz = (2*(mw+mh)*(.15**2+.10**2)
                    +2*(mw+mh)*(.10**2+.115**2)
                    +4*(wheel_inertia[2]+carrier_i))
    chassis_izz = .04712-unsprung_izz-mc*cx**2
    if chassis_inertia_xy is None:
        dx, dy, dz = (max(x, .001) for x in dimensions)
        chassis_inertia_xy = (chassis_izz*(dy*dy+dz*dz)/(dx*dx+dy*dy),
                              chassis_izz*(dx*dx+dz*dz)/(dx*dx+dy*dy))
    chassis_inertia = tuple(chassis_inertia_xy)+(chassis_izz,)
    if min(chassis_inertia) <= 0 or 2*max(chassis_inertia) > sum(chassis_inertia)+1e-10:
        raise ValueError(f'Chassis inertia is not physically valid: {chassis_inertia}')

    def rigid(path, position, mass, inertia, com=(0,0,0)):
        xform = UsdGeom.Xform.Define(stage, path)
        xform.AddTranslateOp().Set(Gf.Vec3d(*position))
        body = UsdPhysics.RigidBodyAPI.Apply(xform.GetPrim())
        body.CreateRigidBodyEnabledAttr(True)
        body.CreateKinematicEnabledAttr(False)
        mass_api = UsdPhysics.MassAPI.Apply(xform.GetPrim())
        mass_api.CreateMassAttr(mass)
        mass_api.CreateCenterOfMassAttr(Gf.Vec3f(*com))
        mass_api.CreateDiagonalInertiaAttr(Gf.Vec3f(*inertia))
        mass_api.CreatePrincipalAxesAttr(Gf.Quatf(1))
        pb = PhysxSchema.PhysxRigidBodyAPI.Apply(xform.GetPrim())
        pb.CreateEnableCCDAttr(True)
        pb.CreateLinearDampingAttr(0)
        pb.CreateAngularDampingAttr(0)
        return xform

    chassis_path = '/World/RC_Car/Chassis'
    chassis = rigid(chassis_path, (0,0,0), mc, chassis_inertia, (cx,0,cz))
    UsdPhysics.ArticulationRootAPI.Apply(chassis.GetPrim())
    art = PhysxSchema.PhysxArticulationAPI.Apply(chassis.GetPrim())
    art.CreateEnabledSelfCollisionsAttr(False)
    art.CreateSolverPositionIterationCountAttr(16)
    art.CreateSolverVelocityIterationCountAttr(4)
    UsdGeom.Scope.Define(stage, chassis_path+'/Visuals')
    material_cache = {}
    mesh_count, face_count = 0, 0
    for index, chunk in enumerate(chunks):
        if not chunk['faces']:
            continue
        name = Tf.MakeValidIdentifier(chunk['name']) or 'CAD'
        mesh = UsdGeom.Mesh.Define(stage, f'{chassis_path}/Visuals/{name}_{index:04d}')
        mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
        mesh.CreateDoubleSidedAttr(True)
        used, local_points, counts, indices, face_normals = {}, [], [], [], []
        has_normals = True
        for original_face in chunk['faces']:
            face = original_face if determinant > 0 else list(reversed(original_face))
            counts.append(len(face))
            for vi, ni in face:
                if vi not in used:
                    used[vi] = len(local_points)
                    local_points.append(Gf.Vec3f(*points[vi]))
                indices.append(used[vi])
                if ni is not None and 0 <= ni < len(normals):
                    face_normals.append(Gf.Vec3f(*_transform_components(normals[ni], transform, normal=True)))
                else:
                    has_normals = False
        mesh.CreatePointsAttr(local_points)
        mesh.CreateFaceVertexCountsAttr(counts)
        mesh.CreateFaceVertexIndicesAttr(indices)
        if has_normals:
            mesh.CreateNormalsAttr(face_normals)
            mesh.SetNormalsInterpolation(UsdGeom.Tokens.faceVarying)
        key = chunk['material']
        if key not in material_cache:
            props = source_materials.get(key, {'color': (.55,.57,.60), 'opacity': 1})
            # Preserve CAD color; keep solid engineering visuals for occluded components.
            material_cache[key] = material('/World/Looks/CAD_'+str(len(material_cache)), props['color'], .5, props['opacity'])
        bind(mesh.GetPrim(), material_cache[key])
        mesh_count += 1
        face_count += len(counts)

    proxies = collision_proxies or [{'center': (.025,0,-.052), 'size': (.30,.12,.012)}]
    for index, proxy in enumerate(proxies):
        collider = cube(f'{chassis_path}/CollisionProxy_{index}', proxy['center'], proxy['size'], True)
        if proxy.get('source_name'):
            collider.GetPrim().SetDisplayName(proxy['source_name'])
            collider.GetPrim().SetCustomDataByKey('source_obj_group', proxy['source_name'])
        collision(collider.GetPrim())

    def cylinder_visual_mesh(path):
        # Author the tessellation explicitly: the viewport's implicit Cylinder
        # tessellation was visibly coarse even though its nominal radius was exact.
        count = int(tire_visual_segments)
        if count < 64 or count % 4:
            raise ValueError('tire_visual_segments must be a multiple of four, at least 64')
        mesh = UsdGeom.Mesh.Define(stage, path)
        points = [Gf.Vec3f(radius*math.cos(2*math.pi*j/count), y,
                          radius*math.sin(2*math.pi*j/count))
                  for y in (-width/2, width/2) for j in range(count)]
        points += [Gf.Vec3f(0, -width/2, 0), Gf.Vec3f(0, width/2, 0)]
        faces, face_normals = [], []
        for j in range(count):
            jp = (j+1) % count
            face = [j+count, jp+count, jp, j]
            faces.append(face)
            for index in face:
                angle = 2*math.pi*(index % count)/count
                face_normals.append(Gf.Vec3f(math.cos(angle), 0, math.sin(angle)))
        for j in range(count):
            jp = (j+1) % count
            faces.append([2*count, j, jp])
            face_normals.extend([Gf.Vec3f(0, -1, 0)]*3)
            faces.append([2*count+1, jp+count, j+count])
            face_normals.extend([Gf.Vec3f(0, 1, 0)]*3)
        mesh.CreatePointsAttr(points)
        mesh.CreateFaceVertexCountsAttr([len(f) for f in faces])
        mesh.CreateFaceVertexIndicesAttr([i for f in faces for i in f])
        mesh.CreateNormalsAttr(face_normals)
        mesh.SetNormalsInterpolation(UsdGeom.Tokens.faceVarying)
        mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
        mesh.CreateDoubleSidedAttr(False)
        mesh.CreateExtentAttr([Gf.Vec3f(-radius,-width/2,-radius),
                              Gf.Vec3f(radius,width/2,radius)])
        mesh.GetPrim().SetDisplayName('Smooth cylinder tire - '+str(count)+' radial segments')
        mesh.GetPrim().SetCustomDataByKey('radial_segments', count)
        mesh.GetPrim().SetCustomDataByKey('radius_m', radius)
        mesh.GetPrim().SetCustomDataByKey('width_m', width)
        bind(mesh.GetPrim(), rubber)
        return mesh

    def cylinder_collision(path):
        # Isaac Sim's AnalyticCylinderDemo uses a native Cylinder with collision
        # and collisionApproximateCylinders=False for a smooth analytic surface.
        cylinder = UsdGeom.Cylinder.Define(stage, path)
        cylinder.CreateRadiusAttr(radius)
        cylinder.CreateHeightAttr(width)
        cylinder.CreateAxisAttr(UsdGeom.Tokens.y)
        cylinder.CreateVisibilityAttr(UsdGeom.Tokens.invisible)
        cylinder.GetPrim().SetDisplayName('Analytic cylinder tire collision')
        collision(cylinder.GetPrim())
        return cylinder

    wheel_report = []
    for name, x, y, preload in [('FL',.15,.10,-.0002943), ('FR',.15,-.10,-.0002943),
                                ('RL',-.10,.115,-.000465975), ('RR',-.10,-.115,-.000465975)]:
        position = (x,y,radius-.07)
        hub_path, wheel_path = f'/World/RC_Car/Carrier_{name}', f'/World/RC_Car/Wheel_{name}'
        rigid(hub_path, position, mh, (carrier_i,)*3)
        rigid(wheel_path, position, mw, wheel_inertia)
        cylinder_visual_mesh(wheel_path+'/Tire')
        cylinder_collision(wheel_path+'/TireCollision')
        joint = UsdPhysics.PrismaticJoint.Define(stage, f'/World/RC_Car/Joints/Suspension_{name}')
        joint.CreateBody0Rel().SetTargets([Sdf.Path(chassis_path)])
        joint.CreateBody1Rel().SetTargets([Sdf.Path(hub_path)])
        joint.CreateLocalPos0Attr(Gf.Vec3f(*position))
        joint.CreateLocalPos1Attr(Gf.Vec3f(0))
        joint.CreateLocalRot0Attr(Gf.Quatf(1))
        joint.CreateLocalRot1Attr(Gf.Quatf(1))
        joint.CreateAxisAttr('Z')
        joint.CreateLowerLimitAttr(-.0015)
        joint.CreateUpperLimitAttr(.0015)
        joint.CreateCollisionEnabledAttr(False)
        drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), 'linear')
        drive.CreateTypeAttr('force')
        drive.CreateStiffnessAttr(20000)
        drive.CreateDampingAttr(200)
        drive.CreateMaxForceAttr(100)
        drive.CreateTargetPositionAttr(preload)
        drive.CreateTargetVelocityAttr(0)
        axle = UsdPhysics.RevoluteJoint.Define(stage, f'/World/RC_Car/Joints/Axle_{name}')
        axle.CreateBody0Rel().SetTargets([Sdf.Path(hub_path)])
        axle.CreateBody1Rel().SetTargets([Sdf.Path(wheel_path)])
        axle.CreateLocalPos0Attr(Gf.Vec3f(0))
        axle.CreateLocalPos1Attr(Gf.Vec3f(0))
        axle.CreateLocalRot0Attr(Gf.Quatf(1))
        axle.CreateLocalRot1Attr(Gf.Quatf(1))
        axle.CreateAxisAttr('Y')
        axle.CreateCollisionEnabledAttr(False)
        # No angular drive: wheels can spin, so the attachment is not a fixed joint.
        wheel_report.append({'name': name, 'center_vehicle_m': position,
                             'spring_target_m': preload, 'carrier_mass_kg': mh, 'tire_mass_kg': mw})

    dome = UsdLux.DomeLight.Define(stage, '/World/Lighting/Dome')
    dome.CreateIntensityAttr(700)
    distant = UsdLux.DistantLight.Define(stage, '/World/Lighting/Sun')
    distant.CreateIntensityAttr(1500)
    distant.CreateAngleAttr(1)
    distant.AddRotateXYZOp().Set(Gf.Vec3f(-35,-25,-20))
    camera = UsdGeom.Camera.Define(stage, '/World/Camera')
    view = Gf.Matrix4d().SetLookAt(Gf.Vec3d(.72,-.78,.52), Gf.Vec3d(0,0,.08), Gf.Vec3d(0,0,1))
    camera.AddTransformOp().Set(view.GetInverse())
    camera.CreateFocalLengthAttr(35)
    camera.CreateClippingRangeAttr(Gf.Vec2f(.005,100))
    # Measure the authored bodies independently of the intended constants above.
    cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    measured_bodies = []
    for prim in stage.Traverse():
        if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
            continue
        ma = UsdPhysics.MassAPI(prim)
        mass = float(ma.GetMassAttr().Get())
        world_matrix = cache.GetLocalToWorldTransform(prim)
        com_world = world_matrix.Transform(Gf.Vec3d(ma.GetCenterOfMassAttr().Get()))
        inertia = ma.GetDiagonalInertiaAttr().Get()
        principal_rotation = Gf.Rotation(Gf.Quatd(ma.GetPrincipalAxesAttr().Get()))
        body_izz = sum(float(inertia[i]) * world_matrix.TransformDir(
            principal_rotation.TransformDir(Gf.Vec3d(*axis)))[2]**2
            for i, axis in enumerate(((1,0,0), (0,1,0), (0,0,1))))
        measured_bodies.append((mass, com_world, body_izz))
    measured_mass = sum(b[0] for b in measured_bodies)
    measured_com = [sum(b[0]*b[1][i] for b in measured_bodies)/measured_mass for i in range(3)]
    measured_izz = sum(b[2]+b[0]*((b[1][0]-measured_com[0])**2+(b[1][1]-measured_com[1])**2)
                       for b in measured_bodies)
    if abs(measured_mass-3.5) > 1e-5 or abs(measured_izz-.04712) > 1e-6:
        raise AssertionError(f'Authored mass/inertia mismatch: {measured_mass}, {measured_izz}')
    if max(abs(measured_com[i]-(0,0,.07+spawn_lift)[i]) for i in range(3)) > 1e-6:
        raise AssertionError(f'Authored aggregate COM mismatch: {measured_com}')
    report = {'source_obj': str(Path(obj_path).resolve()), 'output_usd': str(output_path),
              'visual_transform': transform, 'visual_bounds_vehicle_m': [lo,hi],
              'visual_dimensions_m': dimensions, 'visual_meshes': mesh_count,
              'visual_faces': face_count, 'mass_total_kg': 3.5,
              'measured_initial_mass_kg': measured_mass,
              'measured_initial_com_world_m': measured_com,
              'measured_initial_yaw_inertia_kg_m2': measured_izz,
              'measured_rigid_body_count': len(measured_bodies),
              'nominal_loaded_com_world_m': [0,0,.07], 'initial_spawn_lift_m': spawn_lift,
              'aggregate_yaw_inertia_kg_m2': .04712, 'chassis_mass_kg': mc,
              'chassis_com_local_m': [cx,0,cz], 'chassis_diagonal_inertia_kg_m2': chassis_inertia,
              'wheel_diagonal_inertia_kg_m2': wheel_inertia, 'carrier_diagonal_inertia_kg_m2': [carrier_i]*3,
              'wheel_diameter_m': radius*2, 'wheel_width_m': width,
              'tire_visual_radial_segments': int(tire_visual_segments),
              'tire_visual_vertices_per_wheel': int(tire_visual_segments)*2+2,
              'tire_visual_triangles_per_wheel': int(tire_visual_segments)*4,
              'tire_visual_normals': 'smooth radial sidewall normals; flat cap normals',
              'tire_collision_type': 'analytic cylinder',
              'physics_collision_approximate_cylinders': False,
              'physics_gpu_dynamics': False,
              'ground_collision_type': 'native plane at Z=0',
              'ground_visual_dimensions_m': [20,20],
              'front_rear_outside_width_m': [.24,.27], 'ini_planner_width_m': .22,
              'physics_hz': int(physics_hz), 'spring_N_per_m': 20000,
              'damper_N_s_per_m': 200, 'suspension_limits_m': [-.0015,.0015],
              'collision_proxies': proxies, 'wheels': wheel_report,
              'assumptions': ['Wheel/carrier mass split is estimated; total mass/COM/yaw follow INI.',
                              'Roll/pitch inertia uses CAD bounding-box aspect ratio unless overridden.',
                              'CAD visuals are rigidly grouped; collision uses specified box proxies.',
                              'Tire friction is constant .9; INI Magic Formula/drag are not implemented.',
                              'No ROS, active sensors, steering actuator, or wheel propulsion configured.']}
    stage.GetRootLayer().customLayerData = {
        'physicsSettings': {'/physics/collisionApproximateCylinders': False},
        'rc_car_build_report_json': json.dumps(report, ensure_ascii=False)}
    stage.GetRootLayer().Save()
    if Path(stage.GetRootLayer().realPath).resolve() != output_path:
        stage.GetRootLayer().Export(str(output_path))
    output_path.with_suffix('.build.json').write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding='utf-8')
    if open_in_viewport:
        import omni.usd
        omni.usd.get_context().open_stage(str(output_path))
        try:
            from omni.kit.viewport.utility import get_active_viewport
            viewport = get_active_viewport()
            if viewport:
                viewport.camera_path = '/World/Camera'
        except ImportError:
            pass
    return stage, report


if __name__ == '__main__':
    print('Loaded RC-car builder. Call build_scene(obj_path, output_path, visual_transform=...).')
