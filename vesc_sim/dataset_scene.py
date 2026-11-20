"""Bind a dataset track and place the rear axle on its raceline before Play."""
import math
from pathlib import Path

from pxr import Gf, Sdf, UsdGeom, UsdPhysics, UsdShade

from steering import load_configuration


def resolve_dataset(dataset):
    """Validate the named geometry and calibrated profile before stopping a drive."""
    folder = Path(__file__).resolve().parent
    config = load_configuration(folder/'config/vesc_sim.yaml', surface_dataset=dataset)
    surface = config['_surface']
    if not surface or 'fallback' in surface['status']:
        raise ValueError(f'No calibrated friction coefficient for dataset {dataset!r}')
    if dataset != Path(dataset).name:
        raise ValueError('Expected a dataset directory name')
    source = folder.parent/'maps'/(dataset+'.usdc')
    if not source.is_file():
        raise FileNotFoundError(source)
    return config, source, surface


def configure_dataset(stage, dataset, spawn):
    """Apply geometry and calibration in the session layer. All lengths are metres."""
    config, source, surface = resolve_dataset(dataset)
    if UsdGeom.GetStageUpAxis(stage) != UsdGeom.Tokens.z or UsdGeom.GetStageMetersPerUnit(stage) != 1.:
        raise ValueError('Expected a Z-up stage in metres')
    x, y, yaw = map(float, spawn)
    if not all(map(math.isfinite, (x, y, yaw))):
        raise ValueError('Expected a finite rear-axle pose')

    scene = config['scene']
    root = stage.GetPrimAtPath(scene['vehicle_root'])
    track = stage.GetPrimAtPath('/World/TrackEnvironment')
    cache = UsdGeom.XformCache()
    chassis = cache.GetLocalToWorldTransform(stage.GetPrimAtPath(scene['chassis']))
    rear = chassis.Transform(Gf.Vec3d(*scene['rear_axle_offset_m']))
    forward = chassis.TransformDir(Gf.Vec3d(1, 0, 0))
    angle = math.degrees(yaw - math.atan2(forward[1], forward[0]))
    rotate = Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(0, 0, 1), angle))
    rear = rotate.Transform(rear)
    # The source asset is suspended 0.53 m above the map. Start the four
    # upright wheel centres one millimetre above contact to avoid a drop.
    wheel_z = min(cache.GetLocalToWorldTransform(stage.GetPrimAtPath(
        scene['vehicle_root']+'/Wheel_'+wheel)).ExtractTranslation()[2] for wheel in ('FL','FR','RL','RR'))
    dz = float(config['drivetrain']['radius']) + .001 - wheel_z
    shift = Gf.Matrix4d().SetTranslate(Gf.Vec3d(x-rear[0], y-rear[1], dz))
    world = cache.GetLocalToWorldTransform(root)*rotate*shift
    local = world*cache.GetParentToWorldTransform(root).GetInverse()

    stage.SetEditTarget(stage.GetSessionLayer())
    track.GetReferences().SetReferences([Sdf.Reference(str(source))])
    if not stage.GetPrimAtPath('/World/TrackEnvironment/Ground'):
        raise RuntimeError(f'Track ground missing in {source}')
    # The rendered ground slab is flat. A PhysX plane has an analytic wheel
    # contact for every heading and avoids cylinder versus thin-box edge contacts.
    ground = stage.GetPrimAtPath('/World/TrackEnvironment/Ground')
    if not ground.IsA(UsdGeom.Cube):
        raise ValueError('Expected a flat occupancy-map floor slab')
    floor_xform = UsdGeom.XformCache().GetLocalToWorldTransform(ground)
    half_size = UsdGeom.Cube(ground).GetSizeAttr().Get()/2
    top = [floor_xform.Transform(Gf.Vec3d(i*half_size,j*half_size,half_size))[2]
           for i in (-1,1) for j in (-1,1)]
    if any(abs(z)>1e-6 for z in top):
        raise ValueError('Expected all four corners of the map surface at Z=0')
    UsdPhysics.CollisionAPI(ground).CreateCollisionEnabledAttr(False)
    floor = UsdGeom.Plane.Define(stage, '/World/DamviTrackGroundContact')
    floor.CreateAxisAttr('Z'); floor.CreateVisibilityAttr('invisible')
    UsdPhysics.CollisionAPI.Apply(floor.GetPrim()).CreateCollisionEnabledAttr(True)
    floor_mat = UsdShade.MaterialBindingAPI(ground).ComputeBoundMaterial('physics')[0]
    if floor_mat:
        UsdShade.MaterialBindingAPI.Apply(floor.GetPrim()).Bind(floor_mat,materialPurpose='physics')
    track.SetCustomDataByKey('damvi:dataset', dataset)
    UsdGeom.Xformable(root).MakeMatrixXform().Set(local)
    return {'dataset':dataset, 'usd':str(source), 'rear_axle_spawn_xyyaw':list(spawn),
            'surface':surface, 'ground_contact':'horizontal plane at original slab top Z=0'}
