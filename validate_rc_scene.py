"""Independent USD and live PhysX validation for the RC-car scene.

Load in the live Kit process, then schedule (do not block the app thread)::

    exec(open('/home/shin/Desktop/RC_Car_Isaac/validate_rc_scene.py').read())
    import asyncio
    rc_validation_task = asyncio.ensure_future(validate_rc_scene())

The coroutine writes progress to validation.json and restores the saved, stopped
scene in finally. A result passes only if both static and actual runtime checks
pass. No fabricated result or USD-only fallback is used for physical motion.
"""

from __future__ import annotations

import asyncio
import configparser
import json
import math
import time
import traceback
from pathlib import Path

RC_ROOT = '/World/RC_Car'
WHEEL_NAMES = ('FL', 'FR', 'RL', 'RR')


def _parameters(ini_path):
    cfg = configparser.ConfigParser(interpolation=None)
    cfg.read_string(Path(ini_path).read_text(encoding='utf-8-sig', errors='replace'))
    return (json.loads(cfg['GENERAL_OPTIONS']['veh_params']),
            json.loads(cfg['OPTIMIZATION_OPTIONS']['vehicle_params_mintime']))


def _vec(value):
    return [float(x) for x in value]


def _body_failure_snapshot(paths, poses, velocities, elapsed_s, previous_samples):
    """Retain diagnostics without serializing NaN/Inf as invalid JSON numbers."""
    def safe_vector(values):
        return [float(v) if math.isfinite(float(v)) else str(float(v)) for v in values]
    return dict(simulated_time_s=float(elapsed_s),
        body_states=[dict(path=str(path),pose_xyz_xyzw=safe_vector(pose),
                         linear_velocity_mps=safe_vector(velocity[:3]),
                         angular_velocity_radps=safe_vector(velocity[3:]))
                     for path,pose,velocity in zip(paths,poses,velocities)],
        preceding_valid_samples=previous_samples[-10:])


def _qmatrix_xyzw(q):
    import numpy as np
    x, y, z, w = (float(v) for v in q)
    norm = math.sqrt(x*x+y*y+z*z+w*w)
    if not math.isfinite(norm) or norm < 1e-10:
        raise ValueError('Nonfinite or zero body quaternion')
    x, y, z, w = x/norm, y/norm, z/norm, w/norm
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def validate_static(stage, ini_path):
    """Compute aggregate values from authored bodies, not builder report totals."""
    import numpy as np
    from pxr import Gf, Usd, UsdGeom, UsdPhysics, PhysxSchema

    general, vehicle = _parameters(ini_path)
    checks = []

    def check(name, passed, measured=None, expected=None, tolerance=None):
        entry = dict(name=name, passed=bool(passed))
        if measured is not None:
            entry['measured'] = measured
        if expected is not None:
            entry['expected'] = expected
        if tolerance is not None:
            entry['tolerance'] = tolerance
        checks.append(entry)

    def close(name, actual, expected, tolerance):
        a, b = np.asarray(actual, dtype=float), np.asarray(expected, dtype=float)
        check(name, np.all(np.isfinite(a)) and np.all(np.abs(a-b) <= tolerance),
              a.tolist(), b.tolist(), tolerance)

    vehicle_prim = stage.GetPrimAtPath(RC_ROOT)
    if not vehicle_prim:
        raise ValueError(f'Missing {RC_ROOT}')
    all_prims = list(Usd.PrimRange(vehicle_prim))
    bodies = [p for p in all_prims if p.HasAPI(UsdPhysics.RigidBodyAPI)]
    joints = [p for p in all_prims if p.IsA(UsdPhysics.Joint)]
    check('rigid_body_count', len(bodies) == 9, len(bodies), 9)
    check('joint_count', len(joints) == 8, len(joints), 8)
    check('meters_per_unit', UsdGeom.GetStageMetersPerUnit(stage) == 1,
          UsdGeom.GetStageMetersPerUnit(stage), 1)
    check('kilograms_per_unit', UsdPhysics.GetStageKilogramsPerUnit(stage) == 1,
          UsdPhysics.GetStageKilogramsPerUnit(stage), 1)
    check('z_up', UsdGeom.GetStageUpAxis(stage) == UsdGeom.Tokens.z)
    cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    masses, centers, izz_world, body_properties = [], [], [], {}
    for prim in bodies:
        mass_api = UsdPhysics.MassAPI(prim)
        mass = float(mass_api.GetMassAttr().Get())
        com_local = mass_api.GetCenterOfMassAttr().Get()
        inertia = mass_api.GetDiagonalInertiaAttr().Get()
        principal = mass_api.GetPrincipalAxesAttr().Get()
        matrix = cache.GetLocalToWorldTransform(prim)
        center = matrix.Transform(Gf.Vec3d(com_local))
        principal_rotation = Gf.Rotation(Gf.Quatd(principal))
        axes = [matrix.TransformDir(principal_rotation.TransformDir(axis))
                for axis in (Gf.Vec3d(1,0,0), Gf.Vec3d(0,1,0), Gf.Vec3d(0,0,1))]
        scales = [axis.GetLength() for axis in axes]
        close(str(prim.GetPath())+'/unit_scale', scales, [1,1,1], 1e-7)
        check(str(prim.GetPath())+'/inertia_positive', min(inertia) > 0,
              _vec(inertia))
        check(str(prim.GetPath())+'/inertia_triangle',
              2*max(inertia) <= sum(inertia)+1e-7)
        izz_world.append(sum(float(inertia[k])*float(axes[k][2])**2 for k in range(3)))
        masses.append(mass)
        centers.append(_vec(center))
        body_properties[str(prim.GetPath())] = dict(mass_kg=mass,
            com_local_m=_vec(com_local), world_com_m=_vec(center),
            diagonal_inertia_kg_m2=_vec(inertia))
    masses_a, centers_a = np.asarray(masses), np.asarray(centers)
    total_mass = float(masses_a.sum())
    aggregate_com = (masses_a[:, None]*centers_a).sum(axis=0)/total_mass
    aggregate_izz = float(sum(izz_world)+sum(masses_a*((centers_a[:,0]-aggregate_com[0])**2+
                                                        (centers_a[:,1]-aggregate_com[1])**2)))
    close('aggregate_mass_kg', total_mass, general['mass'], 2e-6)
    close('aggregate_yaw_inertia_kg_m2', aggregate_izz, vehicle['I_z'], 2e-7)
    close('aggregate_com_xy_m', aggregate_com[:2], [0,0], 2e-7)

    wheel_centers, tire_points, suspension_nominal, tire_geometry = {}, {}, {}, {}
    radius, width = 2.2*.0254/2, .04
    for name in WHEEL_NAMES:
        wheel_path = RC_ROOT+'/Wheel_'+name
        wheel = stage.GetPrimAtPath(wheel_path)
        center = cache.GetLocalToWorldTransform(wheel).Transform(Gf.Vec3d(0))
        wheel_centers[name] = _vec(center)
        visual_mesh_prim = stage.GetPrimAtPath(wheel_path+'/TireMesh')
        if not visual_mesh_prim:
            candidate = stage.GetPrimAtPath(wheel_path+'/Tire')
            if candidate and candidate.IsA(UsdGeom.Mesh):
                visual_mesh_prim = candidate
        if visual_mesh_prim and visual_mesh_prim.IsA(UsdGeom.Mesh):
            visual_mesh = UsdGeom.Mesh(visual_mesh_prim)
            visual_to_wheel, visual_reset = cache.ComputeRelativeTransform(visual_mesh_prim, wheel)
            vp = np.asarray([_vec(visual_to_wheel.Transform(Gf.Vec3d(p)))
                             for p in visual_mesh.GetPointsAttr().Get()])
            close(name+'/visual_dimensions_m', vp.max(axis=0)-vp.min(axis=0),
                  [2*radius,width,2*radius], 1e-7)
            radial = np.linalg.norm(vp[:,[0,2]],axis=1)
            rim = vp[radial > radius*.5]
            close(name+'/visual_rim_radius_m', radial[radial > radius*.5], radius, 1e-7)
            angles = np.round(np.mod(np.arctan2(rim[:,2],rim[:,0]),2*math.pi),6)
            segment_count = len(np.unique(angles))
            check(name+'/visual_smooth_circle_segments', segment_count >= 256, segment_count, 256)
            normals = visual_mesh.GetNormalsAttr().Get()
            interpolation = str(visual_mesh.GetNormalsInterpolation())
            check(name+'/visual_smooth_normals', bool(normals) and interpolation in ('vertex','faceVarying'),
                  {'normal_count':len(normals) if normals else 0,'interpolation':interpolation})
            check(name+'/visual_inherits_wheel_transform', not visual_reset)
            visual_type='smooth_mesh'
        else:
            tire = UsdGeom.Cylinder(stage.GetPrimAtPath(wheel_path+'/Tire'))
            check(name+'/cylindrical_tire_visual', bool(tire))
            if tire:
                close(name+'/diameter_m', 2*tire.GetRadiusAttr().Get(), 2*radius, 1e-8)
                close(name+'/width_m', tire.GetHeightAttr().Get(), width, 1e-8)
                check(name+'/axle_axis', tire.GetAxisAttr().Get() == 'Y',
                      tire.GetAxisAttr().Get(), 'Y')
            visual_type='native_cylinder'
        collision_prim = stage.GetPrimAtPath(wheel_path+'/TireCollision')
        check(name+'/tire_collision_enabled', collision_prim.HasAPI(UsdPhysics.CollisionAPI))
        if collision_prim.IsA(UsdGeom.Cylinder):
            cylinder=UsdGeom.Cylinder(collision_prim)
            check(name+'/collision_axis_y', cylinder.GetAxisAttr().Get() == 'Y')
            cr, ch = float(cylinder.GetRadiusAttr().Get()), float(cylinder.GetHeightAttr().Get())
            # Dense support samples are for validation only; the collider remains native Cylinder.
            raw_points=[Gf.Vec3d(cr*math.cos(2*math.pi*j/1024),y,cr*math.sin(2*math.pi*j/1024))
                        for y in (-ch/2,ch/2) for j in range(1024)]
            collider_type='native_cylinder'
        elif collision_prim.IsA(UsdGeom.Mesh):
            check(name+'/convex_tire_collider',
                  UsdPhysics.MeshCollisionAPI(collision_prim).GetApproximationAttr().Get() == 'convexHull')
            raw_points=UsdGeom.Mesh(collision_prim).GetPointsAttr().Get()
            collider_type='convex_mesh'
        else:
            raise AssertionError(f'Unsupported tire collision geometry at {collision_prim.GetPath()}')
        collision_to_wheel, resets_stack = cache.ComputeRelativeTransform(collision_prim, wheel)
        check(name+'/collider_inherits_wheel_transform', not resets_stack)
        pts = np.asarray([_vec(collision_to_wheel.Transform(Gf.Vec3d(p)))
                          for p in raw_points])
        tire_points[name] = pts.tolist()
        tire_geometry[name] = dict(visual_type=visual_type, collider_type=collider_type,
            collision_schemas=list(collision_prim.GetAppliedSchemas()))
        close(name+'/collision_dimensions_m', pts.max(axis=0)-pts.min(axis=0),
              [2*radius,width,2*radius], 1e-7)
        spring = UsdPhysics.PrismaticJoint(stage.GetPrimAtPath(RC_ROOT+'/Joints/Suspension_'+name))
        axle = UsdPhysics.RevoluteJoint(stage.GetPrimAtPath(RC_ROOT+'/Joints/Axle_'+name))
        check(name+'/spring_axis', bool(spring) and spring.GetAxisAttr().Get() == 'Z')
        check(name+'/axle_revolute', bool(axle) and axle.GetAxisAttr().Get() == 'Y')
        close(name+'/spring_limits_m', [spring.GetLowerLimitAttr().Get(),spring.GetUpperLimitAttr().Get()],
              [-.0015,.0015], 1e-8)
        spring_drive = UsdPhysics.DriveAPI(spring.GetPrim(), 'linear')
        check(name+'/positive_spring_and_damper',
              spring_drive.GetStiffnessAttr().Get() > 0 and spring_drive.GetDampingAttr().Get() > 0)
        check(name+'/body_connection',
              [str(p) for p in spring.GetBody0Rel().GetTargets()] == [RC_ROOT+'/Chassis'] and
              [str(p) for p in spring.GetBody1Rel().GetTargets()] == [RC_ROOT+'/Carrier_'+name] and
              [str(p) for p in axle.GetBody0Rel().GetTargets()] == [RC_ROOT+'/Carrier_'+name] and
              [str(p) for p in axle.GetBody1Rel().GetTargets()] == [wheel_path])
        suspension_nominal[name] = _vec(spring.GetLocalPos0Attr().Get())

    wc = {k: np.asarray(v) for k,v in wheel_centers.items()}
    close('front_track_m', abs(wc['FL'][1]-wc['FR'][1]), vehicle['track_width_front'], 1e-7)
    close('rear_track_m', abs(wc['RL'][1]-wc['RR'][1]), vehicle['track_width_rear'], 1e-7)
    close('wheelbase_m', wc['FL'][0]-wc['RL'][0], vehicle['wheelbase_front']+vehicle['wheelbase_rear'], 1e-7)
    close('front_axle_relative_com_m', wc['FL'][0]-aggregate_com[0], vehicle['wheelbase_front'], 1e-7)
    close('rear_axle_relative_com_m', aggregate_com[0]-wc['RL'][0], vehicle['wheelbase_rear'], 1e-7)
    nominal_support_height = float(np.mean([v[2]-radius for v in wc.values()]))
    close('nominal_loaded_com_height_m', aggregate_com[2]-nominal_support_height, vehicle['cog_z'], 2e-7)
    close('wheel_center_heights_m', [p[2] for p in wc.values()], [radius+nominal_support_height]*4, 1e-7)
    ground = stage.GetPrimAtPath('/World/Ground')
    check('ground_static_collision', ground.HasAPI(UsdPhysics.CollisionAPI)
          and not ground.HasAPI(UsdPhysics.RigidBodyAPI))
    bounds = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    if ground.GetTypeName() == 'Plane':
        plane=UsdGeom.Plane(ground)
        plane_axis=str(plane.GetAxisAttr().Get())
        normal={'X':Gf.Vec3d(1,0,0),'Y':Gf.Vec3d(0,1,0),'Z':Gf.Vec3d(0,0,1)}[plane_axis]
        plane_transform=cache.GetLocalToWorldTransform(ground)
        world_normal=plane_transform.TransformDir(normal).GetNormalized()
        close('floor_plane_world_normal',_vec(world_normal),[0,0,1],1e-7)
        ground_top=plane_transform.Transform(Gf.Vec3d(0))[2]
    else:
        ground_top = bounds.ComputeWorldBound(ground).ComputeAlignedRange().GetMax()[2]
    close('floor_top_z_m', ground_top, 0, 1e-7)
    scene = UsdPhysics.Scene(stage.GetPrimAtPath('/World/PhysicsScene'))
    close('gravity_m_per_s2', scene.GetGravityMagnitudeAttr().Get(), general['g'], 1e-6)
    close('gravity_direction', _vec(scene.GetGravityDirectionAttr().Get()), [0,0,-1], 1e-7)
    chassis_prim = stage.GetPrimAtPath(RC_ROOT+'/Chassis')
    world_to_chassis = cache.GetLocalToWorldTransform(chassis_prim).GetInverse()
    chassis_collision_points = []
    for p in Usd.PrimRange(chassis_prim):
        if p.HasAPI(UsdPhysics.CollisionAPI) and p.IsA(UsdGeom.Cube):
            half = UsdGeom.Cube(p).GetSizeAttr().Get()/2
            transform = cache.GetLocalToWorldTransform(p)*world_to_chassis
            chassis_collision_points += [_vec(transform.Transform(Gf.Vec3d(x,y,z)))
                for x in (-half,half) for y in (-half,half) for z in (-half,half)]
    check('chassis_box_collision_proxies_present', bool(chassis_collision_points))
    cad_meshes = [p for p in all_prims if p.IsA(UsdGeom.Mesh) and '/Chassis/Visuals/' in str(p.GetPath())]
    check('cad_visuals_present', len(cad_meshes) > 0, len(cad_meshes))
    check('single_rigid_chassis_visuals', all(not p.HasAPI(UsdPhysics.RigidBodyAPI) for p in cad_meshes))
    check('no_fixed_wheel_joints', all(not p.IsA(UsdPhysics.FixedJoint) for p in joints))
    return dict(passed=all(c['passed'] for c in checks), checks=checks,
        measured_mass_kg=total_mass, measured_initial_com_world_m=aggregate_com.tolist(),
        measured_nominal_loaded_com_height_m=float(aggregate_com[2]-nominal_support_height),
        measured_yaw_inertia_kg_m2=aggregate_izz, spawn_lift_m=nominal_support_height,
        wheel_centers_world_m=wheel_centers, body_properties=body_properties,
        suspension_nominal_m=suspension_nominal, tire_collision_points_m=tire_points,
        tire_geometry=tire_geometry,
        chassis_collision_points_m=chassis_collision_points,
        expected_com_height_m=float(vehicle['cog_z']), ground_top_z_m=float(ground_top),
        ground_geometry_type=str(ground.GetTypeName()))


async def _validate_car_rolling(sim_view, rigid_view, paths, static, app, timeline,
        elapsed, physics_steps, simulation, dt, stepping_mode, duration_s=2.0,
        speed_mps=.3, wall_timeout_s=120, diagnostics=None):
    """Initialize actual car motion once; measure free rolling, no persistent drive."""
    import numpy as np

    art=sim_view.create_articulation_view(RC_ROOT+'/Chassis')
    if int(art.count) != 1:
        raise AssertionError(f'Rolling test expects one car articulation, got {art.count}')
    meta=getattr(art,'shared_metatype',None)
    names=list(getattr(meta,'dof_names',[]) or getattr(art,'dof_names',[]))
    if not names:
        raw_paths=getattr(art,'dof_paths',None)
        if raw_paths:
            names=[str(p).rsplit('/',1)[-1] for p in raw_paths[0]]
    if len(names) != 8:
        raise AssertionError(f'Cannot identify eight articulation DOFs for rolling test: {names}')
    dof_velocities=np.zeros_like(art.get_dof_velocities(),dtype=np.float32)
    if dof_velocities.shape != (1,8):
        raise AssertionError(f'Unexpected articulation DOF velocity shape {dof_velocities.shape}')
    radius=.02794
    axle_indices={}
    for name in WHEEL_NAMES:
        matches=[i for i,n in enumerate(names) if str(n).rsplit('/',1)[-1] == 'Axle_'+name]
        if len(matches) != 1:
            raise AssertionError(f'Cannot identify wheel axle {name} in DOFs {names}')
        axle_indices[name]=matches[0]
        dof_velocities[0,matches[0]]=speed_mps/radius
    root_velocity=np.zeros((1,6),dtype=np.float32)
    root_velocity[0,0]=speed_mps
    index={p:i for i,p in enumerate(paths)}
    ci=index[RC_ROOT+'/Chassis']
    initial_poses=np.array(rigid_view.get_transforms(),dtype=float,copy=True)
    initial_chassis=initial_poses[ci,:3].copy()
    start_sim=elapsed[0]
    start_steps=physics_steps[0]
    start_wall=time.monotonic()
    # Tensor angular velocity units are radians/second. Set all joints together
    # so prismatic spring velocities stay zero while four axle speeds match vx/R.
    articulation_indices=np.asarray([0],dtype=np.uint32)
    art.set_root_velocities(root_velocity,articulation_indices)
    art.set_dof_velocities(dof_velocities,articulation_indices)
    samples=[]
    rotations_rad={name:0.0 for name in WHEEL_NAMES}
    previous_t=0.0
    rolling_step_count=0
    while True:
        if time.monotonic()-start_wall > wall_timeout_s:
            raise TimeoutError('Rolling validation did not finish within its wall time bound')
        if stepping_mode == 'timeline':
            if not timeline.is_playing():
                raise RuntimeError('Timeline stopped during rolling validation')
            await app.next_update_async()
        else:
            simulation.simulate(dt,elapsed[0])
            simulation.fetch_results()
            elapsed[0]+=dt
            physics_steps[0]+=1
            rolling_step_count+=1
            if rolling_step_count % 16 == 0:
                await app.next_update_async()
        poses=np.array(rigid_view.get_transforms(),dtype=float,copy=True)
        velocities=np.array(rigid_view.get_velocities(),dtype=float,copy=True)
        if not np.all(np.isfinite(poses)) or not np.all(np.isfinite(velocities)):
            if diagnostics is not None:
                diagnostics['rolling_failure_state']=_body_failure_snapshot(
                    paths,poses,velocities,elapsed[0]-start_sim,samples)
            raise AssertionError('Nonfinite state during rolling validation')
        if np.max(np.abs(poses[:,:3]))>3 or np.max(np.abs(velocities))>30:
            if diagnostics is not None:
                diagnostics['rolling_failure_state']=_body_failure_snapshot(
                    paths,poses,velocities,elapsed[0]-start_sim,samples)
            raise AssertionError('Unbounded body state during rolling validation')
        t=elapsed[0]-start_sim
        sample_dt=t-previous_t
        previous_t=t
        cr=_qmatrix_xyzw(poses[ci,3:7])
        cp=poses[ci,:3]
        wheel_z, gaps, travel, slip, omega={},{},{},{},{}
        for name in WHEEL_NAMES:
            wi=index[RC_ROOT+'/Wheel_'+name]
            hi=index[RC_ROOT+'/Carrier_'+name]
            wr=_qmatrix_xyzw(poses[wi,3:7])
            wheel_z[name]=float(poses[wi,2])
            pts=np.asarray(static['tire_collision_points_m'][name])
            gaps[name]=float(np.min(pts@wr[2,:]+poses[wi,2])-static['ground_top_z_m'])
            spring_relative=cr.T@(poses[hi,:3]-cp)-np.asarray(static['suspension_nominal_m'][name])
            travel[name]=float(spring_relative[2])
            omega[name]=float(velocities[wi,3:]@wr[:,1])
            slip[name]=float(velocities[wi,0]-omega[name]*radius)
            rotations_rad[name]+=omega[name]*sample_dt
        samples.append(dict(t_s=t,chassis_world_m=cp.tolist(),
            chassis_forward_speed_mps=float(velocities[ci,0]),
            chassis_tilt_deg=math.degrees(math.acos(float(np.clip(cr[2,2],-1,1)))),
            wheel_center_height_m=wheel_z,tire_floor_gap_m=gaps,
            suspension_m=travel,longitudinal_slip_mps=slip,
            axle_angular_speed_radps=omega))
        if t>=duration_s:
            break
    if len(samples)<30:
        raise AssertionError(f'Rolling test has only {len(samples)} pose samples')
    tail=[s for s in samples if s['t_s']>=.25]
    heights=np.asarray([list(s['wheel_center_height_m'].values()) for s in tail])
    gaps=np.asarray([list(s['tire_floor_gap_m'].values()) for s in tail])
    travel=np.asarray([list(s['suspension_m'].values()) for s in samples])
    slip=np.asarray([list(s['longitudinal_slip_mps'].values()) for s in tail])
    height_ripple=np.ptp(heights,axis=0)
    forward_distance=float(samples[-1]['chassis_world_m'][0]-initial_chassis[0])
    lateral_drift=float(samples[-1]['chassis_world_m'][1]-initial_chassis[1])
    mean_speed=float(np.mean([s['chassis_forward_speed_mps'] for s in tail]))
    revolutions={name:rotations_rad[name]/(2*math.pi) for name in WHEEL_NAMES}
    checks=[
        dict(name='actual_rolling_duration_at_least_2s',passed=samples[-1]['t_s']>=duration_s,measured_s=samples[-1]['t_s']),
        dict(name='forward_distance_at_least_half_meter',passed=forward_distance>=.5,measured_m=forward_distance),
        dict(name='each_wheel_rotates_at_least_3_revolutions',passed=min(revolutions.values())>=3,measured=revolutions),
        dict(name='mean_forward_speed_within_10percent',passed=abs(mean_speed-speed_mps)<=.1*speed_mps,measured_mps=mean_speed,target_mps=speed_mps),
        dict(name='rolling_wheel_height_ripple_below_0_1mm',passed=bool(np.max(height_ripple)<.0001),measured_peak_to_peak_m=dict(zip(WHEEL_NAMES,height_ripple.tolist()))),
        dict(name='rolling_tire_floor_gap_within_0_3mm',passed=bool(np.max(np.abs(gaps))<.0003),measured_range_m=[float(gaps.min()),float(gaps.max())]),
        dict(name='rolling_suspension_travel_below_1mm',passed=bool(np.max(np.abs(travel))<.001),measured_max_abs_m=float(np.max(np.abs(travel)))),
        dict(name='mean_rolling_slip_below_0_02mps',passed=float(np.mean(np.abs(slip)))<.02,measured_mps=float(np.mean(np.abs(slip)))),
        dict(name='lateral_drift_below_1mm',passed=abs(lateral_drift)<.001,measured_m=lateral_drift),
        dict(name='rolling_chassis_tilt_below_0_1degree',passed=max(s['chassis_tilt_deg'] for s in tail)<.1,measured_deg=max(s['chassis_tilt_deg'] for s in tail)),
    ]
    return dict(passed=all(c['passed'] for c in checks),checks=checks,
        test_method='Actual nine-body car: one-time root vx=.3m/s and axle omega=vx/R, followed by free rolling on the existing floor.',
        transient_excluded_for_ripple_s=.25,initial_velocity_mps=speed_mps,
        initial_axle_speed_radps=speed_mps/radius,dof_names=names,
        simulated_duration_s=samples[-1]['t_s'],physics_steps=physics_steps[0]-start_steps,
        pose_samples=len(samples),wall_duration_s=time.monotonic()-start_wall,
        final=samples[-1],samples=samples[::max(1,len(samples)//120)])


async def validate_rc_scene(ini_path='/home/shin/Desktop/RC_Car_Isaac/source/f110_rgb.ini',
        output_path='/home/shin/Desktop/RC_Car_Isaac/validation.json',
        duration_s=10.0, wall_timeout_s=240.0, stepping_mode='timeline', rolling_test=True):
    """Run the normal Play timeline and sample actual PhysX body state.

    The default mode tests normal Play behavior and samples after app updates;
    several physics steps can happen per sample. Optional mode 'manual' keeps
    the timeline stopped and uses simulate/fetch_results at the authored rate,
    sampling each physics step and yielding every16 steps. Both are stationary
    settling tests. By default a further2s test initializes motion once and
    measures actual car free rolling. Powered driving, steering, obstacles,
    ROS and sensors remain outside these tests.
    """
    import numpy as np
    import omni.kit.app
    import omni.physx
    import omni.physics.tensors
    import omni.timeline
    import omni.usd
    from pxr import PhysxSchema, UsdUtils

    context = omni.usd.get_context()
    app = omni.kit.app.get_app()
    timeline = omni.timeline.get_timeline_interface()
    if stepping_mode not in ('timeline','manual'):
        raise ValueError("stepping_mode must be 'timeline' or 'manual'")
    if timeline.is_playing():
        raise RuntimeError('Validation requires a stopped saved scene; stop the timeline first.')
    stage = context.get_stage()
    if stage is None or not stage.GetRootLayer().realPath:
        raise RuntimeError('Validation requires a saved scene.')
    scene_path = stage.GetRootLayer().realPath
    if stage.GetRootLayer().dirty:
        raise RuntimeError('Save the scene before validation so it can be restored exactly.')
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    result = dict(status='running', passed=False, scene=scene_path,
                  ini_source=str(ini_path), requested_simulated_duration_s=duration_s,
                  stepping_mode=stepping_mode,
                  rolling_test_requested=bool(rolling_test),
                  test_method=('Normal Play timeline; real PhysX tensor poses after app updates, physics-step clock.'
                      if stepping_mode == 'timeline' else
                      'Explicit PhysX attach_stage/simulate/fetch_results; real tensor poses every physics step.'),
                  limitation='Stationary settling and initial-velocity free rolling only. Timeline samples span several physics substeps; powered driving/steering/ROS/sensors are not tested.')

    def save():
        temp = output_path.with_suffix('.json.tmp')
        temp.write_text(json.dumps(result, indent=2, ensure_ascii=False, allow_nan=False), encoding='utf-8')
        temp.replace(output_path)

    save()
    sim_view = None
    simulation = omni.physx.get_physx_simulation_interface()
    manually_attached = False
    physics_subscription = None
    try:
        static = validate_static(stage, ini_path)
        result['static'] = {k:v for k,v in static.items() if k != 'tire_collision_points_m'}
        save()
        if not static['passed']:
            raise AssertionError('Static validation failed; see named checks in validation.json.')
        scene_api = PhysxSchema.PhysxSceneAPI(stage.GetPrimAtPath('/World/PhysicsScene'))
        if any(g['collider_type']=='native_cylinder' for g in static['tire_geometry'].values()):
            import carb.settings
            approximate_cylinders=carb.settings.get_settings().get('/physics/collisionApproximateCylinders')
            gpu_dynamics=scene_api.GetEnableGPUDynamicsAttr().Get()
            result['analytic_cylinder_runtime']=dict(
                approximate_cylinders_setting=approximate_cylinders,
                gpu_dynamics=gpu_dynamics,
                passed=approximate_cylinders is False and gpu_dynamics is False)
            if not result['analytic_cylinder_runtime']['passed']:
                raise AssertionError('Analytic Cylinder validation requires cylinder approximation=False and GPU dynamics=False')
        physics_hz = int(scene_api.GetTimeStepsPerSecondAttr().Get())
        if physics_hz < 60:
            raise ValueError(f'Unexpected authored physics rate: {physics_hz}Hz')
        dt = 1.0/physics_hz
        total_steps = math.ceil(duration_s*physics_hz)
        step_count = 0
        elapsed = [0.0]
        physics_steps = [0]
        stage_id = UsdUtils.StageCache.Get().Insert(stage).ToLongInt()
        if stepping_mode == 'manual':
            if not simulation.attach_stage(stage_id):
                raise RuntimeError(f'PhysX rejected explicit USD stage attachment {stage_id}')
            manually_attached = True
            if simulation.get_attached_stage() != stage_id:
                raise RuntimeError('Attached PhysX stage does not match the saved validation scene')
        else:
            def tick(dt):
                elapsed[0] += float(dt)
                physics_steps[0] += 1
            physics_subscription = omni.physx.get_physx_interface().subscribe_physics_step_events(tick)
            timeline.play()
            for _ in range(120):
                await app.next_update_async()
                if physics_steps[0] > 0:
                    break
            if physics_steps[0] == 0:
                raise RuntimeError('Normal Play produced no physics steps after120 Kit updates')
        result['physics_hz'] = physics_hz
        result['tensor_stage_id'] = stage_id
        sim_view = omni.physics.tensors.create_simulation_view('numpy', stage_id=stage_id)
        sim_view.set_subspace_roots('/')
        rigid_view = sim_view.create_rigid_body_view(RC_ROOT+'/*')
        paths = list(rigid_view.prim_paths)
        expected_paths = list(static['body_properties'])
        if set(paths) != set(expected_paths):
            raise AssertionError(f'Tensor body paths mismatch: {paths}; expected {expected_paths}')
        index = {path:i for i,path in enumerate(paths)}
        chassis_i = index[RC_ROOT+'/Chassis']
        samples = []
        max_travel = {name:0.0 for name in WHEEL_NAMES}
        max_lateral_error = {name:0.0 for name in WHEEL_NAMES}
        max_axle_error = {name:0.0 for name in WHEEL_NAMES}
        max_linear_speed = max_angular_speed = max_tilt = 0.0
        start_wall = time.monotonic()
        last_save = start_wall
        properties = static['body_properties']
        masses = np.asarray([properties[p]['mass_kg'] for p in paths])
        local_com = np.asarray([properties[p]['com_local_m'] for p in paths])
        while (stepping_mode == 'timeline' or step_count < total_steps):
            if time.monotonic()-start_wall > wall_timeout_s:
                raise TimeoutError(f'Only {elapsed[0]:.4f}s simulated in {wall_timeout_s}s wall time.')
            if stepping_mode == 'manual':
                if timeline.is_playing():
                    raise RuntimeError('Timeline became playing during explicit PhysX stepping')
                simulation.simulate(dt, elapsed[0])
                simulation.fetch_results()
                step_count += 1
                physics_steps[0] = step_count
                elapsed[0] = step_count*dt
            elif not timeline.is_playing():
                raise RuntimeError('Timeline stopped before physical validation completed')
            poses = np.array(rigid_view.get_transforms(), dtype=float, copy=True)
            velocities = np.array(rigid_view.get_velocities(), dtype=float, copy=True)
            if poses.shape != (9,7) or velocities.shape != (9,6):
                raise AssertionError(f'Unexpected tensor shapes: {poses.shape}, {velocities.shape}')
            if not np.all(np.isfinite(poses)) or not np.all(np.isfinite(velocities)):
                result['runtime_failure_state']=_body_failure_snapshot(
                    paths,poses,velocities,elapsed[0],samples)
                raise AssertionError('Nonfinite PhysX body state (NaN/Inf).')
            if np.max(np.abs(poses[:,:3])) > 2 or np.max(np.abs(velocities)) > 20:
                result['runtime_failure_state']=_body_failure_snapshot(
                    paths,poses,velocities,elapsed[0],samples)
                result['runtime_failure_state'].update(
                    max_absolute_position_m=float(np.max(np.abs(poses[:,:3]))),
                    max_linear_speed_mps=float(np.max(np.linalg.norm(velocities[:,:3],axis=1))),
                    max_angular_speed_radps=float(np.max(np.linalg.norm(velocities[:,3:],axis=1))))
                raise AssertionError('Explosive/unbounded physical body state.')
            rotations = [_qmatrix_xyzw(q) for q in poses[:,3:7]]
            cr = rotations[chassis_i]
            cp = poses[chassis_i,:3]
            tilt = math.degrees(math.acos(float(np.clip(cr[2,2],-1,1))))
            max_tilt = max(max_tilt, tilt)
            max_linear_speed = max(max_linear_speed, float(np.max(np.linalg.norm(velocities[:,:3],axis=1))))
            max_angular_speed = max(max_angular_speed, float(np.max(np.linalg.norm(velocities[:,3:],axis=1))))
            com = sum(masses[i]*(poses[i,:3]+rotations[i]@local_com[i]) for i in range(9))/sum(masses)
            chassis_clearance = float(np.min(np.asarray(static['chassis_collision_points_m'])@cr[2,:]+cp[2])
                                      -static['ground_top_z_m'])
            travel, bottoms = {}, {}
            for name in WHEEL_NAMES:
                hi, wi = index[RC_ROOT+'/Carrier_'+name], index[RC_ROOT+'/Wheel_'+name]
                relative = cr.T@(poses[hi,:3]-cp)
                error = relative-np.asarray(static['suspension_nominal_m'][name])
                travel[name] = float(error[2])
                max_travel[name] = max(max_travel[name], abs(float(error[2])))
                max_lateral_error[name] = max(max_lateral_error[name],float(np.linalg.norm(error[:2])))
                max_axle_error[name] = max(max_axle_error[name], float(np.linalg.norm(poses[hi,:3]-poses[wi,:3])))
                pts = np.asarray(static['tire_collision_points_m'][name])
                bottoms[name] = float(np.min(pts@rotations[wi][2,:]+poses[wi,2])-static['ground_top_z_m'])
            samples.append(dict(t_s=elapsed[0], com_world_m=com.tolist(),
                chassis_origin_world_m=cp.tolist(), chassis_tilt_deg=tilt,
                chassis_collider_floor_clearance_m=chassis_clearance,
                suspension_m=travel, tire_floor_gap_m=bottoms,
                maximum_body_speed_mps=float(np.max(np.linalg.norm(velocities[:,:3],axis=1)))))
            if time.monotonic()-last_save > 2:
                result['runtime_progress'] = dict(simulated_s=elapsed[0],pose_samples=len(samples),
                                                  physics_steps=physics_steps[0])
                save()
                last_save=time.monotonic()
            if stepping_mode == 'timeline' and elapsed[0] >= duration_s:
                break
            if stepping_mode == 'timeline' or step_count % 16 == 0:
                await app.next_update_async()
        if len(samples) < 30:
            raise AssertionError(f'Insufficient actual runtime pose samples: {len(samples)}')
        last = samples[-1]
        tail = [s for s in samples if s['t_s'] >= max(0,elapsed[0]-2)]
        tail_com = np.asarray([s['com_world_m'] for s in tail])
        tail_gap = np.asarray([list(s['tire_floor_gap_m'].values()) for s in tail])
        tail_speed = max(s['maximum_body_speed_mps'] for s in tail)
        tail_tilt = max(s['chassis_tilt_deg'] for s in tail)
        tail_clearance = min(s['chassis_collider_floor_clearance_m'] for s in tail)
        checks = [
            dict(name='simulated_at_least_10s',passed=elapsed[0]>=duration_s,measured=elapsed[0]),
            dict(name='finite_bounded_states',passed=True,pose_samples=len(samples)),
            dict(name='suspension_sampled_travel_below_1mm',passed=max(max_travel.values())<.001,measured_m=max_travel),
            dict(name='spring_lateral_constraint_below_0_1mm',passed=max(max_lateral_error.values())<.0001,measured_m=max_lateral_error),
            dict(name='axle_attachment_error_below_0_1mm',passed=max(max_axle_error.values())<.0001,measured_m=max_axle_error),
            dict(name='settled_tire_floor_gap_within_0_3mm',passed=bool(np.max(np.abs(tail_gap))<.0003),measured_range_m=[float(tail_gap.min()),float(tail_gap.max())]),
            dict(name='settled_chassis_clearance_above_0_5mm',passed=tail_clearance>.0005,measured_m=tail_clearance),
            dict(name='settled_com_height_within_0_3mm',passed=bool(np.max(np.abs(tail_com[:,2]-static['expected_com_height_m']))<.0003),measured_range_m=[float(tail_com[:,2].min()),float(tail_com[:,2].max())]),
            dict(name='settled_chassis_tilt_below_0_1degree',passed=tail_tilt<.1,measured_deg=tail_tilt),
            dict(name='settled_com_drift_below_0_1mm',passed=float(np.linalg.norm(np.ptp(tail_com,axis=0)))<.0001,measured_xyz_range_m=np.ptp(tail_com,axis=0).tolist()),
            dict(name='settled_body_speed_below_1mm_per_s',passed=tail_speed<.001,measured_mps=tail_speed),
        ]
        result['runtime'] = dict(passed=all(c['passed'] for c in checks), checks=checks,
            simulated_duration_s=elapsed[0], wall_duration_s=time.monotonic()-start_wall,
            pose_samples=len(samples), physics_steps=physics_steps[0], final=last, maximum_sampled_tilt_deg=max_tilt,
            maximum_sampled_linear_speed_mps=max_linear_speed,
            maximum_sampled_angular_speed_radps=max_angular_speed,
            samples=samples[::max(1,len(samples)//100)]+[last])
        if rolling_test:
            result['phase']='rolling'
            save()
            result['rolling']=await _validate_car_rolling(sim_view,rigid_view,paths,static,
                app,timeline,elapsed,physics_steps,simulation,dt,stepping_mode,diagnostics=result)
        result['passed'] = (result['static']['passed'] and result['runtime']['passed']
                            and (result['rolling']['passed'] if rolling_test else True))
        result['phase']='complete'
        result['status'] = 'passed' if result['passed'] else 'failed'
    except Exception as exc:
        result['status']='failed'
        result['error']=repr(exc)
        result['traceback']=traceback.format_exc()
    finally:
        physics_subscription=None
        if manually_attached:
            simulation.detach_stage()
        timeline.stop()
        for _ in range(2):
            await app.next_update_async()
        sim_view=None
        try:
            success = await context.open_stage_async(scene_path)
            if isinstance(success,tuple):
                success=success[0]
            if success is False:
                raise RuntimeError('open_stage_async returned false')
            for _ in range(2):
                await app.next_update_async()
            from omni.kit.viewport.utility import get_active_viewport
            viewport=get_active_viewport()
            if viewport:
                viewport.camera_path='/World/Camera'
            result['restored_saved_stopped_scene']=not timeline.is_playing()
        except Exception as exc:
            result['restored_saved_stopped_scene']=False
            result['restore_error']=repr(exc)
            result['passed']=False
            result['status']='failed'
        save()
    print('RC_CAR_VALIDATION '+json.dumps({k:result.get(k) for k in
          ('status','passed','error','restored_saved_stopped_scene')},ensure_ascii=False))
    return result
