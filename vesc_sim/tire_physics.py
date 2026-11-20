"""PhysX normal contacts plus optional C99 tangential tire forces.

Tire forces act once at each physical wheel's estimated contact point. The
rolling moment is produced by that force application, not added as a joint
torque. No runtime body pose, body velocity or wheel velocity is prescribed.
"""
import math
import numpy as np
from tire_model import TireModel, WHEEL_NAMES


def prepare_tire_physics(stage, root, config):
    from pxr import UsdPhysics, UsdShade, PhysxSchema
    if not config.get('enabled', False):
        # Reinstalling on the same in-memory stage must also restore the native
        # contact material after a previous explicit-tire run.
        native_path=config.get('native_contact_material','/World/Looks/TireContact')
        for name in WHEEL_NAMES:
            wheel=stage.GetPrimAtPath(root+'/Wheel_'+name)
            if not wheel:
                continue
            for prim in wheel.GetChildren():
                binding=prim.GetRelationship('material:binding:physics')
                if binding and any(str(p)=='/World/Looks/DamviExplicitTireContact' for p in binding.GetTargets()):
                    native=UsdShade.Material(stage.GetPrimAtPath(native_path))
                    if not native:
                        raise ValueError('Cannot restore the original tire contact material')
                    UsdShade.MaterialBindingAPI.Apply(prim).Bind(native,
                        UsdShade.Tokens.strongerThanDescendants,'physics')
        return {'enabled': False, 'mode': 'native_contact_friction'}
    # Validate all coefficients before changing the stage.
    TireModel(config)
    # PhysX uses the higher-priority combine mode. A positive-friction `max`
    # material could defeat the private tire's zero×road friction contract.
    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.MaterialAPI):
            mode = PhysxSchema.PhysxMaterialAPI(prim).GetFrictionCombineModeAttr().Get()
            if mode == 'max':
                mat = UsdPhysics.MaterialAPI(prim)
                if max(float(mat.GetStaticFrictionAttr().Get()), float(mat.GetDynamicFrictionAttr().Get())) > 0:
                    raise ValueError(f'Positive max-combine material conflicts with explicit tires: {prim.GetPath()}')
    material = UsdShade.Material.Define(stage, '/World/Looks/DamviExplicitTireContact')
    mat = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    mat.CreateStaticFrictionAttr(0.);mat.CreateDynamicFrictionAttr(0.);mat.CreateRestitutionAttr(0.)
    physx_material = PhysxSchema.PhysxMaterialAPI.Apply(material.GetPrim())
    physx_material.CreateFrictionCombineModeAttr('multiply')
    physx_material.CreateRestitutionCombineModeAttr('min')
    bound = []
    for name in WHEEL_NAMES:
        wheel = stage.GetPrimAtPath(root+'/Wheel_'+name)
        PhysxSchema.PhysxContactReportAPI.Apply(wheel).CreateThresholdAttr(0.)
        for prim in wheel.GetChildren():
            if prim.HasAPI(UsdPhysics.CollisionAPI):
                UsdShade.MaterialBindingAPI.Apply(prim).Bind(
                    material, UsdShade.Tokens.strongerThanDescendants, 'physics')
                bound.append(str(prim.GetPath()))
    return {'enabled': True, 'mode': config['model'], 'private_material': str(material.GetPath()),
            'tire_colliders': bound, 'native_tangential_friction': 0., 'normal_contact': 'PhysX'}


def rotate(q, vectors):
    xyz = q[:, :3]
    doubled_cross = 2.*np.cross(xyz, vectors)
    return vectors + q[:, 3, None]*doubled_cross + np.cross(xyz, doubled_cross)


class TirePhysics:
    def __init__(self, stage, view, vehicle_config, config):
        import carb
        import omni.physx
        from lidar import CollisionRaycaster
        self.config = config
        self.model = TireModel(config)
        if not self.model.enabled:
            raise ValueError('An explicit tire view requires enabled: true')
        self.root = vehicle_config['scene']['vehicle_root']
        self.wheels = view.create_rigid_body_view(self.root+'/Wheel_*')
        self.contacts = view.create_rigid_contact_view(self.root+'/Wheel_*')
        self.chassis = view.create_rigid_body_view(vehicle_config['scene']['chassis'])
        expected = [self.root+'/Wheel_'+n for n in WHEEL_NAMES]
        self.body_indices = [list(self.wheels.prim_paths).index(p) for p in expected]
        paths = list(self.contacts.sensor_paths)
        self.contact_indices = []
        for p in expected:
            matches = [i for i, path in enumerate(paths) if path == p or path.startswith(p+'/')]
            if len(matches) != 1:
                raise ValueError(f'Expected one active normal-contact sensor for {p}: {paths}')
            self.contact_indices.append(matches[0])
        self.ids = np.arange(self.wheels.count, dtype=np.uint32)
        self.radius = float(config['wheel_radius_m'])
        self.load = np.zeros(4)
        self.last_normal_force_n = np.zeros((4,3))
        self.latest_forces_world = np.zeros((4,3))
        self.latest_contact_positions = np.zeros((4,3))
        self.max_native_friction_n = 0.
        self.samples = 0
        self.ground_query = CollisionRaycaster(omni.physx.get_physx_scene_query_interface(),
            carb.Float3,1.,ignored_root=self.root)
        self.contact_tolerance = float(config.get('contact_separation_tolerance_m', .001))
        if not math.isfinite(self.contact_tolerance) or not 0<=self.contact_tolerance<=.005:
            raise ValueError('Invalid tire contact-separation tolerance')
        self.airborne_loads_rejected = 0
        self.load_filter_s = float(config.get('normal_load_filter_time_constant_s', .015))
        if not math.isfinite(self.load_filter_s) or self.load_filter_s < 0:
            raise ValueError('Invalid normal-load filter time constant')

    def step(self, dt):
        poses = np.asarray(self.wheels.get_transforms(), dtype=float)[self.body_indices]
        velocities = np.asarray(self.wheels.get_velocities(), dtype=float)[self.body_indices]
        normal = np.asarray(self.contacts.get_net_normal_contact_forces(dt), dtype=float)[self.contact_indices]
        friction = np.asarray(self.contacts.get_net_friction_contact_forces(dt), dtype=float)[self.contact_indices]
        self.max_native_friction_n = max(self.max_native_friction_n, float(np.max(np.linalg.norm(friction,axis=1))))
        if self.max_native_friction_n > 1e-3:
            raise RuntimeError('Native tangential friction remains active alongside explicit tires')
        if not all(np.isfinite(v).all() for v in (poses, velocities, normal)):
            raise ValueError('Non-finite wheel/contact state')
        q = poses[:,3:]
        axles = rotate(q, np.tile([0.,1.,0.], (4,1)))
        # Contact normals provide the local surface direction. Ground support
        # requires an upward normal; a horizontal wall contact is not traction.
        normal_length = np.linalg.norm(normal,axis=1)
        supported = (normal_length>1e-4) & (normal[:,2]>.5*normal_length)
        directions = np.tile([0.,0.,1.],(4,1))
        directions[supported] = normal[supported]/normal_length[supported,None]
        # Contact impulses describe the preceding solver interval. Confirm that
        # its support surface still touches the current wheel before applying a
        # tangential force, rather than carrying an old load into free flight.
        contact_positions = poses[:,:3]-self.radius*directions
        for i in range(4):
            if not supported[i]:
                continue
            origin = poses[i,:3]+.1*directions[i]
            distance = self.ground_query.range(origin,-directions[i],0.,.1+self.radius+.02)
            separation = distance-.1-self.radius
            if not math.isfinite(distance) or separation>self.contact_tolerance:
                supported[i] = False
                self.airborne_loads_rejected += 1
            else:
                contact_positions[i] = origin-distance*directions[i]
        raw_load = np.where(supported, np.maximum(normal[:,2],0.), 0.)
        alpha = -math.expm1(-dt/self.load_filter_s) if self.load_filter_s>0 else 1.
        self.load += alpha*(raw_load-self.load)
        self.load[~supported] = 0.
        lateral = axles-directions*np.sum(axles*directions,axis=1)[:,None]
        lateral /= np.maximum(np.linalg.norm(lateral,axis=1)[:,None],1e-12)
        forward = np.cross(lateral,directions)
        radial = contact_positions-poses[:,:3]
        contact_velocity = velocities[:,:3]+np.cross(velocities[:,3:],radial)
        vx = np.sum(velocities[:,:3]*forward,axis=1)
        vy = np.sum(contact_velocity*lateral,axis=1)
        # This effective tread speed makes Fx*(vx-R*omega) exactly equal to
        # force·contact_velocity, including camber and finite contact offsets.
        # VESC telemetry still uses the actual rolling-joint angular velocity.
        omega = -np.sum(np.cross(velocities[:,3:],radial)*forward,axis=1)/self.radius
        chassis_pose = np.asarray(self.chassis.get_transforms()[0],dtype=float)
        center = chassis_pose[:3]
        lever = contact_positions-center
        inverse_q = np.tile(chassis_pose[3:],(4,1))
        inverse_q[:,:3] *= -1.
        positions_body = rotate(inverse_q,lever)
        long_body = rotate(inverse_q,forward)
        lat_body = rotate(inverse_q,lateral)
        mode = self.config.get('contact_response_bound','full_spatial')
        if mode == 'ground_supported_planar':
            positions_body[:,2] = 0.
            long_body[:,2] = 0.;lat_body[:,2] = 0.
            long_body /= np.maximum(np.linalg.norm(long_body,axis=1)[:,None],1e-12)
            lat_body /= np.maximum(np.linalg.norm(lat_body,axis=1)[:,None],1e-12)
        elif mode != 'full_spatial':
            raise ValueError('Unknown contact-response bound')
        self.model.set_contact_geometry(positions_body,long_body,lat_body)
        outputs = self.model.step([dict(normal_load_n=float(self.load[i]),
            longitudinal_speed_mps=float(vx[i]), lateral_contact_speed_mps=float(vy[i]),
            wheel_angular_speed_rad_s=float(omega[i])) for i in range(4)],dt)
        forces = np.asarray([o.longitudinal_force_n for o in outputs])[:,None]*forward + \
                 np.asarray([o.lateral_force_n for o in outputs])[:,None]*lateral
        native_forces = np.zeros((self.wheels.count,3),dtype=np.float32)
        native_positions = np.zeros((self.wheels.count,3),dtype=np.float32)
        native_forces[self.body_indices] = forces
        native_positions[self.body_indices] = contact_positions
        self.wheels.apply_forces_and_torques_at_position(native_forces,None,native_positions,self.ids,True)
        self.last_normal_force_n = normal
        self.latest_forces_world = forces
        self.latest_contact_positions = contact_positions
        self.samples += 1

    def status(self):
        result = self.model.status()
        result.update(normal_load_n=self.load.tolist(), forces_world_n=self.latest_forces_world.tolist(),
            contact_positions_world_m=self.latest_contact_positions.tolist(), samples=self.samples,
            max_native_tangential_friction_n=self.max_native_friction_n,
            airborne_stale_loads_rejected=self.airborne_loads_rejected,
            normal_source='measured PhysX wheel contacts', pose_or_velocity_commands=False)
        return result
