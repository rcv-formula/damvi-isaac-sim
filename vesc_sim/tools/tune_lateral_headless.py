"""Repeatable 200 Hz lateral experiments; one headless Isaac process, no ROS.

Each experiment opens a fresh in-memory session layer. Initial rolling velocity
is set once before the measured interval; thereafter only torques/forces act.
Original vehicle/map assets are never saved. See experiment manifests for inputs.
"""
import argparse
import copy
import json
import math
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--stage', type=Path, required=True)
parser.add_argument('--experiments', type=Path, required=True)
parser.add_argument('--output', type=Path, required=True)
args = parser.parse_args()
args.output.mkdir(parents=True, exist_ok=True)
(args.output/'error.txt').unlink(missing_ok=True)
experiments = json.loads(args.experiments.read_text())

from isaacsim import SimulationApp
app = SimulationApp({'headless': True, 'width': 320, 'height': 240,
                     'disable_viewport_updates': True})
failed = False

try:
    import numpy as np
    import omni.usd
    import omni.physx
    import omni.physics.tensors
    import carb.settings
    from pxr import Gf, Usd, UsdGeom, UsdPhysics, UsdShade, UsdUtils, PhysxSchema
    from native import NativeController
    from steering import load_configuration, SteeringController
    from steering_scene import prepare_front_steering
    from wheel_collision import prepare_wheel_collisions

    settings = carb.settings.get_settings()
    settings.set('/physics/physxDispatcher', False)
    settings.set('/persistent/physics/numThreads', 0)
    settings.set('/physics/updateToUsd', True)
    settings.set('/physics/collisionApproximateCylinders', False)
    base = load_configuration(ROOT/'config/vesc_sim.yaml')
    context = omni.usd.get_context()
    simulator = omni.physx.get_physx_simulation_interface()
    results = []
    for number, experiment in enumerate(experiments):
        started = time.monotonic()
        config = copy.deepcopy(base)
        config['_steering'].update(experiment.get('steering', {}))
        config['drivetrain'].update(experiment.get('drivetrain', {}))
        config['_runtime'].update(experiment.get('runtime', {}))
        hz = config['physics_hz']
        if hz != 200:
            raise ValueError('The calibration contract requires 200 Hz physics')
        dt = 1./hz
        context.open_stage(str(args.stage))
        stage = context.get_stage()
        stage.SetEditTarget(stage.GetSessionLayer())
        root = config['scene']['vehicle_root']
        for prim in stage.Traverse():
            if prim.HasAPI(UsdPhysics.CollisionAPI) and not prim.GetPath().HasPrefix(root):
                UsdPhysics.CollisionAPI(prim).CreateCollisionEnabledAttr(False)
        circle_spec = experiment.get('circle_pp')
        spawn_y = -float(circle_spec.get('radius_m',20.))-float(circle_spec.get('initial_outward_offset_m',.5)) if circle_spec else 0.
        stage.GetPrimAtPath(root).GetAttribute('xformOp:translate').Set(Gf.Vec3d(0,spawn_y,.07))
        ground = UsdGeom.Plane.Define(stage, '/World/CalibrationGround')
        ground.CreateAxisAttr('Z');ground.CreateWidthAttr(1000);ground.CreateLengthAttr(1000)
        UsdPhysics.CollisionAPI.Apply(ground.GetPrim())
        material = UsdShade.Material.Define(stage, '/World/CalibrationGroundMaterial')
        physics_material = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
        physics_material.CreateStaticFrictionAttr(.9)
        physics_material.CreateDynamicFrictionAttr(.9)
        physics_material.CreateRestitutionAttr(0.)
        UsdShade.MaterialBindingAPI.Apply(ground.GetPrim()).Bind(
            material, UsdShade.Tokens.weakerThanDescendants, 'physics')
        prepare_front_steering(stage, root, config['_steering'])
        prepare_wheel_collisions(stage, root, config['_runtime'])
        for name in ('FL','FR','RL','RR'):
            joint = stage.GetPrimAtPath(root+'/Joints/Axle_'+name)
            PhysxSchema.PhysxJointAPI.Apply(joint).CreateJointFrictionAttr(0.)
            if joint.HasAPI(UsdPhysics.DriveAPI, 'angular'):
                drive = UsdPhysics.DriveAPI(joint, 'angular')
                drive.CreateStiffnessAttr(0.);drive.CreateDampingAttr(0.);drive.CreateMaxForceAttr(0.)
        for prim in stage.Traverse():
            if prim.IsA(UsdPhysics.Scene):
                PhysxSchema.PhysxSceneAPI.Apply(prim).CreateTimeStepsPerSecondAttr(hz)
        # Optional force-based tire adapter is prepared before PhysX attachment.
        tire_config = experiment.get('tires')
        tire_model = None
        if tire_config is not None:
            from tire_physics import prepare_tire_physics, TirePhysics
            prepare_tire_physics(stage, root, tire_config)
        stage_id = UsdUtils.StageCache.Get().Insert(stage).ToLongInt()
        if not simulator.attach_stage(stage_id):
            raise RuntimeError('Failed to attach the calibration stage')
        simulator.simulate(dt, 0.);simulator.fetch_results()
        view = omni.physics.tensors.create_simulation_view('numpy', stage_id=stage_id)
        view.set_subspace_roots('/')
        art = view.create_articulation_view(config['scene']['chassis'])
        body = view.create_rigid_body_view(config['scene']['chassis'])
        wheels = view.create_rigid_body_view(root+'/Wheel_*')
        dof_names = list(art.shared_metatype.dof_names)
        rolling = [dof_names.index('Axle_'+n) for n in ('FL','FR','RL','RR')]
        steering_indices = [dof_names.index('Steer_'+n) for n in ('FL','FR')]
        ids = np.array([0], dtype=np.uint32)
        torque = np.zeros_like(art.get_dof_velocities(), dtype=np.float32)
        # Preserve authored suspension preload while updating steering targets.
        targets = np.array(art.get_dof_position_targets(), copy=True)
        motor = NativeController(config)
        steering = SteeringController(config['_steering'])
        if tire_config is not None:
            tire_model = TirePhysics(stage, view, config, tire_config)
        for i in range(100):
            if tire_model:tire_model.step(dt)
            simulator.simulate(dt, (i+1)*dt);simulator.fetch_results()
        replay = np.load(experiment['replay_npz']) if 'replay_npz' in experiment else None
        initial_speed = float(experiment.get('speed_mps', 1.))
        initial_body_speed = initial_speed
        initial_yaw_rate = 0.
        if replay is not None and experiment.get('initialize_reference_state',False):
            initial_speed = float(replay['wheel_speed_mps'][0])
            initial_body_speed = float(replay['initial_body_speed_mps']) if 'initial_body_speed_mps' in replay else float(replay['body_speed_reference_mps'][0])
            initial_yaw_rate = float(replay['initial_yaw_rate_rad_s']) if 'initial_yaw_rate_rad_s' in replay else float(replay['yaw_rate_reference_rad_s'][0])
            first_command = float(replay['steering_input_rad'][0])
            for _ in range(100):
                initial_steering = steering.step(first_command,dt)
            initial_positions = np.array(art.get_dof_positions(),copy=True)
            initial_positions[0,steering_indices] = [initial_steering.left,initial_steering.right]
            targets[0,steering_indices] = initial_positions[0,steering_indices]
            art.set_dof_positions(initial_positions,ids)
            art.set_dof_position_targets(targets,ids)
        initial_dofs = np.zeros_like(torque)
        initial_dofs[0, rolling] = initial_speed/config['drivetrain']['radius']
        art.set_dof_velocities(initial_dofs, ids)
        art.set_root_velocities(np.array([[initial_body_speed,0,0,0,0,initial_yaw_rate]], dtype=np.float32), ids)
        rows = []
        error = None
        circle = None
        if circle_spec:
            from lateral_circle_controller import CircleController
            circle = CircleController(config['_steering'], radius_m=float(circle_spec.get('radius_m',20.)),
                path_tangent_gain=float(circle_spec.get('path_tangent_gain', 0.)),
                understeer_rad_per_mps2=float(circle_spec.get('understeer_rad_per_mps2', 0.)),
                pose_preview_s=float(circle_spec.get('pose_preview_s', 0.)),
                min_lookahead=float(circle_spec.get('min_lookahead', .6)),
                max_lookahead=float(circle_spec.get('max_lookahead', 1.2)),
                lookahead_ratio=float(circle_spec.get('lookahead_ratio', 2.)))
        duration = float(experiment.get('duration_s', 10.))
        for i in range(round(duration*hz)):
            t = i*dt
            speed = initial_speed
            wheel_omega = np.asarray(art.get_dof_velocities()[0,rolling], dtype=float)
            if circle is not None:
                pose = np.asarray(body.get_transforms()[0],dtype=float)
                x,y,z,w = pose[3:]
                yaw = math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
                rear_pose = (pose[0]-.1*math.cos(yaw),pose[1]-.1*math.sin(yaw),yaw)
                command = circle.step(t,rear_pose,speed,
                    motor.erpm(wheel_omega)/config['speed_to_erpm_gain'])
            elif replay is not None:
                target_key = 'speed_target_mps' if 'speed_target_mps' in replay else 'wheel_speed_mps'
                speed = float(np.interp(t, replay['t'], replay[target_key]))
                command = float(np.interp(t, replay['t'], replay['steering_input_rad']))
            else:
                amplitude = float(experiment.get('amplitude_rad', .2))
                if experiment.get('signal', 'step') == 'sine':
                    command = amplitude*math.sin(2*math.pi*float(experiment.get('frequency_hz', .4))*max(0.,t-1.)) if t>=1. else 0.
                else:
                    command = 0. if t<1. else amplitude if t<3. else -amplitude if t<5. else 0.
            output = motor.step(speed*config['speed_to_erpm_gain'], wheel_omega, dt)
            steer = steering.step(command, dt)
            targets[0,steering_indices] = [steer.left,steer.right]
            art.set_dof_position_targets(targets, ids)
            torque.fill(0);torque[0,rolling] = output.torque
            art.set_dof_actuation_forces(torque, ids)
            if tire_model:tire_model.step(dt)
            simulator.simulate(dt, (101+i)*dt);simulator.fetch_results()
            if i%4 == 0:
                pose = np.asarray(body.get_transforms()[0], dtype=float)
                velocity = np.asarray(body.get_velocities()[0], dtype=float)
                x,y,z,w = pose[3:]
                yaw = math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
                roll = math.atan2(2*(w*x+y*z),1-2*(x*x+y*y))
                pitch = math.asin(np.clip(2*(w*y-z*x),-1.,1.))
                wheel_pose = np.asarray(wheels.get_transforms(), dtype=float)
                row = dict(t=t+dt, speed_target_mps=speed, input_rad=command,
                    wheel_speed_mps=float(motor.erpm(art.get_dof_velocities()[0,rolling])/config['speed_to_erpm_gain']),
                    position_m=pose[:3].tolist(), yaw_rad=yaw, roll_pitch_rad=[roll,pitch],
                    velocity_world=velocity.tolist(),
                    vx_body_mps=float(math.cos(yaw)*velocity[0]+math.sin(yaw)*velocity[1]),
                    vy_body_mps=float(-math.sin(yaw)*velocity[0]+math.cos(yaw)*velocity[1]),
                    yaw_rate_rad_s=float(velocity[5]),
                    actual_steering_rad=art.get_dof_positions()[0,steering_indices].tolist(),
                    wheel_omega_rad_s=art.get_dof_velocities()[0,rolling].tolist(),
                    motor_current_a=float(output.current),
                    applied_wheel_torques_nm=[float(v) for v in output.torque],
                    target_center_rad=float(steer.target_center),
                    wheel_center_z_m=wheel_pose[:,2].tolist())
                if tire_model:row['tires'] = tire_model.status()
                if circle:row['circle_controller'] = circle.status()
                rows.append(row)
                if not np.isfinite(pose).all() or not np.isfinite(velocity).all():
                    error = 'Non-finite physics state';break
                if min(wheel_pose[:,2])<config['drivetrain']['radius']-.003 or max(abs(roll),abs(pitch))>.35:
                    error = 'Wheel penetration or rollover threshold exceeded';break
            if i%400 == 0:
                app.update()
        result = dict(name=experiment['name'],experiment=experiment,status='fail' if error else 'pass',
            error=error,physics_hz=hz,wall_duration_s=time.monotonic()-started,
            original_usd_written=False,initial_velocity_only=True,rows=rows)
        destination = args.output/(experiment['name']+'.json')
        destination.write_text(json.dumps(result,indent=2)+'\n')
        results.append({k:v for k,v in result.items() if k!='rows'})
        (args.output/'summary.json').write_text(json.dumps(results,indent=2)+'\n')
        print('[LATERAL] '+json.dumps(results[-1]),flush=True)
        tire_model=None;wheels=None;body=None;art=None;view=None
        simulator.detach_stage()
        context.close_stage()
        if UsdUtils.StageCache.Get().Contains(stage):
            UsdUtils.StageCache.Get().Erase(stage)
        app.update()
except Exception:
    import traceback
    failed = True
    message = traceback.format_exc()
    (args.output/'error.txt').write_text(message)
    print(message, flush=True)
finally:
    app.close()
if failed:
    raise SystemExit(1)
