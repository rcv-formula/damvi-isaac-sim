"""Standalone Isaac runner. All scene edits are in memory, no original USD saved.

Simulation samples wheel joint angular velocities and only writes joint torques;
it never sets chassis velocity or a wheel speed target to follow the command.
"""
import argparse
import csv
import json
import math
from pathlib import Path
import time

HERE=Path(__file__).resolve().parent
p=argparse.ArgumentParser()
p.add_argument('--stage',type=Path,required=True)
p.add_argument('--config',type=Path,default=HERE/'config/vesc_sim.yaml')
p.add_argument('--output',type=Path,default=HERE/'outputs/latest')
p.add_argument('--scenario',choices=['idle','forward_reverse','wheelspin','coast','brake'],default='idle')
p.add_argument('--duration',type=float,default=12.0)
p.add_argument('--ros',action='store_true');p.add_argument('--gui',action='store_true')
p.add_argument('--realtime',action='store_true')
p.add_argument('--flat-ground',action='store_true',help='Validation only: obstacle-free plane and original vehicle pose in session layer')
a=p.parse_args()
if a.duration<=0: p.error('duration must be positive')
a.output.mkdir(parents=True,exist_ok=True)
from isaacsim import SimulationApp
app=SimulationApp({'headless':not a.gui,'width':960,'height':640,'renderer':'RayTracedLighting'})


def target_for(t):
    if a.scenario in ('wheelspin','coast','brake'):return 2.0 if .5<=t<4 else 0.0
    if a.scenario=='forward_reverse':
        if .5<=t<4:return 1.5
        if 5.5<=t<9:return -1.5
    return 0.0


report={'status':'running','scenario':a.scenario,'stage':str(a.stage),'original_usd_written':False}
sim=None;view=None;art=None;body=None;bridge=None;attached=False;stage_id=None
try:
    import yaml
    import numpy as np
    import omni.usd,omni.timeline,omni.physx,omni.physics.tensors
    from pxr import Usd,UsdUtils,UsdPhysics,UsdGeom,UsdShade,PhysxSchema,Gf,Sdf
    from native import NativeController
    config=yaml.safe_load(a.config.read_text())
    hz=config['physics_hz'];dt=1.0/hz
    if hz<=0 or config['controller_hz']%hz:raise ValueError('controller_hz must be a positive integer multiple of physics_hz')
    if config['erpm_sign'] not in (-1,1) or config['speed_to_erpm_gain']<=0 or config['speed_to_erpm_offset']!=0:
        raise ValueError('Positive gain, sign +/-1, and zero ERPM offset required')
    ctrl=NativeController(config)
    if a.ros:
        from isaacsim.core.utils.extensions import enable_extension
        enable_extension('isaacsim.ros2.bridge')
        app.update()
        from ros_bridge import RosBridge
        bridge=RosBridge(config)
    ctx=omni.usd.get_context();ctx.open_stage(str(a.stage.resolve()));stage=ctx.get_stage()
    if not stage:raise RuntimeError('Cannot open vehicle stage')
    if UsdGeom.GetStageMetersPerUnit(stage)!=1 or UsdGeom.GetStageUpAxis(stage)!=UsdGeom.Tokens.z:
        raise ValueError('Expected meter / Z-up USD')
    stage.SetEditTarget(stage.GetSessionLayer())
    chassis=config['scene']['chassis'];root=config['scene']['vehicle_root']
    if not stage.GetPrimAtPath(chassis):raise ValueError('Vehicle chassis missing')
    if a.flat_ground:
        for prim in stage.Traverse():
            if prim.HasAPI(UsdPhysics.CollisionAPI) and not prim.GetPath().HasPrefix(root):
                UsdPhysics.CollisionAPI(prim).CreateCollisionEnabledAttr(False)
        stage.GetPrimAtPath(root).GetAttribute('xformOp:translate').Set(Gf.Vec3d(0,0,.07))
        plane=UsdGeom.Plane.Define(stage,'/World/VescTestGround')
        plane.CreateAxisAttr('Z');plane.CreateWidthAttr(100);plane.CreateLengthAttr(100)
        UsdPhysics.CollisionAPI.Apply(plane.GetPrim())
        material=UsdShade.Material.Define(stage,'/World/VescTestGroundMaterial')
        mat=UsdPhysics.MaterialAPI.Apply(material.GetPrim())
        mat.CreateStaticFrictionAttr(.9);mat.CreateDynamicFrictionAttr(.9);mat.CreateRestitutionAttr(0)
        UsdShade.MaterialBindingAPI.Apply(plane.GetPrim()).Bind(material,UsdShade.Tokens.weakerThanDescendants,'physics')
    report['flat_ground_validation']=a.flat_ground
    original_joint_friction={}
    for name in ('FL','FR','RL','RR'):
        joint_prim=stage.GetPrimAtPath(root+'/Joints/Axle_'+name)
        joint_api=PhysxSchema.PhysxJointAPI.Apply(joint_prim)
        original_joint_friction[name]=joint_api.GetJointFrictionAttr().Get()
        # The delivered USD has 200 here, effectively locking all wheel axles.
        # Our explicit, YAML-controlled passive losses replace that joint friction.
        joint_api.CreateJointFrictionAttr(0.)
        if joint_prim.HasAPI(UsdPhysics.DriveAPI,'angular'):
            drive=UsdPhysics.DriveAPI(joint_prim,'angular')
            drive.CreateStiffnessAttr(0.);drive.CreateDampingAttr(0.);drive.CreateMaxForceAttr(0.)
    report['original_joint_friction']=original_joint_friction
    # Existing wheels use real contact, not raycast tire forces.
    physics=[x for x in stage.Traverse() if x.IsA(UsdPhysics.Scene)]
    if len(physics)!=1:raise ValueError('Expected one PhysicsScene')
    PhysxSchema.PhysxSceneAPI.Apply(physics[0]).CreateTimeStepsPerSecondAttr(hz)
    import carb.settings
    carb.settings.get_settings().set('/physics/collisionApproximateCylinders',False)
    # A named rear axle frame is enough for optional later localization/TF.
    frame=UsdGeom.Xform.Define(stage,chassis+'/base_link')
    frame.AddTranslateOp().Set(Gf.Vec3d(*config['scene']['rear_axle_offset_m']))
    if a.scenario=='wheelspin':
        joint=UsdPhysics.FixedJoint.Define(stage,'/World/ValidationChassisAnchor')
        joint.CreateBody1Rel().SetTargets([Sdf.Path(chassis)])
        # Match the existing chassis world transform; do not snap the car to z=0.
        matrix=UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath(chassis))
        joint.CreateLocalPos0Attr(Gf.Vec3f(*matrix.ExtractTranslation()))
        joint.CreateLocalRot0Attr(Gf.Quatf(matrix.ExtractRotationQuat()))
        joint.CreateCollisionEnabledAttr(False)
    if a.scenario=='brake':
        # Remove tire friction for a deliberate wheel-lock / translating-body test.
        for prim in stage.Traverse():
            if prim.HasAPI(UsdPhysics.MaterialAPI):
                material=UsdPhysics.MaterialAPI(prim)
                material.CreateStaticFrictionAttr(0);material.CreateDynamicFrictionAttr(0)
    sim=omni.physx.get_physx_simulation_interface()
    stage_id=UsdUtils.StageCache.Get().Insert(stage).ToLongInt()
    if not sim.attach_stage(stage_id):raise RuntimeError('PhysX stage attachment failed')
    attached=True
    # Articulations are inserted into the PhysX scene on the first simulate.
    sim.simulate(dt,0.0);sim.fetch_results()
    view=omni.physics.tensors.create_simulation_view('numpy',stage_id=stage_id)
    view.set_subspace_roots('/')
    art=view.create_articulation_view(chassis)
    if art.count!=1:raise RuntimeError(f'Expected one articulation, got {art.count}')
    names=list(art.shared_metatype.dof_names)
    wheel_names=('FL','FR','RL','RR')
    indices=[names.index('Axle_'+n) for n in wheel_names]
    dofs=np.zeros_like(art.get_dof_velocities(),dtype=np.float32)
    art_indices=np.asarray([0],dtype=np.uint32)
    body=view.create_rigid_body_view(chassis)
    origin=np.array(body.get_transforms()[0,:3],copy=True)
    if a.scenario=='brake':
        # Test setup only: prescribe an initial rolling state once, before control.
        initial=np.zeros_like(dofs)
        initial[0,indices]=2.0/config['drivetrain']['radius']
        art.set_dof_velocities(initial,art_indices)
        art.set_root_velocities(np.asarray([[2.,0,0,0,0,0]],dtype=np.float32),art_indices)
    pub_period=1.0/config['ros']['publish_rate_hz'];next_pub=0.;next_sample=0.
    rows=[];wall_start=time.monotonic();last_update=wall_start
    max_center_error=0.;regen_peak=0.;brake_power_max=0.
    for i in range(math.ceil(a.duration*hz)):
        t=i*dt
        if bridge:
            target,brake=bridge.spin(t)
        else:
            target=target_for(t)*config['speed_to_erpm_gain'];brake=None
            if a.scenario=='brake':target=0.;brake=40.
        omega=np.asarray(art.get_dof_velocities()[0,indices],dtype=float)
        if not np.isfinite(omega).all():raise RuntimeError('Nonfinite wheel state')
        # coast intentionally bypasses motor braking after releasing its command.
        out=ctrl.step(target,omega,dt,brake,coast=a.scenario=='coast' and t>=4)
        torque=np.asarray(out.torque,dtype=np.float32)
        dofs.fill(0);dofs[0,indices]=torque
        art.set_dof_actuation_forces(dofs,art_indices)
        sim.simulate(dt,t);sim.fetch_results()
        now=t+dt
        actual_omega=np.asarray(art.get_dof_velocities()[0,indices],dtype=float)
        measured=ctrl.erpm(actual_omega)
        if bridge and now+1e-9>=next_pub:
            bridge.publish(now,measured);next_pub+=pub_period
        max_center_error=max(max_center_error,abs(out.center_speed_error))
        regen_peak=max(regen_peak,out.regen_power)
        if a.scenario=='brake':brake_power_max=max(brake_power_max,float(np.dot(omega,torque)))
        if now+1e-9>=next_sample:
            pose=np.asarray(body.get_transforms()[0]);vel=np.asarray(body.get_velocities()[0])
            if not np.isfinite(pose).all() or not np.isfinite(vel).all():raise RuntimeError('Nonfinite body state')
            rows.append({'t':now,'target_erpm':target,'ramped_erpm':ctrl.speed_output.target_erpm,
                'erpm':measured,'current_a':out.current,'battery_current_a':out.battery_current,
                'regen_w':out.regen_power,'electrical_brake':out.electrical_brake,
                'x':float(pose[0]),'y':float(pose[1]),'z':float(pose[2]),'vx':float(vel[0]),
                'wheel_equivalent_mps':measured/config['speed_to_erpm_gain'],
                'center_error_rad_s':out.center_speed_error,
                **{f'omega_{n}':float(v) for n,v in zip(wheel_names,actual_omega)}})
            next_sample+=pub_period
        if i%100==0:
            app.update()
            if not app.is_running():break
        if i%1000==0:
            (a.output/'progress.json').write_text(json.dumps({'simulated_s':now,'target_erpm':target,'erpm':measured,'current_a':out.current}))
        if a.realtime:
            delay=wall_start+now-time.monotonic()
            if delay>0:time.sleep(min(delay,0.02))
    if not rows:raise RuntimeError('No physical samples')
    csv_path=a.output/'trace.csv'
    with csv_path.open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=rows[0]);writer.writeheader();writer.writerows(rows)
    report.update(status='pass',physics_hz=hz,controller_hz=config['controller_hz'],samples=len(rows),
        simulated_s=rows[-1]['t'],wall_s=time.monotonic()-wall_start,
        erpm_gain=config['speed_to_erpm_gain'],max_center_error_rad_s=max_center_error,
        regen_peak_w=regen_peak,regen_j=ctrl.drive.regen_joules,
        final=rows[-1],trace=str(csv_path),base_link_prim=chassis+'/base_link',
        drivetrain='compliant center shaft; viscous front/rear diffs; coefficients uncalibrated',
        electrical_model='averaged current/voltage/loss envelope; no PWM, observer, BMS or thermal model')
    if a.scenario=='forward_reverse' and a.duration>=11:
        forward=[r for r in rows if 3<r['t']<4];reverse=[r for r in rows if 8<r['t']<9]
        report['checks']={'forward_motion':sum(r['vx'] for r in forward)/len(forward)>0.7,
            'reverse_motion':sum(r['vx'] for r in reverse)/len(reverse)<-0.7,
            'stops':abs(rows[-1]['vx'])<0.1,
            'regeneration':regen_peak>0,
            'forward_erpm':all(r['erpm']>0 for r in forward),
            'reverse_erpm':all(r['erpm']<0 for r in reverse)}
    elif a.scenario=='wheelspin' and a.duration>=4:
        spin=[r for r in rows if 2<r['t']<4]
        report['checks']={'chassis_fixed':max(abs(r['vx']) for r in spin)<0.01,
            'wheelspin_reported':max(r['wheel_equivalent_mps'] for r in spin)>0.5}
    elif a.scenario=='brake' and a.duration>=2:
        report['checks']={'body_still_sliding':rows[-1]['vx']>1.,
            'wheel_speed_low':abs(rows[-1]['wheel_equivalent_mps'])<0.1,
            'brake_never_drives':brake_power_max<1e-4}
    if 'checks' in report and not all(report['checks'].values()):report['status']='fail'
    if 'checks' in report:report['checks']={k:bool(v) for k,v in report['checks'].items()}
    if rows[-1]['t']<a.duration-dt:report['status']='incomplete'
except Exception:
    import traceback
    report.update(status='error',error=traceback.format_exc())
finally:
    (a.output/'report.json').write_text(json.dumps(report,indent=2))
    print('[VESC_SIM] '+json.dumps(report),flush=True)
    if bridge:bridge.close()
    body=None;art=None;view=None
    if sim and attached:sim.detach_stage()
    app.close()
if report['status']!='pass':raise SystemExit(1)
