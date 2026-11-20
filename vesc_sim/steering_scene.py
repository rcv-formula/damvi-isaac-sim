"""Add two physical steering knuckles to the existing suspension/rolling joints.

Author in the caller's session layer. No wheel/chassis poses are overwritten at
runtime; actual angles come from articulation DOFs. Existing vehicle mass stays
constant by splitting each front carrier's mass between carrier and knuckle.
"""
import math


def prepare_front_steering(stage,root,config):
    from pxr import UsdGeom,UsdPhysics,PhysxSchema,Gf,Sdf
    centers={}
    for name in ('FL','FR','RL','RR'):
        carrier=stage.GetPrimAtPath(root+'/Carrier_'+name)
        centers[name]=carrier.GetAttribute('xformOp:translate').Get()
    wb=float((centers['FL'][0]+centers['FR'][0]-centers['RL'][0]-centers['RR'][0])/2)
    track=float(centers['FL'][1]-centers['FR'][1])
    if abs(wb-config['simulation_wheelbase_m'])>.001 or abs(track-config['simulation_front_track_m'])>.001:
        raise ValueError(f'USD geometry differs from steering config: wheelbase={wb}, front track={track}')
    for name in ('FL','FR'):
        carrier=stage.GetPrimAtPath(root+'/Carrier_'+name)
        mass=UsdPhysics.MassAPI(carrier)
        original=carrier.GetAttribute('damvi:originalCarrierMass')
        if not original:
            original=carrier.CreateAttribute('damvi:originalCarrierMass',Sdf.ValueTypeNames.Float)
            original.Set(mass.GetMassAttr().Get())
            carrier.CreateAttribute('damvi:originalCarrierInertia',Sdf.ValueTypeNames.Float3).Set(mass.GetDiagonalInertiaAttr().Get())
        full_mass=float(original.Get())
        inertia=carrier.GetAttribute('damvi:originalCarrierInertia').Get()
        half_inertia=Gf.Vec3f(*(float(x)*.5 for x in inertia))
        mass.CreateMassAttr(full_mass*.5);mass.CreateDiagonalInertiaAttr(half_inertia)
        knuckle_path=root+'/Knuckle_'+name
        knuckle=UsdGeom.Xform.Define(stage,knuckle_path)
        if not knuckle.GetPrim().GetAttribute('xformOp:translate'):
            knuckle.AddTranslateOp()
        knuckle.GetPrim().GetAttribute('xformOp:translate').Set(Gf.Vec3d(*centers[name]))
        UsdPhysics.RigidBodyAPI.Apply(knuckle.GetPrim())
        rigid=PhysxSchema.PhysxRigidBodyAPI.Apply(knuckle.GetPrim())
        rigid.CreateAngularDampingAttr(0.);rigid.CreateLinearDampingAttr(0.)
        km=UsdPhysics.MassAPI.Apply(knuckle.GetPrim())
        km.CreateMassAttr(full_mass*.5);km.CreateDiagonalInertiaAttr(half_inertia)
        km.CreateCenterOfMassAttr(Gf.Vec3f(0));km.CreatePrincipalAxesAttr(Gf.Quatf(1))
        joint=UsdPhysics.RevoluteJoint.Define(stage,root+'/Joints/Steer_'+name)
        joint.CreateBody0Rel().SetTargets([carrier.GetPath()])
        joint.CreateBody1Rel().SetTargets([Sdf.Path(knuckle_path)])
        joint.CreateAxisAttr('Z');joint.CreateCollisionEnabledAttr(False)
        joint.CreateLocalPos0Attr(Gf.Vec3f(0));joint.CreateLocalPos1Attr(Gf.Vec3f(0))
        joint.CreateLocalRot0Attr(Gf.Quatf(1));joint.CreateLocalRot1Attr(Gf.Quatf(1))
        joint.CreateLowerLimitAttr(-45.);joint.CreateUpperLimitAttr(45.)
        PhysxSchema.PhysxJointAPI.Apply(joint.GetPrim()).CreateJointFrictionAttr(0.)
        drive=UsdPhysics.DriveAPI.Apply(joint.GetPrim(),'angular')
        # YAML uses SI torque/radian gains. USD angular drive gains are per
        # degree, while tensor DOF targets and gains are expressed in radians.
        drive.CreateTypeAttr('force');drive.CreateStiffnessAttr(math.radians(config['joint_stiffness']))
        drive.CreateDampingAttr(math.radians(config['joint_damping']));drive.CreateMaxForceAttr(config['joint_max_torque_nm'])
        drive.CreateTargetPositionAttr(0.);drive.CreateTargetVelocityAttr(0.)
        axle=UsdPhysics.RevoluteJoint(stage.GetPrimAtPath(root+'/Joints/Axle_'+name))
        axle.CreateBody0Rel().SetTargets([Sdf.Path(knuckle_path)])
    return {'wheelbase_m':wb,'front_track_m':track,'mass_added_kg':0.,'steering_dofs':['Steer_FL','Steer_FR']}
