def _setup_steering_closeup():
    import live_gui,math
    from pxr import Gf,Usd,UsdGeom
    from omni.kit.viewport.utility import get_active_viewport
    c=live_gui.controller
    if '_damvi_follow_camera' in globals():_damvi_follow_camera.close()
    pose=c.body.get_transforms()[0];x,y,z,w=map(float,pose[3:]);yaw=math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
    f=Gf.Vec3d(math.cos(yaw),math.sin(yaw),0);side=Gf.Vec3d(-math.sin(yaw),math.cos(yaw),0)
    p=Gf.Vec3d(*map(float,pose[:3]));eye=p+.65*f-.5*side;eye[2]=.65;target=p+.1*f;target[2]=.035
    with Usd.EditContext(c.stage,c.stage.GetSessionLayer()):
        camera=UsdGeom.Camera.Define(c.stage,'/World/SteeringVisualCheckCamera')
        camera.CreateFocalLengthAttr(40.);camera.CreateHorizontalApertureAttr(36.);camera.CreateVerticalApertureAttr(20.25)
        camera.CreateClippingRangeAttr(Gf.Vec2f(.01,1000.))
        UsdGeom.Xformable(camera.GetPrim()).MakeMatrixXform().Set(Gf.Matrix4d().SetLookAt(eye,target,Gf.Vec3d(0,0,1)).GetInverse())
    get_active_viewport().camera_path=camera.GetPath()
    print('Close-up camera ready; vehicle poses and physics unchanged')
_setup_steering_closeup()
