def _inspect_wheel_visuals():
    import json
    import carb.settings
    import live_gui
    from pxr import UsdGeom,UsdPhysics
    c=live_gui.controller
    stage=c.stage;cache=UsdGeom.XformCache()
    root=c.config['scene']['vehicle_root']
    view=c.view.create_rigid_body_view(root+'/*')
    poses=view.get_transforms()
    native={p:[float(v) for v in pose] for p,pose in zip(view.prim_paths,poses)}
    rows=[]
    for p in stage.Traverse():
        path=str(p.GetPath())
        if not path.startswith(root):continue
        if any(word in path.lower() for word in ('wheel','knuckle','steer')) or p.IsA(UsdGeom.Mesh):
            row={'path':path,'type':p.GetTypeName(),'active':p.IsActive()}
            if p.IsA(UsdGeom.Imageable):row['visibility']=UsdGeom.Imageable(p).ComputeVisibility()
            if p.IsA(UsdGeom.Xformable):
                matrix=cache.GetLocalToWorldTransform(p)
                row['usd_world_pose']={'position':list(matrix.ExtractTranslation()),'rotation_xyzw':list(matrix.ExtractRotationQuat().GetImaginary())+[matrix.ExtractRotationQuat().GetReal()]}
                row['xformOps']=[{'name':str(o.GetOpName()),'value':str(o.Get())} for o in UsdGeom.Xformable(p).GetOrderedXformOps()]
            row['native_pose']=native.get(path)
            row['rigid_body']=p.HasAPI(UsdPhysics.RigidBodyAPI)
            row['collision']=p.HasAPI(UsdPhysics.CollisionAPI)
            rows.append(row)
    settings=carb.settings.get_settings()
    print(json.dumps({'rows':rows,'settings':{k:settings.get(k) for k in ('/physics/updateToUsd','/physics/updateVelocitiesToUsd','/physics/updateParticlesToUsd','/physics/updateToFabric','/app/useFabricSceneDelegate','/physics/fabricEnabled')},'latest':c.rows[-1]},indent=2))
    if '_damvi_movie_contact_monitor' in globals():_damvi_movie_contact_monitor.duration_s=0.
_inspect_wheel_visuals()
