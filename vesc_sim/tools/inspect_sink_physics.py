"""Read-only live stage / articulation / contact surface forensic snapshot."""
def _inspect_sink_physics():
    import json, math
    from pathlib import Path
    import numpy as np
    import omni.usd, omni.physx, omni.timeline, carb.settings
    from pxr import Usd, UsdGeom, UsdPhysics, PhysxSchema
    import live_gui
    ctl=live_gui.controller
    stage=ctl.stage
    root=ctl.config['scene']['vehicle_root']
    cache=UsdGeom.XformCache()
    def cv(v):
        if v is None or isinstance(v,(str,int,float,bool)):return v
        if isinstance(v,dict):return {str(k):cv(x) for k,x in v.items()}
        try:return [cv(x) for x in v]
        except TypeError:return str(v)
    def info(p):
        d={'path':str(p.GetPath()),'type':p.GetTypeName(),'schemas':p.GetAppliedSchemas()}
        d['attributes']={a.GetName():cv(a.Get()) for a in p.GetAttributes() if
            a.GetName().startswith(('physics:','physx','drive:','xformOp:','visibility','purpose','radius','height','axis','size','extent'))}
        d['relationships']={r.GetName():cv(r.GetTargets()) for r in p.GetRelationships()}
        if p.IsA(UsdGeom.Xformable):
            mat=cache.GetLocalToWorldTransform(p)
            d['usd_world_translation']=cv(mat.ExtractTranslation())
            try:
                b=UsdGeom.BBoxCache(Usd.TimeCode.Default(),['default','render','proxy']).ComputeWorldBound(p).ComputeAlignedBox()
                d['usd_world_aabb']={'min':cv(b.GetMin()),'max':cv(b.GetMax())}
            except Exception as e:d['bounds_error']=str(e)
        return d
    report={'read_only':True,'scene':stage.GetRootLayer().identifier,
        'timeline_playing':omni.timeline.get_timeline_interface().is_playing(),
        'elapsed':ctl.elapsed,'latest':ctl.rows[-1] if ctl.rows else None,
        'car_prims':[],'scenes':[],'ground_rays':[],'errors':[]}
    setting_names=['/physics/collisionApproximateCylinders','/defaults/physics/collisionApproximateCylinders',
                   '/physics/collisionCylinderCustomGeometry','/physics/collisionApproximateCones']
    report['settings']={k:carb.settings.get_settings().get(k) for k in setting_names}
    report['collision_filters']=[]
    for p in stage.Traverse():
        if str(p.GetPath()).startswith(root):report['car_prims'].append(info(p))
        elif p.IsA(UsdPhysics.Scene):report['scenes'].append(info(p))
        if p.IsA(UsdPhysics.CollisionGroup) or p.HasAPI(UsdPhysics.FilteredPairsAPI):
            report['collision_filters'].append(info(p))
    original=Usd.Stage.Open(stage.GetRootLayer().identifier)
    report['original_file_parameters']=[info(p) for p in original.Traverse() if
        str(p.GetPath()).startswith(root) and
        (p.HasAPI(UsdPhysics.RigidBodyAPI) or '/Joints/' in str(p.GetPath()) or p.HasAPI(UsdPhysics.CollisionAPI))]
    try:
        art=ctl.art
        report['articulation']={
            'dof_names':list(art.shared_metatype.dof_names),
            'dof_positions':cv(art.get_dof_positions()),
            'dof_velocities':cv(art.get_dof_velocities()),
            'root_transforms':cv(art.get_root_transforms()),
            'root_velocities':cv(art.get_root_velocities())}
        for getter in ('get_dof_limits','get_dof_stiffnesses','get_dof_dampings','get_dof_max_forces',
                       'get_dof_armatures','get_dof_friction_coefficients','get_link_transforms'):
            try:report['articulation'][getter]=cv(getattr(art,getter)())
            except Exception as e:report['articulation'][getter]={'unavailable':str(e)}
        links=ctl.view.create_rigid_body_view(root+'/*')
        poses=np.asarray(links.get_transforms())
        report['bodies']={'prim_paths':cv(links.prim_paths),'poses':cv(poses),'velocities':cv(links.get_velocities())}
        for getter in ('get_masses','get_inertias','get_coms','get_disable_gravities','get_disable_simulations'):
            try:report['bodies'][getter]=cv(getattr(links,getter)())
            except Exception as e:report['bodies'][getter]={'unavailable':str(e)}
        query=omni.physx.get_physx_scene_query_interface()
        for path,pose in zip(links.prim_paths,poses):
            if 'Wheel' not in path and not path.endswith('Chassis'):continue
            hits=[]
            def collect(hit):
                collider=str(hit.collision)
                if not collider.startswith(root):
                    hits.append({'collision':collider,'rigidBody':str(hit.rigid_body),
                        'position':cv(hit.position),'normal':cv(hit.normal),'distance':hit.distance})
                return True
            query.raycast_all((float(pose[0]),float(pose[1]),float(pose[2])+1.),(0.,0.,-1.),5.,collect,True)
            hits.sort(key=lambda h:h['distance'])
            report['ground_rays'].append({'actor':path,'origin_body':cv(pose[:3]),'hits':hits[:8]})
            for hit in hits[:2]:
                p=stage.GetPrimAtPath(hit['collision'])
                if p:report.setdefault('hit_colliders',{})[str(p.GetPath())]=info(p)
    except Exception as e:
        import traceback
        report['errors'].append(traceback.format_exc())
    out=Path('/home/shin/Desktop/RC_Car_Isaac/vesc_sim/outputs/live_gui/sinking_physics.json')
    out.write_text(json.dumps(report,indent=2))
    print(json.dumps({'output':str(out),'latest':report['latest'],'body_count':len(report.get('bodies',{}).get('poses',[])),
        'ground_rays':report['ground_rays'],'errors':report['errors']}))
_inspect_sink_physics()
