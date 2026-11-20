def _check_tire_material_restore():
    import runpy,json,yaml
    from pathlib import Path
    from pxr import Usd,UsdGeom,UsdPhysics,UsdShade
    import live_gui
    source=Path.home()/'Desktop/RC_Car_Isaac/vesc_sim/tire_physics.py'
    prepare=runpy.run_path(str(source))['prepare_tire_physics']
    stage=Usd.Stage.CreateInMemory()
    native=UsdShade.Material.Define(stage,'/World/Looks/TireContact')
    material=UsdPhysics.MaterialAPI.Apply(native.GetPrim());material.CreateStaticFrictionAttr(.9);material.CreateDynamicFrictionAttr(.9)
    paths=[]
    for name in ('FL','FR','RL','RR'):
        UsdGeom.Xform.Define(stage,'/World/RC_Car/Wheel_'+name)
        collider=UsdGeom.Cylinder.Define(stage,'/World/RC_Car/Wheel_'+name+'/TireCollision')
        UsdPhysics.CollisionAPI.Apply(collider.GetPrim());paths.append(collider.GetPath())
        UsdShade.MaterialBindingAPI.Apply(collider.GetPrim()).Bind(native,UsdShade.Tokens.strongerThanDescendants,'physics')
    cfg=yaml.safe_load((source.parent/'config/tires.yaml').read_text())
    prepare(stage,'/World/RC_Car',cfg)
    enabled=[list(stage.GetPrimAtPath(p).GetRelationship('material:binding:physics').GetTargets()) for p in paths]
    prepare(stage,'/World/RC_Car',{'enabled':False})
    restored=[list(stage.GetPrimAtPath(p).GetRelationship('material:binding:physics').GetTargets()) for p in paths]
    report=dict(status='pass' if all(str(p[0])=='/World/Looks/DamviExplicitTireContact' for p in enabled) and all(str(p[0])=='/World/Looks/TireContact' for p in restored) else 'fail',active_stage_untouched=True)
    (live_gui.controller.output_dir/'tire_material_restore_check.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report))
_check_tire_material_restore()
