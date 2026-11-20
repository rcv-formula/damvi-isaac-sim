def _check_steering_mesh():
    import asyncio,json,math,live_gui
    from pathlib import Path
    from pxr import Gf,UsdGeom
    from omni.kit.viewport.utility import get_active_viewport,capture_viewport_to_file
    c=live_gui.controller;cache=UsdGeom.XformCache();root=c.config['scene']['vehicle_root']
    chassis_inverse=cache.GetLocalToWorldTransform(c.stage.GetPrimAtPath(c.config['scene']['chassis'])).GetInverse()
    angles=[]
    for name in ('FL','FR'):
        mesh=c.stage.GetPrimAtPath(root+'/Wheel_'+name+'/Tire')
        matrix=cache.GetLocalToWorldTransform(mesh)*chassis_inverse
        axis=matrix.TransformDir(Gf.Vec3d(0,1,0))
        angles.append(math.atan2(-axis[0],axis[1]))
    tag='left' if c.steering_input>0 else 'right' if c.steering_input<0 else 'center'
    output=Path.home()/'Desktop/RC_Car_Isaac/vesc_sim/outputs/steering_mesh_check';output.mkdir(parents=True,exist_ok=True)
    data=dict(tag=tag,input_rad=c.steering_input,target_wheel_rad=[float(v) for v in c.position_targets[0,c.steer_indices]],actual_wheel_rad=[float(v) for v in c.art.get_dof_positions()[0,c.steer_indices]],visible_mesh_heading_rad=angles,error=c.error)
    (output/(tag+'.json')).write_text(json.dumps(data,indent=2)+'\n')
    capture=capture_viewport_to_file(get_active_viewport(),str(output/(tag+'.png')))
    print(json.dumps(data))
_check_steering_mesh()
