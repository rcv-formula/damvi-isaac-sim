"""Rebuild session adapters after a failed state; check zero-input contact."""
import asyncio
from pathlib import Path as ContactTestFile
import json
import math
import numpy as np
import omni.kit.app
import live_gui

if 'validation_gt_hook' in globals():
    validation_gt_hook.close()
    del validation_gt_hook
_contact_install_file=ContactTestFile('/home/shin/Desktop/RC_Car_Isaac/vesc_sim/tools/install_gui.py')
exec(compile(_contact_install_file.read_text(),str(_contact_install_file),'exec'),globals())

async def run_zero_input_contact_check():
    await vesc_gui_task
    c=live_gui.controller
    if c.error:
        print(c.error)
        return
    gt_file=ContactTestFile('/home/shin/Desktop/RC_Car_Isaac/vesc_sim/tools/publish_gui_ground_truth.py')
    exec(compile(gt_file.read_text(),str(gt_file),'exec'),globals())
    app=omni.kit.app.get_app()
    wheel_view=c.view.create_rigid_body_view('/World/RC_Car/Wheel_*')
    if wheel_view.count!=4:
        raise RuntimeError('Expected four physical wheel bodies')
    names=list(c.art.shared_metatype.dof_names)
    suspension=[names.index('Suspension_'+n) for n in ('FL','FR','RL','RR')]
    report={'test':'zero_input_recovery','physics_hz':c.config['physics_hz'],
        'steering_tensor_stiffness':c.art.get_dof_stiffnesses()[0,c.steer_indices].tolist(),
        'steering_tensor_damping':c.art.get_dof_dampings()[0,c.steer_indices].tolist(),
        'samples':[],'status':'running'}
    end=c.elapsed+15.
    while c.elapsed<end:
        await app.next_update_async()
        pose=c.body.get_transforms()[0]
        wheel=wheel_view.get_transforms()
        q=wheel[:,3:]
        axis_z=2*(q[:,1]*q[:,2]+q[:,0]*q[:,3])
        vertical=c.config['drivetrain']['radius']*np.sqrt(np.maximum(0.,1-axis_z**2))+.02*np.abs(axis_z)
        bottom=wheel[:,2]-vertical
        x,y,z,w=pose[3:]
        roll=math.atan2(2*(w*x+y*z),1-2*(x*x+y*y))
        pitch=math.asin(max(-1.,min(1.,2*(w*y-z*x))))
        s=c.art.get_dof_positions()[0,suspension]
        report['samples'].append({'time':c.elapsed,'chassis_z':float(pose[2]),
            'wheel_bottom_z':bottom.tolist(),'roll_deg':math.degrees(roll),
            'pitch_deg':math.degrees(pitch),'suspension_m':s.tolist(),
            'target_erpm':c.target,'wheel_omega':c.art.get_dof_velocities()[0,c.indices].tolist()})
        if c.elapsed>2. and (float(np.min(bottom))<-.003 or abs(roll)>.35 or abs(pitch)>.35):
            report['status']='fail'
            report['reason']='Contact penetration or excessive chassis tilt'
            c.timeline.pause()
            break
    if report['status']=='running':
        report['status']='pass'
    destination=c.output_dir/'zero_input_contact_check.json'
    destination.write_text(json.dumps(report,indent=2))
    print(json.dumps({k:v for k,v in report.items() if k!='samples'}))

contact_recovery_task=asyncio.ensure_future(run_zero_input_contact_check())
print('Session recovery and bounded zero-input contact check scheduled. No autonomous command is enabled.')
