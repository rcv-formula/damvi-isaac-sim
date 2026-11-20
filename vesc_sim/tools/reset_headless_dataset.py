"""Run inside Isaac via reset_dataset.py. Its caller supplies a reset_request dict."""
import asyncio


async def _reset_headless_dataset(request):
    import hashlib
    import json
    import math
    from pathlib import Path
    import time
    import traceback
    import live_gui
    import omni.kit.app
    import omni.usd

    folder = Path(live_gui.__file__).resolve().parent
    result_path = folder/'outputs/live_gui/dataset_reset.json'
    report = {'request_id':request['request_id'], 'status':'running'}

    def write():
        tmp = result_path.with_suffix('.tmp')
        tmp.write_text(json.dumps(report, indent=2)+'\n'); tmp.replace(result_path)

    write()
    try:
        import importlib.util
        import sys
        # Kit's fast importer can cache the project folder before deployment.
        spec=importlib.util.spec_from_file_location('dataset_scene',folder/'dataset_scene.py')
        dataset_scene=sys.modules.get('dataset_scene') or importlib.util.module_from_spec(spec)
        source=folder/'dataset_scene.py'
        exec(compile(source.read_bytes(),str(source),'exec'),dataset_scene.__dict__)
        sys.modules['dataset_scene']=dataset_scene
        c = live_gui.controller
        if not c.headless:
            raise RuntimeError('Dataset reset requires the headless ROS simulator')
        if abs(c.target)>1:
            raise RuntimeError('Stop the command publisher before resetting the dataset')
        dataset_scene.resolve_dataset(request['dataset'])
        if not all(math.isfinite(v) for v in request['spawn']) or len(request['spawn']) != 3:
            raise ValueError('Invalid requested spawn')
        if getattr(dataset_scene, '_validation_trace', None):
            dataset_scene._validation_trace.close()
            dataset_scene._validation_trace=None
        if getattr(dataset_scene, '_gt_hook', None):
            dataset_scene._gt_hook.close(); dataset_scene._gt_hook = None
        app = omni.kit.app.get_app()
        ctx = omni.usd.get_context()
        root_path = c.stage.GetRootLayer().realPath
        c.stop(); c.timeline.stop()
        for _ in range(3):
            await app.next_update_async()
        c.close()
        # A new session layer restores joint positions/velocities and sensor
        # histories even after a collision or timeline pause in the last lap.
        ok, error = await ctx.open_stage_async(root_path)
        if not ok:
            raise RuntimeError(error)
        stage = ctx.get_stage()
        report.update(dataset_scene.configure_dataset(stage, request['dataset'], request['spawn']))
        report['usd_sha256'] = hashlib.sha256(Path(report['usd']).read_bytes()).hexdigest()
        await live_gui.install()
        c = live_gui.controller
        if c.error or not c.bridge:
            raise RuntimeError(c.error or 'ROS bridge failed to start')
        gt = folder/'tools/publish_gui_ground_truth.py'
        ns = {'__name__':'damvi_validation_gt'}
        exec(compile(gt.read_text(), str(gt), 'exec'), ns)
        dataset_scene._gt_hook = ns['validation_gt_hook']
        source=folder/'episode_trace.py'
        trace_ns={}
        exec(compile(source.read_bytes(),str(source),'exec'),trace_ns)
        dataset_scene._validation_trace=trace_ns['EpisodeTrace'](c)
        timeout = time.monotonic()+10
        while (not c.rows or c.elapsed < .4) and time.monotonic()<timeout:
            await app.next_update_async()
        if not c.rows:
            raise RuntimeError('No measured rear-axle observation')
        row = c.rows[-1]; qx,qy,qz,qw = row['orientation_xyzw']
        vx,vy,vz = c.config['scene']['rear_axle_offset_m']
        tx,ty,tz = 2*(qy*vz-qz*vy),2*(qz*vx-qx*vz),2*(qx*vy-qy*vx)
        x = row['position'][0] + vx+qw*tx+qy*tz-qz*ty
        y = row['position'][1] + vy+qw*ty+qz*tx-qx*tz
        yaw = math.atan2(2*(qw*qz+qx*qy),1-2*(qy*qy+qz*qz))
        dx,dy,dyaw = x-request['spawn'][0],y-request['spawn'][1],yaw-request['spawn'][2]
        upright_z=1-2*(qx*qx+qy*qy)
        report.update(body_up_z=upright_z, body_z_m=row['position'][2],
                      ground_contact='horizontal plane at original slab top Z=0', measured_rear_xyyaw=[x,y,yaw], target_erpm=c.target, physics_hz=c.config['physics_hz'],
                      native_lateral_mu=[p.friction_lateral for p in c.tires.model.params],
                      lidar_scans=c.lidar.status(), imu=c.imu.status(), ground_truth_topic='/validation/ground_truth')
        if math.hypot(dx,dy)>.04 or abs(math.atan2(math.sin(dyaw),math.cos(dyaw)))>.04:
            raise RuntimeError(f'Unexpected measured spawn {x,y,yaw}')
        if upright_z<.98 or not .045<row['position'][2]<.10:
            raise RuntimeError('Spawn did not settle upright on the map floor')
        if not report['lidar_scans']['published_count'] or not report['imu']['published_count']:
            raise RuntimeError('Waiting for first LiDAR and IMU observations failed')
        if c.error or c.config['_surface']!=report['surface'] or c.config['physics_hz']!=200:
            raise RuntimeError('Simulator state disagrees with requested calibration')
        surface=report['surface']
        scale=surface['coefficient']/surface['reference_coefficient']
        for axis in ('longitudinal','lateral'):
            key='friction_'+axis
            expected=[c.config['_tires'][axle][key]*scale for axle in ('front','front','rear','rear')]
            actual=[getattr(p,key) for p in c.tires.model.params]
            if any(abs(a-b)>1e-7 for a,b in zip(actual,expected)):
                raise RuntimeError(f'Incorrect {axis} tire coefficients for this surface')
        if abs(c.target)>1:
            raise RuntimeError('Unexpected nonzero input just after spawn')
        subs=[s for s in c.bridge.node.subscriptions if s.topic_name in ('/cmd_vel',c.config['ros']['command_topic'])]
        report['input_qos']={s.topic_name:{'depth':s.qos_profile.depth,'history':s.qos_profile.history.name,
                              'reliability':s.qos_profile.reliability.name} for s in subs}
        c.write_status()
        report['status']='ready'
    except Exception:
        report['status']='failed';report['error']=traceback.format_exc()
    finally:
        write()


if 'damvi_reset_task' in globals() and not damvi_reset_task.done():
    raise RuntimeError('Another dataset reset is still running')
damvi_reset_task=asyncio.ensure_future(_reset_headless_dataset(reset_request))
