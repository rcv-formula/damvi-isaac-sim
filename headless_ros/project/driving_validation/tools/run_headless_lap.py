#!/usr/bin/env python3
"""Reset Isaac, start the PF and control stack, then run one scored autonomous lap test."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile

ROOT=Path(__file__).resolve().parents[1]
SIM=ROOT.parent/'damvi_vesc_sim'
DATA=Path(os.environ.get('DAMVI_DATASET_ROOT', Path.home()/'Desktop/red-damvi dataset'))

from headless_route import dataset_inputs, prepare_route

# --existing observes a preconfigured simulator. Read its live surface and
# the last verified geometry on the server immediately before attaching.
LIVE_SCENE_QUERY = """
import json, time
from pathlib import Path
folder = Path.home()/'Desktop/RC_Car_Isaac/vesc_sim/outputs/live_gui'
status_file = folder/'status.json'
status = json.loads(status_file.read_text())
scene = json.loads((folder/'dataset_reset.json').read_text())
if (time.time()-status_file.stat().st_mtime > 15 or not status.get('headless_session')
        or status.get('error') or status.get('status') != 'running'):
    raise RuntimeError('No fresh running headless simulator status')
active = status.get('surface_friction', {})
if (active.get('selected_dataset') != scene['dataset']
        or active.get('coefficient') != scene['surface']['coefficient']):
    raise RuntimeError('Active simulator surface differs from last confirmed map reset')
print(json.dumps(scene))
"""



def install_prepared_surface(out, dataset):
    """Install the staged physical/planner input under the remote episode lock."""
    report = json.loads((out/'surface_selection.json').read_text())
    config = SIM.resolve()/'config/surface_friction.yaml'
    target = out/'selected_surface.yaml'
    active = config.read_bytes()
    staged = target.read_bytes()
    digest = hashlib.sha256(staged).hexdigest()
    if report['dataset'] != dataset or digest != report['surface_sha256']:
        raise ValueError('Prepared surface YAML, report and dataset must match')
    if staged != active:
        if hashlib.sha256(active).hexdigest() != report['remote_before_sha256']:
            raise ValueError('Isaac surface YAML changed during preparation. Its edits were preserved.')
        with tempfile.NamedTemporaryFile(dir=config.parent, delete=False) as f:
            f.write(staged)
            f.flush()
            os.fsync(f.fileno())
            os.fchmod(f.fileno(), config.stat().st_mode & 0o777)
        if config.read_bytes() != active:
            Path(f.name).unlink()
            raise ValueError('Isaac surface YAML was edited while installing a profile')
        Path(f.name).replace(config)
    (out/'surface_installed.sha256').write_text(digest+'\n')



def invoke(command, dest, env, timeout):
    with dest.open('w') as log:
        result=subprocess.run([str(s) for s in command],stdout=log,stderr=subprocess.STDOUT,
                              env=env,timeout=timeout)
    if result.returncode:
        raise RuntimeError(f'{dest.name} failed with exit {result.returncode}: {dest.read_text()[-800:]}')


def snapshot_physics(out, socket, scene, same_host=False):
    ssh = ['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=5']
    if socket:
        ssh += ['-S', socket]
    script = 'python3 ~/Desktop/RC_Car_Isaac/isaac_live_client.py ~/Desktop/RC_Car_Isaac/vesc_sim/tools/read_episode_trace.py'
    command = (['/usr/bin/python3', SIM.resolve().parent/'isaac_live_client.py', SIM/'tools/read_episode_trace.py']
               if same_host else ssh + ['shin@192.168.0.2', script])
    proc = subprocess.run(command, capture_output=True, text=True, timeout=18)
    if proc.returncode:
        raise RuntimeError(proc.stderr or proc.stdout[-600:])
    trace = json.loads(json.loads(proc.stdout)['output'])
    if trace['request_id'] != scene['request_id'] or trace['dataset'] != scene['dataset']:
        raise RuntimeError('The captured physics trace belongs to a different map reset')
    (out/'physics_trace.json').write_text(json.dumps(trace,separators=(',',':')))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('dataset')
    p.add_argument('cap',type=float)
    p.add_argument('profile',help='baseline, tuned, or absolute PP parameter overlay')
    p.add_argument('out',type=Path)
    p.add_argument('--socket',default=os.environ.get('DAMVI_SSH_SOCKET'))
    speed_input = p.add_mutually_exclusive_group()
    speed_input.add_argument('--speed-csv', type=Path)
    speed_input.add_argument('--speed-plan', choices=['auto', 'steer-brake', 'legacy'],
                             help='auto applies the grip, steering and brake envelope for speed caps of 3 m/s and above')
    p.add_argument('--laps',type=float,default=1.05)
    p.add_argument('--duration',type=float,default=70.)
    p.add_argument('--same-host',action='store_true',help='Run under server_ros.sh with the installed Jazzy workspace')
    p.add_argument('--existing',action='store_true',help='Observe a preconfigured, seeded stack; leave launch running in Stop')
    args=p.parse_args()
    if args.same_host and args.existing:
        p.error('--same-host owns the complete lap startup and shutdown')
    args.out.mkdir(exist_ok=True,parents=True)
    out=args.out.resolve()
    if (out/'lap.json').exists():
        p.error('output already contains a lap')
    env=dict(os.environ,ROS_DOMAIN_ID='0',PYTHONNOUSERSITE='1')
    if args.speed_csv:
        env['DAMVI_PATH_CSV']=str(args.speed_csv.resolve(strict=True))
    env['DAMVI_SPEED_PLAN'] = args.speed_plan or env.get('DAMVI_SPEED_PLAN', 'auto')
    if env.get('DAMVI_PATH_CSV') and env['DAMVI_SPEED_PLAN'] != 'auto':
        p.error('Select one of --speed-csv / DAMVI_PATH_CSV or --speed-plan steer-brake')
    if args.same_host and (out/'surface_selection.json').is_file():
        install_prepared_surface(out, args.dataset)
    _, route = dataset_inputs(args.dataset, DATA)
    setup = out/'setup.json'
    scene_file = out/'scene.json'
    metadata = dict(controller=args.profile, lap_target=args.laps,
                    execution_host=os.uname().nodename, ros_distro=os.environ.get('ROS_DISTRO'))
    if not args.existing:
        metadata['headless_planner_profile'] = env.get('DAMVI_HEADLESS_GPD_PROFILE', 'tuned')
    if args.same_host:
        server = Path(os.environ['DAMVI_SERVER_ROS_WS'])/'server_ros.sh'
        launch_command, seed_command, stop_command = [server,'stack'], [server,'seed'], [server,'stop']
    else:
        launch_command = [ROOT/'run_headless_stack.sh']
        seed_command = [ROOT/'seed_headless_stack.sh']
        stop_command = [ROOT/'set_headless_mode.sh','stop']
    env['DAMVI_ROUTE_REPORT'] = str(out/'route.json')
    env['DAMVI_GRAVITY_REPORT'] = str(out/'imu_startup.yaml')
    stack=stack_log=None
    try:
        if args.existing:
            query = ['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=5']
            if args.socket:
                query += ['-S', args.socket]
            query += ['shin@192.168.0.2', 'python3 -']
            live = subprocess.run(query, input=LIVE_SCENE_QUERY, text=True, capture_output=True, timeout=12)
            if live.returncode:
                raise RuntimeError(f'Could not confirm current simulator scene: {live.stderr.strip()}')
            scene_file.write_text(live.stdout)
        else:
            reset=[sys.executable,SIM/'tools/reset_dataset.py',args.dataset,
                   '--dataset-root', DATA, '--csv-name', route.name]
            if args.same_host:
                reset.append('--local-isaac')
            if args.socket:
                reset+=['--socket',args.socket]
            invoke(reset,scene_file,env,110)
        scene=json.loads(scene_file.read_text())
        plan=prepare_route(args.dataset, args.cap, DATA, override=env.get('DAMVI_PATH_CSV'),
                           scene=scene, metadata_path=env['DAMVI_ROUTE_REPORT'], speed_plan=env['DAMVI_SPEED_PLAN'])
        metadata.update(plan, cap_mps=args.cap)
        setup.write_text(json.dumps(metadata,indent=2)+'\n')
        env['DAMVI_SCENE_REPORT']=str(scene_file)
        if not args.existing:
            stack_log=(out/'stack.log').open('w')
            stack=subprocess.Popen([*launch_command,args.dataset,str(args.cap),args.profile],
                                   stdout=stack_log,stderr=subprocess.STDOUT,env=env,start_new_session=True)
            invoke([*seed_command,args.dataset],out/'seed.log',env,165)
        observe=[sys.executable,ROOT/'tools/observe_lap.py',route,out/'lap.json','--operate',
                 '--laps',str(args.laps),'--duration',str(args.duration),'--wall-timeout',str(args.duration*2+15)]
        invoke(observe,out/'observation.log',env,args.duration*2+32)
        try:
            snapshot_physics(out,args.socket,scene,args.same_host)
        except (RuntimeError,subprocess.TimeoutExpired,ValueError,KeyError) as error:
            # Preserve independently recorded lap observations if diagnostics fail.
            (out/'physics_trace_error.txt').write_text(str(error)+'\n')
        invoke([sys.executable,ROOT/'tools/score_lap.py',out/'lap.json'],out/'scoring.log',env,15)
        score=json.loads((out/'score.json').read_text())
        print(args.dataset,args.profile,args.cap,score['finish'],round(score.get('net_laps',0),2),
              'laps; GT max',round(score.get('gt_speed_peak_mps',0),2),'m/s', flush=True)
    finally:
        if stack is not None:
            try:
                if stack.poll() is None:
                    if not (out/'score.json').exists():
                        # The observer normally acknowledges Stop before scoring.
                        # Clean up the entire service CLI on a failed startup.
                        stopper = subprocess.Popen(stop_command, env=env, start_new_session=True,
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                        try:
                            stopper.wait(timeout=12)
                        except subprocess.TimeoutExpired:
                            os.killpg(stopper.pid, signal.SIGKILL)
                            stopper.wait()
                    stack.send_signal(signal.SIGINT)
                    try:
                        stack.wait(timeout=16)
                    except subprocess.TimeoutExpired:
                        os.killpg(stack.pid, signal.SIGTERM)
                        stack.wait(timeout=6)
            finally:
                stack_log.close()


if __name__=='__main__':
    main()
