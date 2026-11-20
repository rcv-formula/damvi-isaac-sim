"""0821 stack in the existing simulator; the mode UI owns final cmd_vel."""
import json
import math
import os
from pathlib import Path
import sys
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

ROOT=Path(__file__).resolve().parents[1]
CONFIG=ROOT/'config'

def deformer_parameter_files(headless, profile):
    base = CONFIG/'global_path_deformer.yaml'
    if not headless or profile == 'legacy':
        return [str(base)]
    if profile != 'tuned':
        raise ValueError('DAMVI_HEADLESS_GPD_PROFILE must be tuned or legacy')
    return [str(base), str(CONFIG/'headless_global_path_deformer.yaml')]

def generate_launch_description():
    # Read the same source configuration on every launch; no second ERPM gain.
    sim=yaml.safe_load((ROOT.parent/'damvi_vesc_sim/config/vesc_sim.yaml').read_text())
    gain=float(sim['speed_to_erpm_gain'])
    offset=float(sim['speed_to_erpm_offset'])
    if gain<=0:
        raise ValueError('VESC ERPM gain must be positive')
    # Generate only the small machine-specific overlay, leaving upstream tuning.
    tuning=json.loads(''.join(line for line in (CONFIG/'co_driver_isaac_tuning.jsonc').read_text().splitlines(True)
                              if not line.lstrip().startswith('//')))
    for name in ('obstacles','clearance'):
        tuning['inputs'][name]['params'].update(speed_to_erpm_gain=gain,
            speed_to_erpm_offset=offset,wheel_speed_scale=1.0)
    generated=ROOT/'outputs/co_driver_runtime_tuning.json'
    generated.parent.mkdir(parents=True,exist_ok=True)
    generated.write_text(json.dumps(tuning,indent=2)+'\n')
    share=Path(get_package_share_directory('co_driver'))
    tuning_files=','.join(str(p) for p in (share/'config/localization_scoring.jsonc',
        share/'config/obstacle_scoring.jsonc',generated))
    def node(package,exe,name,config,**extra):
        return Node(package=package,executable=exe,name=name,output='screen',
                    parameters=[str(CONFIG/config)],
                    arguments=['--ros-args','--log-level','error' if name=='bt_main' else 'warn'],**extra)
    # The historical GUI launch retains its original settings. Headless runs
    # use the same nodes and the same mux, with a service for mode selection.
    dataset = os.environ.get('DAMVI_HEADLESS_DATASET')
    initial_nodes = []
    path_parameters = {'use_sim_time': True}
    pp_parameters = [str(CONFIG/'ppcontroller.yaml')]
    support_args = []
    co_parameters = {}
    mode_node = node('damvi_teleop_ui','teleop_ui','damvi_teleop_ui','teleop.yaml')
    if dataset:
        sys.path.insert(0, str(ROOT/'tools'))
        from headless_route import prepare_route
        scene_path = os.environ.get('DAMVI_SCENE_REPORT')
        scene = json.loads(Path(scene_path).read_text()) if scene_path else None
        dataset_root = os.environ.get('DAMVI_DATASET_ROOT', str(Path.home()/'Desktop/red-damvi dataset'))
        route = prepare_route(dataset, os.environ['DAMVI_SPEED_CAP'], dataset_root,
                              override=os.environ.get('DAMVI_PATH_CSV'), scene=scene,
                              speed_plan=os.environ.get('DAMVI_SPEED_PLAN', 'auto'),
                              metadata_path=os.environ.get('DAMVI_ROUTE_REPORT',
                                                           ROOT/'outputs/headless_route.json'))
        cap, path_max = route['speed_cap_mps'], route['path_max']
        map_yaml = Path(route['map_yaml'])
        path_parameters['csv_file'] = route['speed_profile']
        # AckermannDrive.speed is float32. Its largest finite value leaves every
        # valid downstream speed unchanged while keeping steering and validity guards.
        downstream_limit = cap if cap is not None else float.fromhex('0x1.fffffep+127')
        support_args = ['-p', f'speed_cap_mps:={downstream_limit}']
        co_parameters['postprocess.clamp.max_speed'] = downstream_limit
        overlay = os.environ.get('DAMVI_PP_OVERLAY', str(CONFIG/'pp_tracking_tuned.yaml'))
        if overlay:
            if not Path(overlay).is_file():
                raise FileNotFoundError(overlay)
            pp_parameters.append(overlay)
        pp_speed = {'max_speed_limit_percentage': min(cap / path_max, 1.0) if cap is not None else 1.0,
                    'speed_hold_max_speed': cap if cap is not None else 20.0}
        if 'DAMVI_PP_VELOCITY_PERCENTAGE' in os.environ:
            multiplier = float(os.environ['DAMVI_PP_VELOCITY_PERCENTAGE'])
            if not math.isfinite(multiplier) or not 0.0 <= multiplier <= 1.0:
                raise ValueError('PP velocity percentage must be between 0 and 1')
            pp_speed['velocity_percentage'] = multiplier
        pp_parameters.append(pp_speed)
        initial_nodes.append(Node(
            package='localization_pf', executable='localization_pf_node', name='localization_pf',
            output='screen', arguments=['--ros-args', '--log-level', 'warn'],
            parameters=[str(ROOT.parent/'localization_validation/config/sim0821.yaml'),
                        {'map_loader.map_dir': str(map_yaml.parent),
                         'map_loader.map_name': map_yaml.stem,
                         'ekf.speed_to_erpm_gain': gain, 'ekf.speed_to_erpm_offset': offset,
                         'ekf.gravity_calibration_path': os.environ.get('DAMVI_GRAVITY_REPORT', '')},
                        str(CONFIG/'headless_pf_precompute.yaml')]))
        mode_node = ExecuteProcess(cmd=[sys.executable, str(ROOT/'tools/headless_operator.py'),
            '--ros-args', '--params-file', str(CONFIG/'teleop.yaml'),
            '-p', 'use_sim_time:=true'], output='screen')
    pp_parameters.append({'speed_to_erpm_gain':gain, 'speed_to_erpm_offset':offset,
                          'wheel_speed_scale':1.0})
    return LaunchDescription(initial_nodes + [
        ExecuteProcess(cmd=[sys.executable,str(ROOT/'tools/sim_support.py'),
            '--ros-args','--params-file',str(CONFIG/'sim_support.yaml'), *support_args],output='screen'),
        # GUI keeps the installed CSV configuration. Headless selects this run's CSV.
        Node(package='path_loader',executable='path_loader',name='path_loader',
             output='screen',parameters=[path_parameters]),
        Node(package='global_path_deformer', executable='global_path_deformer_node',
             name='global_path_deformer', output='screen',
             arguments=['--ros-args', '--log-level', 'warn'],
             parameters=deformer_parameter_files(dataset, os.environ.get('DAMVI_HEADLESS_GPD_PROFILE', 'tuned'))),
        node('behavior_tree_cpp','health_monitor','health_monitor','behavior_tree.yaml'),
        node('behavior_tree_cpp','bt_main','bt_main','behavior_tree.yaml'),
        Node(package='pure_pursuit',executable='pure_pursuit',name='pure_pursuit',output='screen',
             arguments=['--ros-args','--log-level','warn'],
             parameters=pp_parameters),
        node('gap_follow','gap_follow','gap_follow','gap_loc.yaml',namespace='gap_loc'),
        node('gap_follow','gap_follow','gap_follow','gap_obs.yaml',namespace='gap_obs'),
        Node(package='co_driver',executable='co_driver_node',name='co_driver_node',output='screen',
             arguments=['--ros-args','--log-level','warn'],
             parameters=[str(CONFIG/'co_driver_isaac.yaml'),{
                'topics_file':str(share/'config/co_driver_red_damvi_topics.jsonc'),
                'tuning_file':tuning_files,'return_assist_file':'','use_sim_time':True, **co_parameters}]),
        mode_node,
    ])
