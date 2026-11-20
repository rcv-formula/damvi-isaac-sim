"""Resolve the matching PF map, bag-calibrated grip and speed plan for a headless run."""
import hashlib
from io import StringIO
import json
import math
from pathlib import Path
import sys
import tempfile

import numpy as np
import yaml

from corner_speed_profile import limit_corner_speeds
from steering_speed_profile import configured_steering_profile

ROOT = Path(__file__).resolve().parents[1]
SIM = ROOT.parent/'damvi_vesc_sim'
sys.path.insert(0, str(SIM))
from surface_config import load_surface_configuration


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def unique(paths, label):
    files = list(paths)
    if len(files) != 1:
        raise ValueError(f'Expected one {label}, found {files}')
    return files[0].resolve()


def write_atomic(path, text):
    path.parent.mkdir(exist_ok=True, parents=True)
    with tempfile.NamedTemporaryFile(mode='w', dir=path.parent, delete=False) as temporary:
        temporary.write(text)
    Path(temporary.name).replace(path)


def dataset_inputs(dataset, root):
    if not dataset or dataset in ('.', '..') or Path(dataset).name != dataset:
        raise ValueError('Dataset must name one directory')
    folder = Path(root).expanduser() / dataset
    occupancy_map = unique((p for p in folder.glob('*.yaml') if p.name != 'metadata.yaml'), 'map YAML')
    route = unique(folder.glob('*.csv'), 'raceline CSV')
    return occupancy_map, route


def check_scene(scene, dataset, surface, map_yaml):
    if scene.get('status') != 'ready' or scene.get('dataset') != dataset:
        raise ValueError(f'Isaac scene did not confirm the requested dataset {dataset}')
    observed = scene.get('surface') or {}
    measured_mu = scene.get('native_lateral_mu', [])
    coefficient = surface['coefficient']
    if (observed.get('selected_dataset') != dataset
            or not math.isclose(observed.get('coefficient', -1), coefficient, abs_tol=1e-7)
            or len(measured_mu) != 4
            or not all(math.isclose(mu, coefficient, abs_tol=1e-7) for mu in measured_mu)):
        raise ValueError(f"Isaac's active tire friction disagrees with {dataset} coefficient {coefficient}")
    usd = ROOT.parent/'occupancy_to_usd/generated'/dataset/map_yaml.stem/'track.usdc'
    if scene.get('usd_sha256') != sha(usd):
        raise ValueError(f'Isaac track geometry disagrees with local {usd}')


def prepare_route(dataset, cap, dataset_root, override=None, scene=None, metadata_path=None, speed_plan='auto'):
    if speed_plan not in ('auto', 'steer-brake', 'legacy'):
        raise ValueError(f'Unknown speed plan {speed_plan!r}')
    if override and speed_plan != 'auto':
        raise ValueError('Select either a speed CSV or a generated speed plan')
    cap = None if cap is None or cap == 'uncapped' else float(cap)
    if cap is not None and (not math.isfinite(cap) or cap <= 0):
        raise ValueError('Speed cap must be finite and positive')
    map_yaml, route = dataset_inputs(dataset, dataset_root)
    config = SIM/'config/vesc_sim.yaml'
    surface_path = config.parent / yaml.safe_load(config.read_text())['surface_friction_config']
    surface = load_surface_configuration(surface_path, dataset)
    if 'fallback' in surface['status']:
        raise ValueError(f'{dataset} has no measured or explicitly configured friction coefficient')
    if scene is not None:
        check_scene(scene, dataset, surface, map_yaml)
    original = np.loadtxt(route, delimiter=',', ndmin=2)
    if (original.shape[1] != 3 or len(original) < 3 or not np.all(np.isfinite(original))
            or np.min(original[:, 2]) < 0 or np.max(original[:, 2]) <= 0):
        raise ValueError(f'Raceline requires finite x,y and nonnegative target speeds: {route}')
    selected = route
    mode = 'dataset_csv'
    response = None
    steer_brake = speed_plan == 'steer-brake' or (speed_plan == 'auto' and cap is not None and cap >= 3)
    if override:
        selected = Path(override).expanduser().resolve(strict=True)
        mode = 'explicit_csv'
    elif steer_brake or (cap is not None and cap > 3):
        if steer_brake:
            planned, response = configured_steering_profile(original, surface['coefficient'], config)
            mode = 'steer_brake_envelope'
        else:
            planned = limit_corner_speeds(original[:, :2], original[:, 2], surface['coefficient'])
            mode = 'bag_grip_envelope'
        data = np.column_stack([original[:, :2], planned])
        # Files are content-addressed. A concurrent run cannot overwrite another profile.
        lines = StringIO()
        np.savetxt(lines, data, delimiter=',', fmt='%.17g')
        content = lines.getvalue()
        digest = hashlib.sha256(content.encode()).hexdigest()
        selected = ROOT/'outputs/headless_routes'/dataset/(digest+'.csv')
        if not selected.exists() or sha(selected) != digest:
            write_atomic(selected, content)
    active = original if selected == route else np.loadtxt(selected, delimiter=',', ndmin=2)
    if (active.shape != original.shape or not np.all(np.isfinite(active))
            or not np.allclose(active[:, :2], original[:, :2], atol=1e-8, rtol=0)):
        raise ValueError(f'Speed profile must keep all source CSV x/y positions in order: {selected}')
    if np.any(active[:, 2] < 0) or np.any(active[:, 2] > original[:, 2] + 1e-8) or active[:, 2].max() <= 0:
        raise ValueError(f'Speed profile must stay within source CSV speeds: {selected}')
    profile = dict(dataset=dataset, map_yaml=str(map_yaml), map_sha256=sha(map_yaml),
                   route=str(route), route_sha256=sha(route), speed_profile=str(selected),
                   speed_profile_sha256=sha(selected), profile_source=mode,
                   surface=surface, point_count=len(active),
                   path_max=float(active[:, 2].max()), speed_cap_mps=cap,
                   scene_request_id=scene.get('request_id') if scene else None)
    profile['speed_plan'] = speed_plan
    if response is not None:
        profile['steer_brake_parameters'] = response
    if metadata_path:
        write_atomic(Path(metadata_path), json.dumps(profile, indent=2)+'\n')
    return profile
