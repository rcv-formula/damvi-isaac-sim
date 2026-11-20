"""Optional raceline speed profile with steer saturation and delayed braking."""
import argparse
import hashlib
import json
import math
from pathlib import Path

import numpy as np
import yaml

from corner_speed_profile import limit_corner_speeds

ROOT = Path(__file__).resolve().parents[1]


def steering_limited_profile(track, mu, wheelbase, max_steer, understeer, lag, crawl, reserve=.06):
    if (not np.all(np.isfinite([mu, wheelbase, max_steer, understeer, lag, crawl, reserve]))
            or min(mu, wheelbase, max_steer, understeer, crawl) <= 0 or lag < 0
            or not 0 <= reserve < 1):
        raise ValueError('Invalid steering geometry, friction or response parameters')
    xy, original = track[:, :2], track[:, 2]
    legacy = limit_corner_speeds(xy, original, mu)
    closed_duplicate = np.linalg.norm(xy[-1]-xy[0]) < 1e-6
    if closed_duplicate:
        xy, legacy = xy[:-1], legacy[:-1]
    previous, following = xy-np.roll(xy, 1, axis=0), np.roll(xy, -1, axis=0)-xy
    ds = np.linalg.norm(following, axis=1)
    turn = np.abs(previous[:, 0]*following[:, 1]-previous[:, 1]*following[:, 0])
    k = 2*turn/(np.roll(ds, 1)*ds*np.linalg.norm(previous+following, axis=1))
    arc = np.r_[0., np.cumsum(ds[:-1])]
    mu_g = mu*9.81
    # Equilibrium with the calibrated tanh lateral tire curve gives
    # delta = atan(L*k) + K*mu*g*atanh(v²*k/(mu*g)). Leave 6% of the
    # servo range for feedback. Infeasible zero-speed geometry is traversed
    # at an explicitly reported crawl; this cannot make that curvature feasible.
    available = (1.-reserve)*max_steer - np.arctan(wheelbase*k)
    used = np.tanh(np.maximum(available, 0.)/(understeer*mu_g))
    speed = np.where(k > 1e-8, np.sqrt(mu_g*used/np.maximum(k, 1e-8)), legacy)
    crawl_points = int(np.sum((speed < crawl) & (legacy > speed)))
    speed = np.minimum(legacy, np.maximum(speed, crawl))
    # Pay the actuator's delay ONCE per future speed restriction. During the
    # delay speed u is held, then b slows the car to w over (u²-w²)/(2*b).
    # Evaluate all anchors on a loop so braking anticipation crosses the seam.
    b = .45*mu_g
    distance = np.remainder(arc[None, :] - arc[:, None], ds.sum())
    approach = np.sqrt((b*lag)**2 + speed[None, :]**2 + 2*b*distance) - b*lag
    speed = np.minimum(speed, np.maximum(speed[None, :], approach).min(axis=1))
    # Mirror the original profile's acceleration budget after each apex.
    anchor = int(np.argmin(speed))
    for i in range(anchor, anchor+len(speed)):
        j = i % len(speed)
        n = (j+1) % len(speed)
        speed[n] = min(speed[n], np.sqrt(speed[j]**2 + 2*b*ds[j]))
    diagnostics = dict(length_m=float(ds.sum()), steering_reserved_fraction=reserve,
                       understeer_rad_per_mps2=understeer, brake_lag_s=lag,
                       physical_delta_limit_rad=max_steer, wheelbase_m=wheelbase,
                       points_above_geometric_limit=int(np.sum(np.arctan(wheelbase*k)>max_steer)),
                       points_using_crawl=crawl_points,
                       crawl_mps=crawl, max_speed_before=float(legacy.max()),
                       max_speed_after=float(speed.max()))
    for name, mask in [('straight_k_lt_0p12', k < .12), ('arc_21_to_23', (arc>=21)&(arc<=23)),
                       ('arc_41_to_43',(arc>=41)&(arc<=43))]:
        if mask.any():
            diagnostics[name] = dict(before_mps=np.round(np.percentile(legacy[mask],[0,50,100]),3).tolist(),
                                     after_mps=np.round(np.percentile(speed[mask],[0,50,100]),3).tolist())
    if closed_duplicate:
        speed = np.append(speed, speed[0])
    return speed, diagnostics


def configured_steering_profile(track, mu, vesc_config, *, lag=None, crawl=None):
    """Read active vehicle geometry and the fixed A/B response calibration."""
    vesc = yaml.safe_load(vesc_config.read_text())
    steering_path = vesc_config.parent/vesc['steering_config']
    plan_path = ROOT/'config/steer_brake_plan.yaml'
    policy = yaml.safe_load(plan_path.read_text())
    pp_path = plan_path.parent/policy['pp_limits_config']
    steer = yaml.safe_load(steering_path.read_text())
    pp = yaml.safe_load(pp_path.read_text())['pure_pursuit']['ros__parameters']
    limit = min(math.radians(pp['steering_limit']), abs(min(steer['wheel_angle_knots_rad'])),
                max(steer['wheel_angle_knots_rad']), steer['center_angle_limit_rad'])
    speed, report = steering_limited_profile(
        track, mu, steer['simulation_wheelbase_m'], limit, policy['understeer_rad_per_mps2'],
        policy['brake_response_delay_s'] if lag is None else lag,
        policy['infeasible_corner_speed_mps'] if crawl is None else crawl,
        reserve=policy['steering_reserve_fraction'])
    report['sources'] = {
        key: {'path': str(path.resolve()), 'sha256': hashlib.sha256(path.read_bytes()).hexdigest()}
        for key, path in [('vehicle',vesc_config), ('steering',steering_path),
                          ('profile',plan_path), ('PP_limits',pp_path)]}
    return speed, report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('dataset')
    parser.add_argument('output', type=Path)
    parser.add_argument('--dataset-root', type=Path, default=Path.home()/'Desktop/red-damvi dataset')
    parser.add_argument('--brake-lag', type=float)
    parser.add_argument('--crawl', type=float)
    args = parser.parse_args()
    config = ROOT.parent/'damvi_vesc_sim/config/vesc_sim.yaml'
    # Share the same dataset resolver and default profiles as launch-time generation.
    vesc = yaml.safe_load(config.read_text())
    surface_path = config.parent / vesc['surface_friction_config']
    surface = yaml.safe_load(surface_path.read_text())['datasets'][args.dataset]
    mu = surface.get('coefficient')
    if not mu or not math.isfinite(mu):
        parser.error('Missing positive bag-calibrated friction coefficient')
    csvs = list((args.dataset_root/args.dataset).glob('*.csv'))
    if len(csvs)!=1:
        parser.error('Expected one source raceline CSV')
    track = np.loadtxt(csvs[0], delimiter=',',ndmin=2)
    if track.shape[1]!=3:
        parser.error('Expected x,y,speed columns')
    speed, report = configured_steering_profile(track, mu, config, lag=args.brake_lag, crawl=args.crawl)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    np.savetxt(args.output, np.column_stack((track[:,:2], speed)), delimiter=',', fmt='%.17g')
    report.update(dataset=args.dataset, surface_mu=mu, source_csv=str(csvs[0]), output_csv=str(args.output))
    print(json.dumps(report,indent=2))


if __name__ == '__main__':
    main()
