"""Bound the original closed-loop CSV speed by corner grip and available braking distance."""
import argparse
from pathlib import Path

import numpy as np
import yaml


def limit_corner_speeds(xy, speed_mps, coefficient):
    """Return a forward/backward periodic speed envelope in m/s; retain original point order."""
    xy = np.asarray(xy, dtype=float)
    speed = np.asarray(speed_mps, dtype=float)
    if (xy.ndim != 2 or xy.shape[1] != 2 or speed.shape != (len(xy),)
            or len(speed) < 3 or not np.all(np.isfinite(xy)) or not np.all(np.isfinite(speed))
            or np.any(speed < 0) or not np.isfinite(coefficient) or coefficient <= 0):
        raise ValueError('Expected a closed finite XY raceline, nonnegative speeds and positive friction')

    duplicate_end = np.linalg.norm(xy[0]-xy[-1]) < 1e-6
    if duplicate_end:
        xy = xy[:-1]
        speed = speed[:-1].copy()
        speed[0] = min(speed[0], speed_mps[-1])
    incoming = xy - np.roll(xy, 1, axis=0)
    outgoing = np.roll(xy, -1, axis=0) - xy
    ds = np.linalg.norm(outgoing, axis=1)
    if len(xy) < 3 or np.any(ds < 1e-6):
        raise ValueError('Raceline needs three distinct points and nonzero segments')
    chord = np.linalg.norm(incoming + outgoing, axis=1)
    if np.any(chord < 1e-6):
        raise ValueError('Raceline contains a returning segment with undefined turn curvature')
    cross = incoming[:, 0]*outgoing[:, 1] - incoming[:, 1]*outgoing[:, 0]
    curvature = 2*np.abs(cross) / (np.roll(ds, 1)*ds*chord)

    # Reserve friction for tracking error and simultaneously changing speed.
    # 0.83² + 0.45² < 1 for the lateral/braking combination.
    lateral_limit = 0.83*coefficient*9.81
    brake_limit = 0.45*coefficient*9.81
    accel_limit = 0.45*coefficient*9.81
    speed_squared = np.minimum(speed**2, lateral_limit / np.maximum(curvature, 1e-9))

    # Start both passes at the global minimum. Each traverses a full lap so
    # braking and acceleration inequalities also hold across the CSV seam.
    anchor = int(np.argmin(speed_squared))
    for i in range(anchor - 1, anchor - len(ds) - 1, -1):
        j = i % len(ds)
        speed_squared[j] = min(speed_squared[j], speed_squared[(j+1) % len(ds)] + 2*brake_limit*ds[j])
    for i in range(anchor, anchor+len(ds)):
        j = i % len(ds)
        k = (j+1) % len(ds)
        speed_squared[k] = min(speed_squared[k], speed_squared[j] + 2*accel_limit*ds[j])
    result = np.sqrt(speed_squared)
    return np.append(result, result[0]) if duplicate_end else result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv', type=Path)
    parser.add_argument('dataset')
    parser.add_argument('output', type=Path)
    parser.add_argument('--surface', type=Path, default=Path(__file__).resolve().parents[2] /
                        'damvi_vesc_sim/config/surface_friction.yaml')
    args = parser.parse_args()
    entry = yaml.safe_load(args.surface.read_text())['datasets'][args.dataset]
    if entry.get('coefficient') is None:
        parser.error(f'{args.dataset} has no bag-estimated friction coefficient')
    track = np.loadtxt(args.csv, delimiter=',', ndmin=2)
    if track.shape[1] != 3:
        parser.error('Expected a three-column x,y,speed CSV')
    original_max = track[:, 2].max()
    track[:, 2] = limit_corner_speeds(track[:, :2], track[:, 2], entry['coefficient'])
    args.output.parent.mkdir(parents=True, exist_ok=True)
    np.savetxt(args.output, track, delimiter=',', fmt='%.9f')
    print(f"{args.dataset}, mu={entry['coefficient']:.6f}, planned maximum "
          f"{track[:,2].max():.3f} m/s (source {original_max:.3f} m/s)")
