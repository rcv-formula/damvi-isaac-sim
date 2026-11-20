"""Observe the current GUI's stationary sensors and retained mounting TF."""
import argparse
import json
import math
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, LaserScan
from tf2_msgs.msg import TFMessage

parser = argparse.ArgumentParser()
parser.add_argument('-o', '--output', type=Path, required=True)
args = parser.parse_args()
rclpy.init()
node = rclpy.create_node('check_live_imu_lidar_tf')
clock = [None]
imu = []
scans = []
transforms = {}
node.create_subscription(Clock, '/clock',
    lambda m: clock.__setitem__(0, m.clock.sec + m.clock.nanosec*1e-9), 10)


def on_imu(message):
    imu.append({'stamp': message.header.stamp.sec + message.header.stamp.nanosec*1e-9,
                'frame': message.header.frame_id,
                'orientation': [getattr(message.orientation, n) for n in ('x', 'y', 'z', 'w')],
                'gyro': [getattr(message.angular_velocity, n) for n in ('x', 'y', 'z')],
                'acceleration': [getattr(message.linear_acceleration, n) for n in ('x', 'y', 'z')]})


def on_scan(message):
    scans.append({'stamp': message.header.stamp.sec + message.header.stamp.nanosec*1e-9,
                  'frame': message.header.frame_id, 'beams': len(message.ranges),
                  'has_nan': any(math.isnan(v) for v in message.ranges)})


def on_tf(message):
    for transform in message.transforms:
        transforms[transform.child_frame_id] = {
            'parent': transform.header.frame_id,
            'translation_m': [getattr(transform.transform.translation, n) for n in ('x', 'y', 'z')],
            'rotation_xyzw': [getattr(transform.transform.rotation, n) for n in ('x', 'y', 'z', 'w')],
        }


qos = QoSProfile(depth=4000, reliability=ReliabilityPolicy.BEST_EFFORT)
node.create_subscription(Imu, '/imu/data', on_imu, qos)
node.create_subscription(LaserScan, '/scan', on_scan, qos)
node.create_subscription(TFMessage, '/tf_static', on_tf,
    QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
               durability=DurabilityPolicy.TRANSIENT_LOCAL))
wall_start = time.monotonic()
origin = None
while time.monotonic() - wall_start < 90:
    rclpy.spin_once(node, timeout_sec=.002)
    if clock[0] is not None:
        if origin is None:
            origin = clock[0]
        if clock[0] - origin >= 3.:
            break
node.destroy_node()
rclpy.shutdown()
settled = [sample for sample in imu if origin is not None and sample['stamp'] > origin + .8]
periods = np.diff([sample['stamp'] for sample in settled])
scan_periods = np.diff([sample['stamp'] for sample in scans])
acceleration = np.mean([s['acceleration'] for s in settled], axis=0) if settled else np.array([math.nan]*3)
median_acceleration = np.median([s['acceleration'] for s in settled], axis=0) if settled else np.array([math.nan]*3)
gyro = np.median([s['gyro'] for s in settled], axis=0) if settled else np.array([math.nan]*3)
checks = {
    'clock_completed': origin is not None and clock[0] - origin >= 3.,
    'imu_received': len(settled) > 1800,
    'imu_frame': bool(imu) and all(s['frame'] == 'imu' for s in imu),
    'imu_finite': bool(imu) and all(np.isfinite(s['orientation'] + s['gyro'] + s['acceleration']).all() for s in imu),
    'imu_unit_orientation': bool(imu) and max(abs(np.linalg.norm(s['orientation']) - 1) for s in imu) < 1e-5,
    'imu_simulated_1khz': len(periods) > 2 and abs(float(np.median(periods)) - .001) < 1e-7,
    'imu_stamps_increase': len(periods) > 2 and float(np.min(periods)) > 0,
    'imu_stationary_gravity': bool(settled) and float(np.linalg.norm(acceleration - [0, 0, 9.81])) < .3,
    'imu_stationary_gyro': bool(settled) and float(np.linalg.norm(gyro)) < .05,
    'scan_geometry': len(scans) > 80 and all(s['beams'] == 1081 and s['frame'] == 'laser' and not s['has_nan'] for s in scans),
    'scan_simulated_40hz': len(scan_periods) > 2 and abs(float(np.median(scan_periods)) - .025) < 1e-6,
    'user_laser_tf': transforms.get('laser') == dict(parent='base_link', translation_m=[.27, 0., .16], rotation_xyzw=[0., 0., 0., 1.]),
    'user_imu_tf': transforms.get('imu') == dict(parent='base_link', translation_m=[.27, 0., .06], rotation_xyzw=[0., 0., 0., 1.]),
}
checks = {key: bool(value) for key, value in checks.items()}
report = {'status': 'pass' if all(checks.values()) else 'fail', 'checks': checks,
          'imu_samples': len(imu), 'imu_settled_samples': len(settled),
          'imu_median_period_s': float(np.median(periods)) if len(periods) else None,
          'imu_mean_acceleration_mps2': acceleration.tolist(),
          'imu_median_acceleration_mps2': median_acceleration.tolist(),
          'imu_median_gyro_rad_s': gyro.tolist(), 'scan_count': len(scans), 'tf': transforms,
          'wall_duration_s': time.monotonic() - wall_start}
args.output.parent.mkdir(parents=True, exist_ok=True)
args.output.write_text(json.dumps(report, indent=2))
print(json.dumps(report, indent=2))
if report['status'] != 'pass':
    raise SystemExit(1)
