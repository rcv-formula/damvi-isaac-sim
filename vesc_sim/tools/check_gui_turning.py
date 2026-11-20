"""Short forward/reverse turn check in the already-running Isaac GUI."""
import argparse
import json
import math
import time
from pathlib import Path
from gui_trace import flush_trace

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan, Imu
from rclpy.qos import QoSProfile, ReliabilityPolicy

ROOT = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser()
parser.add_argument('-o', '--output', type=Path, required=True)
args = parser.parse_args()
rclpy.init()
node = rclpy.create_node('check_current_gui_turning')
clock = [None]
scan_stamps = []
imu_samples = []
node.create_subscription(Clock, '/clock',
    lambda m: clock.__setitem__(0, m.clock.sec + m.clock.nanosec * 1e-9), 10)
node.create_subscription(LaserScan, '/scan',
    lambda m: scan_stamps.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9),
    QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT))
node.create_subscription(Imu, '/imu/data',
    lambda m: imu_samples.append((m.header.stamp.sec + m.header.stamp.nanosec*1e-9, m.angular_velocity.z)),
    QoSProfile(depth=4000, reliability=ReliabilityPolicy.BEST_EFFORT))
publisher = node.create_publisher(Twist, '/cmd_vel', 10)
origin = None
last = None
wall_start = time.monotonic()
try:
    while time.monotonic() - wall_start < 120:
        rclpy.spin_once(node, timeout_sec=.002)
        if clock[0] is None:
            continue
        if origin is None:
            origin = clock[0]
        elapsed = clock[0] - origin
        if clock[0] != last:
            command = Twist()
            command.linear.x = .8 if .5 <= elapsed < 1.5 else -.8 if 2.5 <= elapsed < 3.5 else 0.
            command.angular.z = .3 if elapsed < 3.5 else 0.
            publisher.publish(command)
            last = clock[0]
        if elapsed >= 7.:
            break
finally:
    publisher.publish(Twist())
    node.destroy_node()
    rclpy.shutdown()

flush_trace(ROOT)
trace = json.loads((ROOT / 'outputs/live_gui/trace.json').read_text())
phases = {}
for name, start, end in [('forward', 1., 1.45), ('reverse', 3., 3.45), ('stop', 6.5, 6.95)]:
    rows = [r for r in trace if origin is not None and start < r['time'] - origin < end]
    if not rows:
        continue
    steering_error = [max(abs(a - b) for a, b in zip(
        r['steering_actual_left_right_rad'], r['steering_target_left_right_rad'])) for r in rows]
    phases[name] = {
        'samples': len(rows),
        'median_erpm': float(np.median([r['erpm'] for r in rows])),
        'median_speed_mps': float(np.median([math.hypot(*r['velocity'][:2]) for r in rows])),
        'median_yaw_rate_rad_s': float(np.median([r['angular_velocity_rad_s'][2] for r in rows])),
        'max_steering_error_deg': float(np.rad2deg(max(steering_error))),
    }
    gyro = [z for stamp,z in imu_samples if start < stamp-origin < end]
    phases[name]['median_imu_gyro_z_rad_s'] = float(np.median(gyro)) if gyro else None
status = json.loads((ROOT / 'outputs/live_gui/status.json').read_text())
checks = {
    'clock_completed': origin is not None and clock[0] - origin >= 7.,
    'all_phases': len(phases) == 3,
    'forward_erpm': phases.get('forward', {}).get('median_erpm', 0) > 800,
    'reverse_erpm': phases.get('reverse', {}).get('median_erpm', 0) < -800,
    'forward_left_yaw': phases.get('forward', {}).get('median_yaw_rate_rad_s', 0) > .05,
    'reverse_right_yaw': phases.get('reverse', {}).get('median_yaw_rate_rad_s', 0) < -.05,
    'physical_translation': all(phases.get(n, {}).get('median_speed_mps', 0) > .4 for n in ('forward', 'reverse')),
    'stopped': abs(phases.get('stop', {}).get('median_erpm', math.inf)) < 5,
    'steering_tracks_under_load': len(phases) == 3 and all(p['max_steering_error_deg'] < 1.5 for p in phases.values()),
    'scan_while_driving': len(scan_stamps) > 100,
    'scan_period_40hz': len(scan_stamps) > 2 and abs(float(np.median(np.diff(scan_stamps))) - .025) < 1e-6,
    'no_gui_error': status['error'] is None,
    'imu_matches_turning': len(phases) == 3 and all(
        p['median_imu_gyro_z_rad_s'] is not None and
        abs(p['median_imu_gyro_z_rad_s']-p['median_yaw_rate_rad_s']) < .05 for p in phases.values()),
}
report = {'status': 'pass' if all(checks.values()) else 'fail', 'checks': checks,
          'phases': phases, 'scans_received': len(scan_stamps),
          'stop_measurement_delay_s': [3., 3.45],
          'wall_duration_s': time.monotonic() - wall_start}
args.output.parent.mkdir(parents=True, exist_ok=True)
args.output.write_text(json.dumps(report, indent=2))
print(json.dumps(report, indent=2))
if report['status'] != 'pass':
    raise SystemExit(1)
