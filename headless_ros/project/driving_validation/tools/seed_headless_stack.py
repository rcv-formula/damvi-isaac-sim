#!/usr/bin/env python3
"""Seed PF once from the configured spawn on the CSV and verify the sensor/drive chain."""
import argparse
import csv
import json
import math
import os
from pathlib import Path
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Path as ROSPath
from rosgraph_msgs.msg import Clock
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('dataset')
    parser.add_argument('--timeout', type=float, default=150)
    parser.add_argument('--status', type=Path, default=ROOT/'outputs/support_status.json')
    args = parser.parse_args()
    data_dir = Path(os.environ.get('DAMVI_DATASET_ROOT', str(Path.home()/'Desktop/red-damvi dataset'))) / args.dataset
    with next(data_dir.glob('*.csv')).open() as stream:
        rows = list(csv.reader(stream))
    x, y = map(float, rows[0][:2])
    x2, y2 = map(float, rows[1][:2])
    yaw = math.atan2(y2 - y, x2 - x)
    rclpy.init()
    node = Node('validation_seed')
    seen = {}
    clock_origin = None
    gravity_capture = os.environ.get('DAMVI_GRAVITY_REPORT')
    node.create_subscription(Clock, '/clock', lambda message: seen.update(
        clock=message.clock.sec + 1e-9*message.clock.nanosec), qos_profile_sensor_data)
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(OccupancyGrid, '/map', lambda msg: seen.update(map=bool(msg.data)), qos)
    node.create_subscription(ROSPath, '/global_path', lambda msg: seen.update(
        csv=bool(msg.poses and len(msg.poses) == len(rows) and
                 math.hypot(msg.poses[0].pose.position.x-x, msg.poses[0].pose.position.y-y) < .01)), qos)
    publisher = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 1)
    pose = PoseWithCovarianceStamped()
    pose.header.frame_id = 'map'
    pose.pose.pose.position.x, pose.pose.pose.position.y = x, y
    pose.pose.pose.orientation.z = math.sin(yaw/2)
    pose.pose.pose.orientation.w = math.cos(yaw/2)
    deadline = time.monotonic() + args.timeout
    seed_time = None
    stationary_s = None
    issues = []
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
            if seed_time is None:
                if seen.get('map') and seen.get('csv') and seen.get('clock') and publisher.get_subscription_count():
                    # PF subscribes to the IMU and wheel only after finishing
                    # map construction. Its startup gravity window spans 2 s.
                    if clock_origin is None:
                        clock_origin = seen['clock']
                    if seen['clock'] - clock_origin >= 2.2:
                        stationary_s = seen['clock'] - clock_origin
                        publisher.publish(pose)
                        seed_time = time.time()
                continue
            try:
                status = json.loads(args.status.read_text())
                odom = status.get('odom_latest') or {}
                q = odom.get('orientation_xyzw', [0, 0, 0, 1])
                heading = math.atan2(2*(q[3]*q[2]+q[0]*q[1]), 1-2*(q[1]**2+q[2]**2))
                pos = odom.get('position', [1e9, 1e9])
                issues = list(status.get('guard_reasons', []))
                if math.hypot(pos[0]-x, pos[1]-y) > .4 or abs(math.remainder(heading-yaw, 2*math.pi)) > .25:
                    issues.append('PF spawn mismatch')
                if status.get('wall_time_unix_s', 0) < seed_time + .6:
                    issues.append('waiting for post-seed sensor status')
                if status.get('counts', {}).get('cloud_published', 0) < 1:
                    issues.append('waiting for PF scan projection')
                if not issues:
                    print(json.dumps(dict(ready=True, dataset=args.dataset, x=x, y=y, yaw=yaw,
                        pf_xy=pos[:2], pf_yaw=heading, co_driver=status.get('co_driver', {}).get('selected'),
                        stationary_before_seed_sim_s=round(stationary_s, 2),
                        gravity_calibration_captured=Path(gravity_capture).is_file() if gravity_capture else None)))
                    return
            except (OSError, ValueError, KeyError, TypeError):
                issues = ['waiting for support status']
        raise SystemExit(f'headless stack timeout: map/path={seen}, seeded={seed_time is not None}, waiting={issues}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
