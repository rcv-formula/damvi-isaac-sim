#!/usr/bin/env python3
"""Read-only, timestamped full-stack lap observation on the simulator's ROS domain."""
import argparse
import json
import math
import time
from collections import Counter
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path as RosPath
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message
from std_msgs.msg import String, Float32MultiArray
from std_srvs.srv import SetBool

from command_timing import CommandTiming


def stamp(header):
    return header.stamp.sec + 1e-9 * header.stamp.nanosec


def pose_row(pose):
    p, q = pose.position, pose.orientation
    return [p.x, p.y, p.z, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))]


class Loop:
    def __init__(self, xy):
        xy = np.asarray(xy, dtype=float)
        segments = np.roll(xy, -1, axis=0) - xy
        valid = np.linalg.norm(segments, axis=1) > 1e-7
        self.xy = xy[valid]
        self.ds = np.linalg.norm(segments[valid], axis=1)
        self.unit = segments[valid] / self.ds[:, None]
        self.length = float(self.ds.sum())
        self.starts = np.r_[0., np.cumsum(self.ds[:-1])]
        # Signed curvature averaged over approximately 0.8 m of track.
        a = np.roll(self.unit, 2, axis=0)
        b = np.roll(self.unit, -2, axis=0)
        self.curvature = np.arctan2(a[:, 0]*b[:, 1]-a[:, 1]*b[:, 0], (a*b).sum(axis=1)) / (
            .5*np.roll(self.ds, 2) + np.roll(self.ds, 1) + self.ds + np.roll(self.ds, -1) + .5*np.roll(self.ds, -2))

    def project(self, pos):
        rel = pos-self.xy
        along = np.clip((rel*self.unit).sum(axis=1), 0, self.ds)
        diff = rel-along[:, None]*self.unit
        i = int(np.argmin((diff*diff).sum(axis=1)))
        lateral = self.unit[i, 0]*diff[i, 1] - self.unit[i, 1]*diff[i, 0]
        return float(self.starts[i]+along[i]), float(lateral), float(self.curvature[i])


class Observer(Node):
    def __init__(self, course, laps, duration, output):
        super().__init__('headless_lap_observer')
        self.course, self.laps, self.duration, self.output = course, laps, duration, output
        self.rows, self.pf, self.wheel, self.latency = [], [], [], []
        self.counts = Counter()
        self.gt_t = self.start = self.prev_s = None
        self.progress = 0.
        self.cmd = [0., 0.]
        self.selected = ''
        self.selected_path = None
        self.path_wall = -1.
        self.finished = None
        obs = QoSProfile(depth=30, reliability=ReliabilityPolicy.BEST_EFFORT)
        timing_qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.timing = CommandTiming(self, timing_qos)
        self.create_subscription(PoseStamped, '/validation/ground_truth', self.on_gt, obs)
        try:
            from vesc_msgs.msg import VescStateStamped
        except ImportError:
            self.get_logger().warning('Source the localization overlay to include optional VESC measurements')
        else:
            self.create_subscription(VescStateStamped, '/sensors/core',
                lambda m: self.wheel.append([stamp(m.header), float(m.state.speed)]), obs)
        self.create_subscription(Float32MultiArray, '/localization_pf/latency',
            lambda m: self.latency.append([self.gt_t, *m.data[:2]]) if self.gt_t is not None else None, obs)
        self.create_subscription(Odometry, '/odom', self.on_pf, obs)
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd, obs)
        self.create_subscription(String, '/co_driver_node/status', self.on_mode, obs)
        self.create_subscription(RosPath, '/selected_path', self.on_path, obs, raw=True)

    def on_path(self, buf):
        now = time.monotonic()
        if now-self.path_wall < .18:
            return
        msg = deserialize_message(buf, RosPath)
        if msg.header.frame_id == 'map' and len(msg.poses) >= 3:
            self.selected_path = Loop([[p.pose.position.x,p.pose.position.y] for p in msg.poses])
            self.path_wall = now
            self.counts['selected_path_sampled'] += 1

    def on_cmd(self, m):
        self.cmd = [m.linear.x, m.angular.z]
        self.timing.row('mux', None, *self.cmd)
        if self.start is None and self.gt_t is not None and m.linear.x > .1:
            self.start = self.gt_t

    def on_mode(self, msg):
        self.selected = json.loads(msg.data).get('selected', '')
        self.counts['arbitrator_status'] += 1

    def on_pf(self, m):
        self.timing.row('pf', m.header.stamp)
        if m.header.frame_id.lstrip('/') == 'map':
            self.pf.append([stamp(m.header), *pose_row(m.pose.pose)])

    def on_gt(self, msg):
        self.timing.row('gt', msg.header.stamp)
        if msg.header.frame_id != 'map':
            return
        t = stamp(msg.header)
        if self.gt_t is not None and t <= self.gt_t:
            return
        self.gt_t = t
        if self.start is None:
            return
        x, y, z, yaw = pose_row(msg.pose)
        pos = np.array([x,y])
        s, e, k = self.course.project(pos)
        if self.prev_s is not None:
            ds = (s-self.prev_s+self.course.length/2) % self.course.length - self.course.length/2
            self.progress += ds
        self.prev_s = s
        deformed = self.selected_path.project(pos)[1] if self.selected_path is not None and time.monotonic()-self.path_wall < .5 else None
        self.rows.append([t, x, y, z, yaw, s, self.progress, e, k, deformed, *self.cmd, self.selected])
        elapsed = t-self.start
        if self.progress >= self.laps*self.course.length:
            self.finished = 'lap_target'
        elif elapsed >= self.duration:
            self.finished = 'simulation_timeout'
        elif z < -.08 or z > .23:
            self.finished = 'height_guard'
        elif len(self.rows) > 2 and elapsed >= 5 and abs(e) > 1.5:
            self.finished = 'off_course'
        elif elapsed >= 10:
            trailing = [r for r in self.rows[-600:] if t-r[0] <= 4]
            if len(trailing) >= 5 and t-trailing[0][0] > 3 and max(r[6] for r in trailing)-min(r[6] for r in trailing) < .18:
                self.finished = 'stalled'

    def write(self):
        columns = 't x y z yaw s progress csv_cte curvature selected_cte cmd_v cmd_steer source'.split()
        content = dict(reason=self.finished, length=self.course.length, start=self.start,
                       columns=columns, rows=self.rows, pf_columns=['t','x','y','z','yaw'], pf=self.pf,
                       wheel_columns=['t','erpm'], wheel=self.wheel,
                       latency_columns=['last_gt_t','extrapolation_ms','processing_ms'],
                       latency=self.latency, counts=dict(self.counts),
                       command_timing_columns=['observer_wall_s','header_sim_s','speed_mps','steer'],
                       command_timing=self.timing.traces)
        self.output.write_text(json.dumps(content, separators=(',',':'), allow_nan=False))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('csv', type=Path)
    p.add_argument('output', type=Path)
    p.add_argument('--laps', type=float, default=1.05)
    p.add_argument('--duration', type=float, default=54.)
    p.add_argument('--wall-timeout', type=float, default=130.)
    p.add_argument('--operate', action='store_true', help='Enable the existing autonomous mux after discovery and stop it on exit')
    args=p.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    course=Loop(np.loadtxt(args.csv, delimiter=',')[:,:2])
    node=Observer(course, args.laps, args.duration, args.output)
    deadline=time.monotonic()+args.wall_timeout
    print('Observer subscribed; waiting for GT and autonomous command.', flush=True)
    mux = node.create_client(SetBool, '/validation/enable_autonomy') if args.operate else None
    def mode(enabled):
        req=SetBool.Request(); req.data=enabled
        future=mux.call_async(req)
        rclpy.spin_until_future_complete(node, future, timeout_sec=3.)
        if not future.done() or not future.result().success:
            raise RuntimeError('Headless operator did not acknowledge mode change')
    try:
        if mux is not None:
            if not mux.wait_for_service(timeout_sec=5.):
                raise RuntimeError('Missing headless operator')
            wait=min(deadline, time.monotonic()+30.)
            while (node.gt_t is None or not node.pf or node.selected_path is None) and time.monotonic()<wait:
                rclpy.spin_once(node,timeout_sec=.05)
            if node.gt_t is None or not node.pf or node.selected_path is None:
                raise RuntimeError(f'Inputs missing before starting: GT={node.gt_t is not None}, '
                                   f'PF={bool(node.pf)}, selected path={node.selected_path is not None}')
            mode(True)
        while not node.finished and time.monotonic()<deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        node.finished = node.finished or 'wall_timeout'
    finally:
        try:
            if mux is not None and mux.service_is_ready():
                mode(False)
        finally:
            node.write()
            print(node.finished, len(node.rows), 'GT samples, progress', round(node.progress,2), flush=True)
            node.destroy_node()
            rclpy.shutdown()

if __name__=='__main__':
    main()
