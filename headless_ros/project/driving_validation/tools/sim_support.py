#!/usr/bin/env python3
"""Validation adapters only; the existing operator UI alone publishes /cmd_vel.

No ground-truth data is used for control, scan projection, or localization.
Timers and freshness guards use a steady clock even while /clock is paused.
"""

from collections import Counter, deque
from copy import deepcopy
from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import tempfile
import time

import numpy as np
import yaml

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message
from rclpy.time import Time
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path as RosPath
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_msgs.msg import String, UInt16MultiArray
from tf2_ros import Buffer, TransformException, TransformListener


PROJECT_ROOT = Path(__file__).resolve().parents[1]


def stamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def path_has_two_poses(serialized):
    """Read the sequence length of nav_msgs/Path without allocating every pose.

    A Path starts with std_msgs/Header, then a sequence of PoseStamped. The
    standard CDR1 layout aligns the sequence length to a four-byte boundary.
    Use the ROS deserializer for an alternative encapsulation.
    """
    if serialized[:2] not in (b'\x00\x00', b'\x00\x01'):
        return len(deserialize_message(serialized, RosPath).poses) >= 2
    if len(serialized) < 20:
        return False
    order = 'little' if serialized[1] else 'big'
    frame_length = int.from_bytes(serialized[12:16], order)  # includes string terminator
    count_offset = 16 + ((frame_length + 3) // 4) * 4
    if not frame_length or count_offset + 4 > len(serialized):
        return False
    return int.from_bytes(serialized[count_offset:count_offset+4], order) >= 2


def finite_pose(pose):
    values = [pose.position.x, pose.position.y, pose.position.z,
              pose.orientation.x, pose.orientation.y, pose.orientation.z,
              pose.orientation.w]
    return all(math.isfinite(v) for v in values) and sum(v*v for v in values[3:]) > 1e-12


def pose_snapshot(header, pose):
    return {
        'stamp_ns': stamp_ns(header.stamp), 'frame_id': header.frame_id,
        'position': [pose.position.x, pose.position.y, pose.position.z],
        'orientation_xyzw': [pose.orientation.x, pose.orientation.y,
                             pose.orientation.z, pose.orientation.w],
    }


@dataclass(frozen=True)
class SteeringMapping:
    gain: float
    bias: float
    input_limit: float
    source: str
    input_knots: tuple = ()
    angle_knots: tuple = ()
    slopes: tuple = ()
    angle_limit: float = math.inf

    @classmethod
    def load(cls, config_path, gain, bias, input_limit):
        """Invert the simulator's static PCHIP, without cancelling its dynamics.

        ``gain`` and ``bias`` remain the endpoint secant for status/fallback
        compatibility. Multi-knot maps use the full curve for the conversion.
        Derivatives match native/steering.c, including endpoint limiting.
        """
        knots = angles = slopes = ()
        angle_limit = math.inf
        if config_path:
            with open(config_path, encoding='utf-8') as stream:
                config = yaml.safe_load(stream)
            if not isinstance(config, dict):
                raise ValueError('simulator steering configuration must be a mapping')
            if config.get('model', 'monotone_pchip') != 'monotone_pchip':
                raise ValueError('unsupported simulator steering model')
            if not config.get('enabled', True):
                raise ValueError('simulator steering must be enabled for autonomous conversion')
            x = np.asarray(config['input_knots_rad'], dtype=float)
            y = np.asarray(config['wheel_angle_knots_rad'], dtype=float)
            if x.ndim != 1 or y.shape != x.shape or not 2 <= x.size <= 64:
                raise ValueError('simulator steering requires 2 to 64 matching knots')
            if (not np.all(np.isfinite(x)) or not np.all(np.isfinite(y)) or
                    np.any(np.diff(x) <= 0) or np.any(np.diff(y) < 0)):
                raise ValueError('invalid simulator steering knots')
            angle_limit = float(config.get('center_angle_limit_rad', math.inf))
            if 'center_angle_limit_rad' in config and (
                    not math.isfinite(angle_limit) or not 0 < angle_limit < 1.4 or
                    np.any(np.abs(y) > angle_limit + 1e-7)):
                raise ValueError('invalid simulator steering angle limit')
            gain = float((y[-1] - y[0]) / (x[-1] - x[0]))
            bias = float(y[0] - gain * x[0])
            input_limit = float(config['input_limit_rad'])
            h = np.diff(x)
            d = np.diff(y) / h
            if x.size == 2:
                derivative = [float(d[0]), float(d[0])]
            else:
                def endpoint(h0, h1, d0, d1):
                    result = ((2*h0+h1)*d0-h0*d1)/(h0+h1)
                    if result*d0 <= 0:
                        return 0.0
                    if d0*d1 <= 0 and abs(result) > 3*abs(d0):
                        return float(3*d0)
                    return float(result)

                derivative = [endpoint(h[0], h[1], d[0], d[1])]
                for index in range(1, x.size-1):
                    d0, d1 = d[index-1], d[index]
                    if d0*d1 <= 0:
                        derivative.append(0.0)
                    else:
                        w0, w1 = 2*h[index]+h[index-1], h[index]+2*h[index-1]
                        derivative.append(float((w0+w1)/(w0/d0+w1/d1)))
                derivative.append(endpoint(h[-1], h[-2], d[-1], d[-2]))
            if not all(math.isfinite(v) for v in derivative):
                raise ValueError('non-finite simulator steering derivatives')
            knots, angles, slopes = tuple(x), tuple(y), tuple(derivative)
        if not all(math.isfinite(v) for v in (gain, bias, input_limit)):
            raise ValueError('steering calibration must be finite')
        if (not knots and abs(gain) < 1e-9) or input_limit <= 0:
            raise ValueError('steering gain must be nonzero and input limit positive')
        return cls(float(gain), float(bias), float(input_limit), str(config_path),
                   knots, angles, slopes, angle_limit)

    def _forward(self, nominal_angle):
        """Static center-angle target; actuator lag/rate limits stay in Isaac."""
        x, y = self.input_knots, self.angle_knots
        query = float(np.clip(nominal_angle, -self.input_limit, self.input_limit))
        if not x:
            return self.bias + self.gain*query
        if query <= x[0]:
            value = y[0]
        elif query >= x[-1]:
            value = y[-1]
        else:
            j = int(np.searchsorted(x, query, side='left'))-1
            h = x[j+1]-x[j]
            t = (query-x[j])/h
            t2, t3 = t*t, t*t*t
            value = ((2*t3-3*t2+1)*y[j] + (t3-2*t2+t)*h*self.slopes[j] +
                     (-2*t3+3*t2)*y[j+1] + (t3-t2)*h*self.slopes[j+1])
        return float(np.clip(value, -self.angle_limit, self.angle_limit))

    def inverse(self, physical_angle):
        if not math.isfinite(physical_angle):
            raise ValueError('non-finite physical steering angle')
        if not self.input_knots or (
                len(self.input_knots) == 2 and self.gain > 0 and
                self.input_knots[0] <= -self.input_limit and
                self.input_knots[-1] >= self.input_limit and
                max(abs(v) for v in self.angle_knots) <= self.angle_limit):
            return float(np.clip((physical_angle-self.bias)/self.gain,
                                 -self.input_limit, self.input_limit))

        low, high = -self.input_limit, self.input_limit
        target = float(np.clip(physical_angle, self._forward(low), self._forward(high)))
        center = self._forward(0.0)
        if target == center:
            return 0.0
        intervals = []
        x, y = self.input_knots, self.angle_knots
        if target == y[0]:
            intervals.append((low, min(high, x[0])))
        if target == y[-1]:
            intervals.append((max(low, x[-1]), high))
        for j in range(len(x)-1):
            if y[j] == target and y[j+1] == target:
                intervals.append((max(low, x[j]), min(high, x[j+1])))
        flat_solutions = [float(np.clip(0.0, a, b)) for a, b in intervals if a <= b]
        if flat_solutions:
            return min(flat_solutions, key=lambda value: (abs(value), value))
        # Seek the edge of a flat interval nearest zero. Strict comparison on
        # the relevant side gives a deterministic inverse for deadbands and
        # endpoint saturation, including when the input limit clips the knots.
        if target > center:
            low = 0.0
            for _ in range(64):
                midpoint = (low+high)/2
                if self._forward(midpoint) < target:
                    low = midpoint
                else:
                    high = midpoint
            return float(high)
        high = 0.0
        for _ in range(64):
            midpoint = (low+high)/2
            if self._forward(midpoint) > target:
                high = midpoint
            else:
                low = midpoint
        return float(low)


def bounded_command(speed, physical_angle, mapping, speed_cap):
    if not math.isfinite(speed) or not math.isfinite(physical_angle):
        raise ValueError('non-finite drive command')
    return float(np.clip(speed, -speed_cap, speed_cap)), mapping.inverse(physical_angle)


def freshness_reasons(now, receipts, limits, valid, configuration_error=''):
    reasons = []
    if configuration_error:
        reasons.append('configuration_error')
    for name, limit in limits.items():
        received = receipts.get(name)
        if received is None:
            reasons.append(name + '_missing')
        elif now - received > limit:
            reasons.append(name + '_stale')
        elif not valid.get(name, False):
            reasons.append(name + '_invalid')
    return reasons


def transform_components(message):
    transform = message.transform
    position = np.array([transform.translation.x, transform.translation.y,
                         transform.translation.z], dtype=float)
    quaternion = np.array([transform.rotation.x, transform.rotation.y,
                           transform.rotation.z, transform.rotation.w], dtype=float)
    norm = float(np.linalg.norm(quaternion))
    if not np.all(np.isfinite(position)) or not np.all(np.isfinite(quaternion)) or norm < 1e-12:
        raise ValueError('non-finite or zero-norm scan transform')
    return position, quaternion / norm


def project_scan(scan, first_transform, last_transform):
    """Project valid beams using PF transforms at their acquisition endpoints."""
    ranges = np.asarray(scan.ranges, dtype=float)
    indices = np.flatnonzero(np.isfinite(ranges) & (ranges >= scan.range_min) &
                            (ranges <= scan.range_max))
    angles = float(scan.angle_min) + indices * float(scan.angle_increment)
    points = np.column_stack((ranges[indices] * np.cos(angles),
                              ranges[indices] * np.sin(angles),
                              np.zeros(indices.size)))
    p0, q0 = transform_components(first_transform)
    p1, q1 = transform_components(last_transform)
    if np.dot(q0, q1) < 0:
        q1 = -q1
    fractions = indices.astype(float) / max(1, ranges.size - 1)
    quaternions = q0[None, :] + fractions[:, None] * (q1 - q0)[None, :]
    if indices.size:
        quaternions /= np.linalg.norm(quaternions, axis=1)[:, None]
    xyz = quaternions[:, :3]
    cross = 2.0 * np.cross(xyz, points)
    rotated = points + quaternions[:, 3, None] * cross + np.cross(xyz, cross)
    positions = p0[None, :] + fractions[:, None] * (p1 - p0)[None, :]
    return np.asarray(rotated + positions, dtype='<f4')


def cloud_message(scan, points, frame):
    result = PointCloud2()
    result.header = deepcopy(scan.header)
    result.header.frame_id = frame
    result.height = 1
    result.width = int(points.shape[0])
    result.fields = [PointField(name=name, offset=offset,
                                datatype=PointField.FLOAT32, count=1)
                     for name, offset in (('x', 0), ('y', 4), ('z', 8))]
    result.is_bigendian = False
    result.point_step = 12
    result.row_step = result.width * result.point_step
    result.is_dense = True
    result.data = points.tobytes(order='C')
    return result


class SimSupport(Node):
    def __init__(self):
        super().__init__('damvi_sim_support')
        defaults = {
            'rf_topic': '/rf', 'rf_hz': 20.0, 'rf_ch9_value': 2000, 'rf_ch10_value': 2000,
            'scan_topic': '/scan', 'scan_gap_topic': '/scan_gap',
            'scan_cloud_topic': '/scan_matched_points2', 'map_frame': 'map',
            'deskew_scan': True, 'scan_queue_size': 8, 'scan_queue_timeout_s': 0.2,
            'gt_topic': '/validation/ground_truth', 'gt_odom_topic': '/odom_gt',
            'gt_child_frame': 'base_link_gt', 'odom_topic': '/odom',
            'path_topic': '/selected_path', 'drive_topic': '/drive_autonomy',
            'autonomy_twist_topic': '/autonomy/cmd_vel',
            'co_driver_status_topic': '/co_driver_node/status',
            'command_hz': 50.0, 'speed_cap_mps': 1.2,
            'drive_timeout_s': 0.3, 'odom_timeout_s': 0.3,
            'scan_timeout_s': 0.3, 'path_timeout_s': 0.5,
            'steering_config': str(PROJECT_ROOT.parent / 'damvi_vesc_sim/config/steering.yaml'),
            'steering_gain': 0.4184748463865287,
            'steering_bias': 0.014131439965515014,
            'steering_input_limit_rad': 0.673076923076923,
            'status_path': str(PROJECT_ROOT / 'outputs/support_status.json'),
        }
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.settings = {key: self.get_parameter(key).value for key in defaults}
        cfg = self.settings
        self.counts = Counter()
        self.tf_errors = Counter()
        self.receipts = {}
        self.valid = {}
        self.configuration_error = ''
        self.pending_scans = deque()
        self.latest_drive = None
        self.latest_odom = None
        self.latest_gt = None
        self.latest_co_driver = None
        self.latest_co_driver_received = None
        self.last_tf_error = ''
        self.last_scan_stamp = None
        self.last_cloud_stamp = None
        self.last_command = {'speed_mps': 0.0, 'nominal_steering_rad': 0.0}
        self.guard_reasons = ['drive_missing', 'odom_missing', 'scan_missing', 'path_missing']
        self.running = True
        self.limits = {name: float(cfg[name + '_timeout_s'])
                       for name in ('drive', 'odom', 'scan', 'path')}
        try:
            if any(not math.isfinite(v) or v <= 0 for v in self.limits.values()):
                raise ValueError('freshness limits must be positive and finite')
            if not math.isfinite(cfg['speed_cap_mps']) or cfg['speed_cap_mps'] <= 0:
                raise ValueError('speed cap must be positive and finite')
            if not 1 <= int(cfg['scan_queue_size']) <= 8:
                raise ValueError('scan queue size must be in [1, 8]')
            if not 0 < cfg['scan_queue_timeout_s'] <= 0.2:
                raise ValueError('scan queue timeout must be in (0, 0.2] seconds')
            if cfg['autonomy_twist_topic'] == '/cmd_vel':
                raise ValueError('support output must go through the existing operator UI')
            self.mapping = SteeringMapping.load(
                cfg['steering_config'], cfg['steering_gain'], cfg['steering_bias'],
                cfg['steering_input_limit_rad'])
        except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError) as error:
            self.configuration_error = str(error)
            self.get_logger().error('Autonomy held at zero: ' + self.configuration_error)
            self.mapping = SteeringMapping(cfg['steering_gain'], cfg['steering_bias'],
                                           cfg['steering_input_limit_rad'], 'invalid_config')

        reliable = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=2,
                              reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.VOLATILE)
        sensor = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=8,
                            reliability=ReliabilityPolicy.BEST_EFFORT,
                            durability=DurabilityPolicy.VOLATILE)
        self.rf_values = [1500] * 14
        self.rf_values[8] = int(cfg['rf_ch9_value'])
        self.rf_values[9] = int(cfg['rf_ch10_value'])
        self.rf_pub = self.create_publisher(UInt16MultiArray, cfg['rf_topic'], reliable)
        self.scan_pub = self.create_publisher(LaserScan, cfg['scan_gap_topic'], reliable)
        self.cloud_pub = self.create_publisher(PointCloud2, cfg['scan_cloud_topic'], reliable)
        self.gt_pub = self.create_publisher(Odometry, cfg['gt_odom_topic'], reliable)
        self.twist_pub = self.create_publisher(Twist, cfg['autonomy_twist_topic'], reliable)
        self.create_subscription(LaserScan, cfg['scan_topic'], self.on_scan, sensor)
        self.create_subscription(PoseStamped, cfg['gt_topic'], self.on_gt, sensor)
        self.create_subscription(Odometry, cfg['odom_topic'], self.on_odom, sensor)
        self.create_subscription(RosPath, cfg['path_topic'], self.on_path, reliable, raw=True)
        self.create_subscription(AckermannDriveStamped, cfg['drive_topic'], self.on_drive, reliable)
        self.create_subscription(String, cfg['co_driver_status_topic'], self.on_co_driver, reliable)

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0), node=self)
        # PF publishes both dynamic links per 100 Hz tick. A default depth of
        # 100 can leave a loaded executor ~0.5 s behind a current LaserScan.
        # Keep several complete TF pairs while favouring recent observations.
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False, qos=sensor)
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(1.0 / float(cfg['rf_hz']), self.publish_rf, clock=self.steady_clock)
        self.create_timer(1.0 / float(cfg['command_hz']), self.publish_command,
                          clock=self.steady_clock)
        self.create_timer(0.02, self.process_scans, clock=self.steady_clock)
        self.create_timer(1.0, self.write_status, clock=self.steady_clock)
        self.get_logger().info(
            f'RF CH9={self.rf_values[8]} CH10={self.rf_values[9]}; '
            f'physical steering inverse gain={self.mapping.gain:.12g}, '
            f'bias={self.mapping.bias:.12g}; output {cfg["autonomy_twist_topic"]}, '
            f'speed cap {cfg["speed_cap_mps"]:.3g} m/s; ground truth is evaluation only')

    def receive(self, name, valid=True):
        self.counts[name + '_received'] += 1
        self.receipts[name] = time.monotonic()
        self.valid[name] = bool(valid)

    def on_drive(self, message):
        valid = math.isfinite(message.drive.speed) and math.isfinite(message.drive.steering_angle)
        self.receive('drive', valid)
        self.latest_drive = message if valid else None

    def on_odom(self, message):
        valid = finite_pose(message.pose.pose)
        self.receive('odom', valid)
        if valid:
            self.latest_odom = pose_snapshot(message.header, message.pose.pose)
            self.latest_odom['child_frame_id'] = message.child_frame_id

    def on_path(self, message):
        self.receive('path', path_has_two_poses(message))

    def on_gt(self, message):
        self.counts['gt_received'] += 1
        if not finite_pose(message.pose):
            self.counts['gt_invalid'] += 1
            return
        if message.header.frame_id != self.settings['map_frame']:
            self.counts['gt_wrong_frame'] += 1
            return
        odom = Odometry()
        odom.header = deepcopy(message.header)
        odom.child_frame_id = self.settings['gt_child_frame']
        odom.pose.pose = deepcopy(message.pose)
        self.gt_pub.publish(odom)
        self.counts['gt_odom_published'] += 1
        self.latest_gt = pose_snapshot(message.header, message.pose)
        self.latest_gt['received_monotonic_s'] = time.monotonic()

    def on_scan(self, message):
        valid = (bool(message.header.frame_id) and len(message.ranges) > 0 and
                 math.isfinite(message.angle_min) and math.isfinite(message.angle_increment) and
                 math.isfinite(message.time_increment) and message.time_increment >= 0 and
                 math.isfinite(message.range_min) and math.isfinite(message.range_max) and
                 message.range_max > message.range_min >= 0)
        self.receive('scan', valid)
        # Relay the original message, including the first-beam stamp and timing.
        self.scan_pub.publish(message)
        self.counts['scan_relayed'] += 1
        if not valid:
            self.counts['cloud_invalid_scan'] += 1
            return
        current_stamp = stamp_ns(message.header.stamp)
        if self.last_scan_stamp is not None and current_stamp < self.last_scan_stamp:
            self.pending_scans.clear()
            self.counts['scan_clock_resets'] += 1
        self.last_scan_stamp = current_stamp
        if len(self.pending_scans) >= int(self.settings['scan_queue_size']):
            self.pending_scans.popleft()
            self.counts['cloud_queue_overflow'] += 1
        self.pending_scans.append((time.monotonic(), message))

    def on_co_driver(self, message):
        self.counts['co_driver_status_received'] += 1
        try:
            self.latest_co_driver = json.loads(message.data)
            self.latest_co_driver_received = time.monotonic()
        except (ValueError, TypeError):
            self.counts['co_driver_status_invalid'] += 1

    def publish_rf(self):
        self.rf_pub.publish(UInt16MultiArray(data=self.rf_values))
        self.counts['rf_published'] += 1

    def publish_command(self):
        now = time.monotonic()
        self.guard_reasons = freshness_reasons(
            now, self.receipts, self.limits, self.valid, self.configuration_error)
        command = Twist()
        if not self.guard_reasons and self.latest_drive is not None:
            command.linear.x, command.angular.z = bounded_command(
                self.latest_drive.drive.speed, self.latest_drive.drive.steering_angle,
                self.mapping, float(self.settings['speed_cap_mps']))
        else:
            self.counts['guard_zero_published'] += 1
        self.twist_pub.publish(command)
        self.counts['autonomy_twist_published'] += 1
        self.last_command = {'speed_mps': command.linear.x,
                             'nominal_steering_rad': command.angular.z}
        if self.latest_drive is not None:
            self.last_command.update({
                'requested_speed_mps': self.latest_drive.drive.speed,
                'requested_physical_steering_rad': self.latest_drive.drive.steering_angle})

    def process_scans(self):
        # Bounded work and waiting: at most eight scans, none older than 0.2 s.
        now = time.monotonic()
        for _ in range(len(self.pending_scans)):
            received, scan = self.pending_scans.popleft()
            if now - received > float(self.settings['scan_queue_timeout_s']):
                self.counts['cloud_tf_timeout_dropped'] += 1
                continue
            try:
                first_ns = stamp_ns(scan.header.stamp)
                last_ns = first_ns + round(
                    max(0, len(scan.ranges) - 1) * float(scan.time_increment) * 1e9)
                first = self.tf_buffer.lookup_transform(
                    self.settings['map_frame'], scan.header.frame_id,
                    Time(nanoseconds=first_ns, clock_type=ClockType.ROS_TIME))
                if self.settings['deskew_scan'] and last_ns != first_ns:
                    last = self.tf_buffer.lookup_transform(
                        self.settings['map_frame'], scan.header.frame_id,
                        Time(nanoseconds=last_ns, clock_type=ClockType.ROS_TIME))
                else:
                    last = first
                points = project_scan(scan, first, last)
                self.cloud_pub.publish(cloud_message(scan, points, self.settings['map_frame']))
                self.counts['cloud_published'] += 1
                self.counts['cloud_points_published'] += int(points.shape[0])
                self.last_cloud_stamp = first_ns
            except TransformException as error:
                self.tf_errors[type(error).__name__] += 1
                self.last_tf_error = str(error)
                self.pending_scans.append((received, scan))
            except (ValueError, TypeError, OverflowError) as error:
                self.counts['cloud_invalid_geometry_dropped'] += 1
                self.last_tf_error = str(error)

    def write_status(self):
        now = time.monotonic()
        reasons = freshness_reasons(now, self.receipts, self.limits, self.valid,
                                    self.configuration_error)
        status = {
            'running': self.running, 'wall_time_unix_s': time.time(),
            'ros_time_ns': self.get_clock().now().nanoseconds,
            'counts': dict(self.counts), 'rf_values': self.rf_values,
            'guard_reason': ','.join(reasons) if reasons else 'ready',
            'guard_reasons': reasons,
            'ages_s': {name: (now - self.receipts[name]) if name in self.receipts else None
                       for name in self.limits},
            'configuration_error': self.configuration_error,
            'steering': {'gain': self.mapping.gain, 'bias': self.mapping.bias,
                         'input_limit_rad': self.mapping.input_limit,
                         'source': self.mapping.source},
            'speed_cap_mps': self.settings['speed_cap_mps'], 'command': self.last_command,
            'co_driver': self.latest_co_driver,
            'co_driver_age_s': (now - self.latest_co_driver_received
                                if self.latest_co_driver_received is not None else None),
            'gt_latest': self.latest_gt, 'odom_latest': self.latest_odom,
            'cloud': {'queue_size': len(self.pending_scans),
                      'last_scan_stamp_ns': self.last_scan_stamp,
                      'last_cloud_stamp_ns': self.last_cloud_stamp,
                      'tf_errors': dict(self.tf_errors), 'last_tf_error': self.last_tf_error,
                      'pose_source': 'localization TF only',
                      'deskew': self.settings['deskew_scan']},
        }
        destination = Path(self.settings['status_path']).expanduser()
        temporary = None
        try:
            destination.parent.mkdir(parents=True, exist_ok=True)
            with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8',
                                             dir=destination.parent, delete=False) as stream:
                temporary = stream.name
                json.dump(status, stream, allow_nan=False, separators=(',', ':'))
                stream.write('\n')
            os.replace(temporary, destination)
        except (OSError, ValueError, TypeError) as error:
            self.counts['status_write_errors'] += 1
            self.get_logger().warning('Could not write support status: ' + str(error))
            if temporary is not None:
                try:
                    os.unlink(temporary)
                except OSError:
                    pass

    def stop(self):
        self.running = False
        # This still goes through the operator UI; never publish /cmd_vel here.
        self.twist_pub.publish(Twist())
        self.last_command = {'speed_mps': 0.0, 'nominal_steering_rad': 0.0}
        self.write_status()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SimSupport()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        # Humble can invalidate a pending subscription while SIGINT shuts rclpy down.
        if rclpy.ok():
            raise
    finally:
        if node is not None:
            try:
                if node.context.ok():
                    node.stop()
            except RuntimeError:
                # SIGINT may invalidate the publisher after the context check.
                if node.context.ok():
                    raise
            finally:
                node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
