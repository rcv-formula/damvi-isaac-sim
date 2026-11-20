"""Rolling UST-10LX LaserScan from the live Isaac PhysX collision scene.

No extra simulator, window, graph, or USD save. Chassis poses come directly
from physics tensors, so Fabric/USD transform synchronization is unnecessary.
ROS/Isaac imports are lazy; geometry and the scheduler are testable off-line.
"""
from pathlib import Path
import math
import time

import numpy as np
import yaml


def quaternion_matrix(xyzw):
    q = np.asarray(xyzw, dtype=float)
    norm = np.linalg.norm(q)
    if q.shape != (4,) or not math.isfinite(norm) or norm < 1e-12:
        raise ValueError('Expected a finite nonzero XYZW quaternion')
    x, y, z, w = q / norm
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])


def rpy_quaternion(rpy):
    r, p, y = np.radians(rpy) / 2
    cr, cp, cy = np.cos([r, p, y]); sr, sp, sy = np.sin([r, p, y])
    return np.array([sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy,
                     cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy])


def interpolate_pose(a, b, fraction):
    """Translation lerp and shortest-path normalized quaternion lerp per tick."""
    out = np.empty(7)
    out[:3] = a[:3] + fraction*(b[:3]-a[:3])
    qa, qb = a[3:], b[3:]
    if np.dot(qa, qb) < 0:
        qb = -qb
    out[3:] = qa + fraction*(qb-qa)
    out[3:] /= np.linalg.norm(out[3:])
    return out


def beam_geometry(previous_pose, current_pose, fractions, offset, directions):
    """Batch the per-tick rolling poses and ray transforms, with XYZW quats.

    This is the same shortest-path nlerp used by ``interpolate_pose``. Only
    geometry is batched: callers still issue each PhysX scene query in order.
    """
    fractions = np.asarray(fractions, dtype=float)[:, None]
    positions = previous_pose[:3]+fractions*(current_pose[:3]-previous_pose[:3])
    qa, qb = previous_pose[3:], current_pose[3:]
    if np.dot(qa, qb) < 0:
        qb = -qb
    quaternions = qa+fractions*(qb-qa)
    quaternions /= np.linalg.norm(quaternions, axis=1)[:, None]
    x, y, z, w = quaternions.T
    rotations = np.empty((len(fractions), 3, 3))
    rotations[:, 0, 0] = 1-2*(y*y+z*z)
    rotations[:, 0, 1] = 2*(x*y-z*w)
    rotations[:, 0, 2] = 2*(x*z+y*w)
    rotations[:, 1, 0] = 2*(x*y+z*w)
    rotations[:, 1, 1] = 1-2*(x*x+z*z)
    rotations[:, 1, 2] = 2*(y*z-x*w)
    rotations[:, 2, 0] = 2*(x*z-y*w)
    rotations[:, 2, 1] = 2*(y*z+x*w)
    rotations[:, 2, 2] = 1-2*(x*x+y*y)
    return (positions+np.einsum('nij,j->ni', rotations, offset),
            np.einsum('nij,nj->ni', rotations, directions))


def stamp_time(stamp, seconds):
    total_ns = round(seconds * 1_000_000_000)
    stamp.sec, stamp.nanosec = divmod(total_ns, 1_000_000_000)


def load_lidar_config(path):
    config = yaml.safe_load(Path(path).read_text())
    for key in ('scan_rate_hz', 'angle_increment_deg', 'range_min_m', 'range_max_m',
                'rotation_degrees_per_scan'):
        if not math.isfinite(float(config[key])) or config[key] <= 0:
            raise ValueError(f'Invalid lidar {key}')
    lo, hi, increment = (float(config[n]) for n in
                        ('angle_min_deg', 'angle_max_deg', 'angle_increment_deg'))
    if not all(map(math.isfinite, (lo, hi))) or not 0 < hi-lo < config['rotation_degrees_per_scan']:
        raise ValueError('Lidar visible arc must be positive and smaller than one rotation')
    intervals = (hi-lo)/increment
    if abs(intervals-round(intervals)) > 1e-6:
        raise ValueError('Lidar angle span must contain an integer number of intervals')
    if config['range_max_m'] <= config['range_min_m']:
        raise ValueError('Invalid lidar range limits')
    if not config['topic'] or not config['frame_id']:
        raise ValueError('Lidar topic/frame names cannot be empty')
    if int(config['qos_depth']) < 1:
        raise ValueError('Invalid lidar QoS depth')
    config.setdefault('backend', 'physx')
    if config['backend'] not in ('physx', 'physics_core'):
        raise ValueError('Lidar backend must be physx or physics_core')
    config['sample_count'] = round(intervals)+1
    config['scan_time_s'] = 1/config['scan_rate_hz']
    config['time_increment_s'] = config['scan_time_s']*increment/config['rotation_degrees_per_scan']
    return config


class CollisionRaycaster:
    """Nearest non-vehicle hit; all-hit PhysX callbacks are not distance sorted."""
    backend = 'physx'
    def __init__(self, query, vector_type, meters_per_unit, ignored_root=None, both_sides=True):
        self.query = query
        self.vector_type = vector_type
        self.meters_per_unit = float(meters_per_unit)
        self.ignored_root = ignored_root.rstrip('/') if ignored_root else None
        self.both_sides = bool(both_sides)

    def ignored(self, path):
        path = str(path)
        return self.ignored_root and (path == self.ignored_root or path.startswith(self.ignored_root+'/'))

    def range(self, origin, direction, range_min, range_max):
        origin = self.vector_type(*map(float, origin))
        direction = self.vector_type(*map(float, direction))
        limit = range_max/self.meters_per_unit
        hit = self.query.raycast_closest(origin, direction, limit, self.both_sides)
        distance = math.inf
        if hit.get('hit', False):
            if not (self.ignored(hit.get('collision', '')) or self.ignored(hit.get('rigidBody', ''))):
                distance = float(hit['distance'])*self.meters_per_unit
            else:
                # Looking past a self hit requires ALL hits, not advancing the
                # origin (which can skip thin walls or re-enter the vehicle).
                def accept(candidate):
                    nonlocal distance
                    if not (self.ignored(candidate.collision) or self.ignored(candidate.rigid_body)):
                        distance = min(distance, float(candidate.distance)*self.meters_per_unit)
                    return True
                self.query.raycast_all(origin, direction, limit, accept, self.both_sides)
        if not math.isfinite(distance) or distance > range_max:
            return math.inf
        return -math.inf if distance < range_min else distance


class PhysicsCoreRaycaster:
    """Physics umbrella queries with integer path IDs, avoiding hit dictionaries.

    The installed API returns ``(bool, RaycastHit)``; collision and rigid_body
    are uint64 IDs matching PhysicsSchemaTools.sdfPathToInt(prim.GetPath()).
    All vehicle descendants, including instance proxies, are encoded on Play.
    """
    backend = 'physics_core'

    def __init__(self, query, vector_type, meters_per_unit, ignored_ids=(), both_sides=True):
        self.query = query; self.vector_type = vector_type
        self.meters_per_unit = float(meters_per_unit)
        self.ignored_ids = frozenset(ignored_ids)
        self.both_sides = bool(both_sides)
        self._closest = query.raycast_closest; self._all = query.raycast_all

    def _range_vectors(self, origin, direction, range_min, range_max):
        found, hit = self._closest(origin, direction, range_max/self.meters_per_unit, self.both_sides)
        distance = math.inf
        if found:
            ignored = self.ignored_ids
            if hit.collision not in ignored and hit.rigid_body not in ignored:
                distance = hit.distance*self.meters_per_unit
            else:
                def accept(candidate):
                    nonlocal distance
                    if candidate.collision not in ignored and candidate.rigid_body not in ignored:
                        distance = min(distance, candidate.distance*self.meters_per_unit)
                    return True
                self._all(origin, direction, range_max/self.meters_per_unit, accept, self.both_sides)
        if not math.isfinite(distance) or distance > range_max:
            return math.inf
        return -math.inf if distance < range_min else distance

    def range(self, origin, direction, range_min, range_max):
        vector = self.vector_type
        return self._range_vectors(vector(*map(float, origin)), vector(*map(float, direction)), range_min, range_max)

    def ranges(self, origins, directions, range_min, range_max):
        # NumPy's C-level tolist converts float scalars in bulk, replacing six
        # Python float conversions and two map objects per beam. Actual query
        # origins/directions remain the exact same float3 values and order.
        vector = self.vector_type; measure = self._range_vectors
        return [measure(vector(*origin), vector(*direction), range_min, range_max)
                for origin, direction in zip(origins.tolist(), directions.tolist())]


class LidarSensor:
    def __init__(self, stage, node, config_path, *, chassis_prim, vehicle_root, base_link_offset_m, frame_tree):
        from omni.physx import get_physx_scene_query_interface
        from pxr import UsdGeom
        from carb import Float3
        from sensor_msgs.msg import LaserScan
        self.stage = stage; self.node = node; self.path = Path(config_path)
        self.frame_tree = frame_tree
        self.chassis_prim = chassis_prim; self.vehicle_root = vehicle_root
        self.base_link_offset_m = np.asarray(base_link_offset_m, dtype=float)
        self.query = get_physx_scene_query_interface(); self.vector_type = Float3
        self.meters_per_unit = UsdGeom.GetStageMetersPerUnit(stage)
        if self.meters_per_unit <= 0:
            raise ValueError('Invalid stage metersPerUnit')
        self.message_type = LaserScan; self.publisher = None
        self.published_count = 0; self.last_scan = None; self.last_wall_time_s = 0.
        self.reload()

    def reload(self, frame_tree=None):
        """Read scan YAML and the central frame tree at every Play.

        TF publication belongs to the common frame-tree publisher. There is
        deliberately no per-sensor /tf_static writer or mount configuration.
        """
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        config = load_lidar_config(self.path)
        tree = self.frame_tree if frame_tree is None else frame_tree
        translation, quaternion = tree.resolve(config['frame_id'])
        translation = np.asarray(translation, dtype=float)
        if translation.shape != (3,) or not np.isfinite(translation).all():
            raise ValueError('Invalid central TF lidar translation')
        quaternion = np.asarray(quaternion, dtype=float)
        rotation = quaternion_matrix(quaternion)
        quaternion = quaternion/np.linalg.norm(quaternion)
        caster = self.make_raycaster(config)
        publisher = None
        if config['enabled']:
            qos = QoSProfile(depth=int(config['qos_depth']), reliability=ReliabilityPolicy.BEST_EFFORT)
            publisher = self.node.create_publisher(self.message_type, config['topic'], qos)
        self.close()
        self.config = config; self.publisher = publisher; self.frame_tree = tree
        self.mount_translation = translation; self.mount_q = quaternion
        self.mount_rotation = rotation
        self.mount_offset = self.base_link_offset_m+translation
        angles = np.radians(config['angle_min_deg']+np.arange(config['sample_count'])*config['angle_increment_deg'])
        self.directions = np.column_stack((np.cos(angles), np.sin(angles), np.zeros_like(angles)))
        self.chassis_directions = self.directions@self.mount_rotation.T
        self.chassis_offset = self.mount_offset/self.meters_per_unit
        self.caster = caster
        self.reset()

    def make_raycaster(self, config):
        if config['backend'] == 'physics_core':
            try:
                from omni.physics.core import get_physics_scene_query_interface
                from pxr import PhysicsSchemaTools, Usd
                core_query = get_physics_scene_query_interface()
                ignored_ids = set()
                if config['ignore_vehicle_colliders']:
                    root = self.stage.GetPrimAtPath(self.vehicle_root)
                    if not root:
                        raise ValueError('Cannot build lidar self-hit IDs: vehicle root missing')
                    for prim in Usd.PrimRange(root, Usd.TraverseInstanceProxies()):
                        ignored_ids.add(int(PhysicsSchemaTools.sdfPathToInt(prim.GetPath())))
                return PhysicsCoreRaycaster(core_query, self.vector_type, self.meters_per_unit,
                                             ignored_ids, config['both_sides'])
            except (ImportError, AttributeError) as error:
                # Older Isaac versions retain the established PhysX backend.
                self.node.get_logger().warning(f'Lidar physics_core unavailable; using physx: {error}')
        return CollisionRaycaster(self.query, self.vector_type, self.meters_per_unit,
                                  self.vehicle_root if config['ignore_vehicle_colliders'] else None,
                                  config['both_sides'])

    def reset(self):
        self.scan_start = None; self.sample_index = 0; self.ranges = []
        self.previous_time = None; self.previous_pose = None

    def update(self, sim_time, chassis_pose_xyzw):
        """Call once AFTER each physics step. Returns number of scans published.

        Rolling beam origins/directions interpolate the two adjacent chassis
        poses. Other dynamic colliders use the current physics tick's pose.
        """
        if not self.config['enabled']:
            return 0
        sim_time = float(sim_time); pose = np.asarray(chassis_pose_xyzw, dtype=float).copy()
        if pose.shape != (7,) or not np.isfinite(pose).all() or not math.isfinite(sim_time) or sim_time < 0:
            raise ValueError('Invalid lidar simulation time/chassis pose')
        quaternion_norm = np.linalg.norm(pose[3:])
        if quaternion_norm < 1e-12:
            raise ValueError('Invalid lidar chassis quaternion')
        pose[3:] /= quaternion_norm
        period = self.config['scan_time_s']; step = self.config['time_increment_s']
        if self.previous_time is not None and (sim_time < self.previous_time or sim_time-self.previous_time > period):
            self.reset()  # Never fabricate old scans following a clock jump.
        if self.scan_start is None:
            self.scan_start = sim_time
        started = time.perf_counter(); published = 0
        while self.scan_start+self.sample_index*step <= sim_time+1e-10:
            # Select exactly the due beams, keeping the original 1e-10s
            # boundary tolerance and the visible-arc / blind-sector timing.
            end_index = min(self.config['sample_count'],
                            math.floor((sim_time+1e-10-self.scan_start)/step)+1)
            # Division/subtraction may round differently from the original
            # addition comparison at a boundary after long simulation runs.
            while end_index > self.sample_index and self.scan_start+(end_index-1)*step > sim_time+1e-10:
                end_index -= 1
            while end_index < self.config['sample_count'] and self.scan_start+end_index*step <= sim_time+1e-10:
                end_index += 1
            indices = np.arange(self.sample_index, end_index)
            beam_times = self.scan_start+indices*step
            if self.previous_time is not None and sim_time > self.previous_time:
                fractions = np.clip((beam_times-self.previous_time)/(sim_time-self.previous_time), 0., 1.)
                origins, directions = beam_geometry(self.previous_pose, pose, fractions,
                                                     self.chassis_offset, self.chassis_directions[indices])
            else:
                rotation = quaternion_matrix(pose[3:])
                origins = np.broadcast_to(pose[:3]+rotation@self.chassis_offset, (len(indices), 3))
                directions = self.chassis_directions[indices]@rotation.T
            measure = self.caster.range
            minimum, maximum = self.config['range_min_m'], self.config['range_max_m']
            batch_measure = getattr(self.caster, 'ranges', None)
            if batch_measure is not None:
                self.ranges.extend(batch_measure(origins, directions, minimum, maximum))
            else:
                self.ranges.extend(measure(origin, direction, minimum, maximum)
                                   for origin, direction in zip(origins, directions))
            self.sample_index = end_index
            if self.sample_index == self.config['sample_count']:
                self.publish_scan(self.scan_start, self.ranges)
                self.scan_start += period; self.sample_index = 0; self.ranges = []
                published += 1
        self.previous_time = sim_time; self.previous_pose = pose
        self.last_wall_time_s = time.perf_counter()-started
        return published

    def publish_scan(self, start_time, ranges):
        message = self.message_type(); config = self.config
        stamp_time(message.header.stamp, start_time); message.header.frame_id = config['frame_id']
        message.angle_min = math.radians(config['angle_min_deg'])
        message.angle_increment = math.radians(config['angle_increment_deg'])
        message.angle_max = message.angle_min+(len(ranges)-1)*message.angle_increment
        message.time_increment = config['time_increment_s']; message.scan_time = config['scan_time_s']
        message.range_min = config['range_min_m']; message.range_max = config['range_max_m']
        message.ranges = ranges; message.intensities = []
        self.publisher.publish(message); self.published_count += 1
        finite = [r for r in ranges if math.isfinite(r)]
        self.last_scan = {'stamp': start_time, 'sample_count': len(ranges),
                          'finite_returns': len(finite), 'min_range_m': min(finite) if finite else None,
                          'max_range_m': max(finite) if finite else None}

    def status(self):
        return {'enabled': self.config['enabled'], 'backend': self.caster.backend,
                'requested_backend': self.config['backend'],
                'model': self.config['model'], 'config_path': str(self.path),
                'topic': self.config['topic'], 'frame_id': self.config['frame_id'],
                'base_frame': self.frame_tree.base_frame,
                'mount_translation_m': self.mount_translation.tolist(),
                'mount_rotation_xyzw': self.mount_q.tolist(),
                'scan_rate_hz': self.config['scan_rate_hz'], 'sample_count': self.config['sample_count'],
                'published_count': self.published_count, 'latest': self.last_scan,
                'last_update_wall_time_s': self.last_wall_time_s}

    def close(self):
        publisher = getattr(self, 'publisher', None)
        if publisher is not None:
            self.node.destroy_publisher(publisher)
            self.publisher = None
