"""ROS IMU sampled from the current Isaac rigid-body physics state.

The inputs use PhysX tensor conventions: pose XYZ + XYZW, then world-frame
linear velocity at the center of mass and world-frame angular velocity.
Accelerometer output is specific force, including +g on a stationary,
upright sensor. This module does not create a simulator, UI, or TF publisher.
"""
from pathlib import Path
import math
import time

import numpy as np
import yaml


def vector3(value, name):
    result = np.asarray(value, dtype=float)
    if result.ndim == 0:
        result = np.repeat(result, 3)
    if result.shape != (3,) or not np.isfinite(result).all():
        raise ValueError(f'IMU {name} must be three finite numbers')
    return result


def quaternion_matrix(xyzw):
    q = np.asarray(xyzw, dtype=float)
    norm = np.linalg.norm(q)
    if q.shape != (4,) or not math.isfinite(norm) or norm < 1e-12:
        raise ValueError('IMU needs a finite nonzero XYZW quaternion')
    x, y, z, w = q / norm
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])


def quaternion_product(a, b):
    """Hamilton product XYZW: world<-body times body<-sensor."""
    ax, ay, az, aw = a; bx, by, bz, bw = b
    return np.array([aw*bx+bw*ax+ay*bz-az*by,
                     aw*by+bw*ay+az*bx-ax*bz,
                     aw*bz+bw*az+ax*by-ay*bx,
                     aw*bw-ax*bx-ay*by-az*bz])


def cross3(a, b):
    """Fixed three-component cross product, avoiding NumPy axis dispatch."""
    ax, ay, az = a; bx, by, bz = b
    return np.array([ay*bz-az*by, az*bx-ax*bz, ax*by-ay*bx])


def rotation_vector_quaternion(vector):
    vector = np.asarray(vector, dtype=float)
    angle = np.linalg.norm(vector)
    scale = .5 if angle < 1e-12 else math.sin(angle/2)/angle
    return np.array([vector[0]*scale, vector[1]*scale, vector[2]*scale, math.cos(angle/2)])


def interpolate_pose(previous, current, fraction):
    """Translation lerp and shortest-path normalized quaternion lerp."""
    result = previous+fraction*(current-previous)
    qa, qb = previous[3:], current[3:]
    if np.dot(qa, qb) < 0:
        qb = -qb
    result[3:] = qa+fraction*(qb-qa)
    result[3:] /= np.linalg.norm(result[3:])
    return result


def covariance9(value, name):
    covariance = np.asarray(value, dtype=float)
    if covariance.shape != (9,) or not np.isfinite(covariance).all():
        raise ValueError(f'IMU {name} must contain nine finite numbers')
    matrix = covariance.reshape(3, 3)
    if (not np.allclose(matrix, matrix.T, atol=1e-12) or
            np.linalg.eigvalsh(matrix).min() < -1e-12):
        raise ValueError(f'IMU {name} must be symmetric positive semidefinite')
    return covariance.tolist()


def load_imu_config(path, physics_hz):
    config = yaml.safe_load(Path(path).read_text())
    if not isinstance(config, dict):
        raise ValueError('IMU configuration must be a mapping')
    hz = float(config['sample_rate_hz']); physics_hz = float(physics_hz)
    if not math.isfinite(physics_hz) or physics_hz <= 0:
        raise ValueError('IMU physics_hz must be positive and finite')
    mode = config.get('sampling_mode', 'interpolated_physics_state')
    if mode not in ('physics_state', 'interpolated_physics_state'):
        raise ValueError('IMU sampling_mode must be physics_state or interpolated_physics_state')
    if not math.isfinite(hz) or hz <= 0:
        raise ValueError('IMU sample_rate_hz must be positive and finite')
    if mode == 'physics_state' and hz > physics_hz*(1+1e-12):
        raise ValueError(f'IMU physics_state sample_rate_hz {hz} exceeds physics_hz={physics_hz}')
    if not config['topic'] or not config['frame_id'] or int(config['qos_depth']) < 1:
        raise ValueError('IMU topic/frame_id must be nonempty and QoS depth positive')
    cutoff = float(config['derivative_lowpass_hz'])
    if not math.isfinite(cutoff) or cutoff < 0:
        raise ValueError('IMU derivative_lowpass_hz must be finite and nonnegative')
    gap = float(config['clock_jump_reset_s'])
    if not math.isfinite(gap) or gap < 1.5/physics_hz:
        raise ValueError('IMU clock_jump_reset_s must be at least 1.5 physics steps')
    probability = float(config['dropout_probability'])
    if not math.isfinite(probability) or not 0 <= probability <= 1:
        raise ValueError('IMU dropout_probability must be between zero and one')
    config['gravity_world_m_s2'] = vector3(config['gravity_world_m_s2'], 'gravity_world_m_s2')
    for group, bias, noise, walk in (
            ('orientation', 'bias_rotation_vector_rad', 'noise_stddev_rad', 'bias_random_walk_rad_per_sqrt_s'),
            ('gyroscope', 'bias_rad_s', 'noise_stddev_rad_s', 'bias_random_walk_rad_s_per_sqrt_s'),
            ('accelerometer', 'bias_m_s2', 'noise_stddev_m_s2', 'bias_random_walk_m_s2_per_sqrt_s')):
        for key in (bias, noise, walk):
            value = vector3(config[group][key], f'{group}.{key}')
            if key != bias and np.any(value < 0):
                raise ValueError(f'IMU {group}.{key} cannot be negative')
            config[group][key] = value
        config[group]['covariance'] = covariance9(config[group]['covariance'], group+'.covariance')
    if config['random_seed'] is not None:
        seed = int(config['random_seed'])
        if seed != config['random_seed'] or seed < 0:
            raise ValueError('IMU random_seed must be a nonnegative integer or null')
        config['random_seed'] = seed
    config['sample_rate_hz'] = hz
    config['sampling_mode'] = mode
    config['derivative_lowpass_hz'] = cutoff
    config['clock_jump_reset_s'] = gap
    config['dropout_probability'] = probability
    return config


def stamp_time(stamp, seconds):
    stamp.sec, stamp.nanosec = divmod(round(seconds*1_000_000_000), 1_000_000_000)


class ImuSensor:
    def __init__(self, stage, node, config_path, *, base_link_offset_m, frame_tree,
                 physics_hz, center_of_mass_offset_m=(0., 0., 0.)):
        from sensor_msgs.msg import Imu
        self.stage = stage; self.node = node; self.path = Path(config_path)
        self.base_link_offset_m = vector3(base_link_offset_m, 'base_link_offset_m')
        self.center_of_mass_offset_m = vector3(center_of_mass_offset_m, 'center_of_mass_offset_m')
        self.frame_tree = frame_tree; self.physics_hz = float(physics_hz)
        self.meters_per_unit = 1.
        if stage is not None:
            from pxr import UsdGeom
            self.meters_per_unit = float(UsdGeom.GetStageMetersPerUnit(stage))
        if not math.isfinite(self.meters_per_unit) or self.meters_per_unit <= 0:
            raise ValueError('IMU stage metersPerUnit must be positive')
        self.message_type = Imu; self.publisher = None; self.message = None
        self.published_count = 0; self.dropped_count = 0; self.clock_reset_count = 0
        self.interpolated_sample_count = 0; self.last_batch_published = 0
        self.last_sample = None; self.last_wall_time_s = 0.
        self.reload()

    def reload(self, frame_tree=None, physics_hz=None, center_of_mass_offset_m=None):
        """Validate and reread YAML/mount at Play, replacing only this publisher."""
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        tree = self.frame_tree if frame_tree is None else frame_tree
        hz = self.physics_hz if physics_hz is None else float(physics_hz)
        config = load_imu_config(self.path, hz)
        translation, rotation = tree.resolve(config['frame_id'])
        mount_offset = self.base_link_offset_m+vector3(translation, 'TF translation')
        mount_q = np.asarray(rotation, dtype=float)
        mount_rotation = quaternion_matrix(mount_q)
        mount_q = mount_q/np.linalg.norm(mount_q)
        com = (self.center_of_mass_offset_m if center_of_mass_offset_m is None else
               vector3(center_of_mass_offset_m, 'center_of_mass_offset_m'))
        publisher = None; message = None
        if config['enabled']:
            # rclpy synchronously converts this Python object into a separate
            # C message on each publish call. Reuse is safe after publish
            # returns in this single-threaded physics callback.
            message = self.message_type()
            message.header.frame_id = config['frame_id']
            if config['orientation']['enabled']:
                message.orientation_covariance = list(config['orientation']['covariance'])
            else:
                message.orientation.w = 1.
                message.orientation_covariance = [-1., 0., 0., 0., 0., 0., 0., 0., 0.]
            message.angular_velocity_covariance = list(config['gyroscope']['covariance'])
            message.linear_acceleration_covariance = list(config['accelerometer']['covariance'])
            qos = QoSProfile(depth=int(config['qos_depth']), reliability=ReliabilityPolicy.BEST_EFFORT)
            publisher = self.node.create_publisher(self.message_type, config['topic'], qos)
        self.close()
        self.config = config; self.frame_tree = tree; self.physics_hz = hz
        self.mount_offset_m = mount_offset; self.mount_q = mount_q
        self.mount_rotation = mount_rotation; self.center_of_mass_offset_m = com.copy()
        self.publisher = publisher; self.message = message
        self.reset()

    def set_center_of_mass_offset_m(self, offset_m):
        """Pass body.get_coms()[0,:3] * metersPerUnit after creating the view."""
        self.center_of_mass_offset_m = vector3(offset_m, 'center_of_mass_offset_m')

    def reset(self):
        self.previous_time = None; self.previous_velocity = None; self.previous_pose = None
        self.anchor_time = None; self.next_sample_index = 0
        self.last_batch_published = 0
        self.filtered_acceleration = np.zeros(3); self.filtered_alpha = np.zeros(3)
        self.last_sample_time = None
        self.rng = np.random.default_rng(self.config['random_seed'])
        self.orientation_bias = self.config['orientation']['bias_rotation_vector_rad'].copy()
        self.gyro_bias = self.config['gyroscope']['bias_rad_s'].copy()
        self.accel_bias = self.config['accelerometer']['bias_m_s2'].copy()
        # YAML is applied on reload/reset, so zero-noise decisions need not run
        # six tiny NumPy reductions and allocate six zero arrays per sample.
        self._noise_active = tuple(bool(np.any(self.config[group][key])) for group, key in (
            ('orientation', 'noise_stddev_rad'), ('gyroscope', 'noise_stddev_rad_s'),
            ('accelerometer', 'noise_stddev_m_s2')))
        self._walk_active = tuple(bool(np.any(self.config[group][key])) for group, key in (
            ('orientation', 'bias_random_walk_rad_per_sqrt_s'),
            ('gyroscope', 'bias_random_walk_rad_s_per_sqrt_s'),
            ('accelerometer', 'bias_random_walk_m_s2_per_sqrt_s')))
        self._error_active = tuple(bool(np.any(bias)) or noise or walk for bias, noise, walk in
                                   zip((self.orientation_bias, self.gyro_bias, self.accel_bias),
                                       self._noise_active, self._walk_active))

    def update(self, sim_time, chassis_pose_xyzw, velocity_world6):
        """Sample AFTER a physics step; return the messages published this tick.

        Interpolation mode emits timestamped samples only from the completed
        previous/current physics interval. These are interpolated states,
        not independent higher-frequency physics measurements. Strict
        physics_state mode emits at most one actual-tick measurement.
        """
        if not self.config['enabled']:
            return 0
        started = time.perf_counter()
        now = float(sim_time)
        pose = np.asarray(chassis_pose_xyzw, dtype=float).copy()
        velocity = np.asarray(velocity_world6, dtype=float).copy()
        if (not math.isfinite(now) or now < 0 or pose.shape != (7,) or
                not np.isfinite(pose).all() or velocity.shape != (6,) or not np.isfinite(velocity).all()):
            raise ValueError('Invalid IMU simulation time, chassis pose, or world velocity')
        quaternion_norm = np.linalg.norm(pose[3:])
        if not math.isfinite(quaternion_norm) or quaternion_norm < 1e-12:
            raise ValueError('IMU needs a finite nonzero XYZW quaternion')
        pose[3:] /= quaternion_norm
        velocity[:3] *= self.meters_per_unit
        previous = self.previous_time
        if previous is not None and (now < previous-1e-12 or now-previous > self.config['clock_jump_reset_s']):
            self.clock_reset_count += 1
            self.reset(); previous = None
        if previous is not None and now <= previous+1e-12:
            self.last_batch_published = 0
            return 0
        previous_velocity = self.previous_velocity
        previous_pose = self.previous_pose
        if previous is None:
            acceleration = np.zeros(3); alpha = np.zeros(3)
        else:
            dt = now-previous
            acceleration = (velocity[:3]-previous_velocity[:3])/dt
            alpha = (velocity[3:]-previous_velocity[3:])/dt
            cutoff = self.config['derivative_lowpass_hz']
            if cutoff > 0:
                fraction = -math.expm1(-2*math.pi*cutoff*dt)
                acceleration = self.filtered_acceleration+fraction*(acceleration-self.filtered_acceleration)
                alpha = self.filtered_alpha+fraction*(alpha-self.filtered_alpha)
            if np.dot(previous_pose[3:], pose[3:]) < 0:
                pose[3:] *= -1  # Keep quaternion representation continuous.
        self.filtered_acceleration = acceleration; self.filtered_alpha = alpha
        self.previous_time = now; self.previous_velocity = velocity; self.previous_pose = pose
        if self.anchor_time is None:
            self.anchor_time = now
        rate = self.config['sample_rate_hz']
        due = self.anchor_time+self.next_sample_index/rate
        if now+1e-10 < due:
            self.last_batch_published = 0
            self.last_wall_time_s = time.perf_counter()-started
            return 0
        # Direct index computation avoids accumulating phase error. Strict
        # mode samples the current tick; interpolation uses completed history.
        next_index = math.floor((now-self.anchor_time+1e-10)*rate)+1
        published = 0
        if self.config['sampling_mode'] == 'physics_state':
            self.dropped_count += max(0, next_index-self.next_sample_index-1)
            published = self._sample_state(now, pose, velocity, acceleration, alpha)
        else:
            for sample_index in range(self.next_sample_index, next_index):
                sample_time = min(now, self.anchor_time+sample_index/rate)
                if previous is None:
                    sample_pose, sample_velocity, fraction = pose, velocity, 1.
                else:
                    fraction = min(1., max(0., (sample_time-previous)/(now-previous)))
                    sample_pose = interpolate_pose(previous_pose, pose, fraction)
                    sample_velocity = previous_velocity+fraction*(velocity-previous_velocity)
                count = self._sample_state(sample_time, sample_pose, sample_velocity, acceleration, alpha)
                published += count
                if count and 1e-10 < fraction < 1-1e-10:
                    self.interpolated_sample_count += 1
        self.next_sample_index = next_index
        self.last_batch_published = published
        self.last_wall_time_s = time.perf_counter()-started
        return published

    def _sample_state(self, stamp, pose, velocity, acceleration, alpha):
        sample_dt = 0. if self.last_sample_time is None else stamp-self.last_sample_time
        self.last_sample_time = stamp
        self._advance_bias(sample_dt)
        if self.config['dropout_probability'] > 0 and self.rng.random() < self.config['dropout_probability']:
            self.dropped_count += 1
            return 0
        body_rotation = quaternion_matrix(pose[3:])
        sensor_rotation = body_rotation@self.mount_rotation
        lever_world = body_rotation@(self.mount_offset_m-self.center_of_mass_offset_m)
        omega = velocity[3:]
        sensor_acceleration = acceleration+cross3(alpha, lever_world)+cross3(omega, cross3(omega, lever_world))
        proper_acceleration = sensor_rotation.T@(sensor_acceleration-self.config['gravity_world_m_s2'])
        gyro = sensor_rotation.T@omega
        orientation = quaternion_product(pose[3:]/np.linalg.norm(pose[3:]), self.mount_q)
        if self._error_active[0]:
            error = self.orientation_bias+self._noise(self.config['orientation']['noise_stddev_rad'], self._noise_active[0])
            orientation = quaternion_product(orientation, rotation_vector_quaternion(error))
        orientation /= np.linalg.norm(orientation)
        if self._error_active[1]:
            gyro += self.gyro_bias+self._noise(self.config['gyroscope']['noise_stddev_rad_s'], self._noise_active[1])
        if self._error_active[2]:
            proper_acceleration += self.accel_bias+self._noise(self.config['accelerometer']['noise_stddev_m_s2'], self._noise_active[2])
        self.publish_sample(stamp, orientation, gyro, proper_acceleration)
        return 1

    def _noise(self, stddev, active):
        return self.rng.normal(size=3)*stddev if active else np.zeros(3)

    def _advance_bias(self, dt):
        if dt <= 0 or not any(self._walk_active):
            return
        scale = math.sqrt(dt)
        if self._walk_active[0]:
            self.orientation_bias += self._noise(self.config['orientation']['bias_random_walk_rad_per_sqrt_s'], True)*scale
        if self._walk_active[1]:
            self.gyro_bias += self._noise(self.config['gyroscope']['bias_random_walk_rad_s_per_sqrt_s'], True)*scale
        if self._walk_active[2]:
            self.accel_bias += self._noise(self.config['accelerometer']['bias_random_walk_m_s2_per_sqrt_s'], True)*scale

    def publish_sample(self, stamp, orientation, gyro, acceleration):
        message = self.message
        stamp_time(message.header.stamp, stamp)
        if self.config['orientation']['enabled']:
            message.orientation.x, message.orientation.y, message.orientation.z, message.orientation.w = map(float, orientation)
        message.angular_velocity.x, message.angular_velocity.y, message.angular_velocity.z = map(float, gyro)
        message.linear_acceleration.x, message.linear_acceleration.y, message.linear_acceleration.z = map(float, acceleration)
        self.publisher.publish(message)
        self.published_count += 1
        self.last_sample = {'stamp': float(stamp), 'orientation_xyzw': orientation.tolist(),
                            'angular_velocity_rad_s': gyro.tolist(), 'linear_acceleration_m_s2': acceleration.tolist()}

    def close(self):
        if self.publisher is not None:
            self.node.destroy_publisher(self.publisher)
            self.publisher = None
        self.message = None

    def status(self):
        return {'enabled': bool(self.config['enabled']), 'topic': self.config['topic'],
                'frame_id': self.config['frame_id'], 'sample_rate_hz': self.config['sample_rate_hz'],
                'sampling_mode': self.config['sampling_mode'],
                'interpolated_physics_states': self.config['sampling_mode'] == 'interpolated_physics_state',
                'interpolated_sample_count': self.interpolated_sample_count,
                'last_batch_published': self.last_batch_published,
                'physics_hz': self.physics_hz, 'published_count': self.published_count,
                'dropped_count': self.dropped_count, 'clock_reset_count': self.clock_reset_count,
                'mount_from_chassis_m': self.mount_offset_m.tolist(),
                'center_of_mass_offset_m': self.center_of_mass_offset_m.tolist(),
                'last_update_wall_s': self.last_wall_time_s, 'last_sample': self.last_sample}
