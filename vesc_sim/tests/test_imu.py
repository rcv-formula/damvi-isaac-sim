"""IMU physics, frame, clock, and ROS lifecycle checks without an Isaac app."""
import copy
import math
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from imu import ImuSensor, load_imu_config, quaternion_matrix, quaternion_product, stamp_time


CONFIG = Path(__file__).resolve().parents[1]/'config/imu.yaml'
IDENTITY_POSE = [0., 0., 0., 0., 0., 0., 1.]


def vector():
    return SimpleNamespace(x=0., y=0., z=0.)


class FakeMessage:
    def __init__(self):
        self.header = SimpleNamespace(stamp=SimpleNamespace(sec=0, nanosec=0), frame_id='')
        self.orientation = SimpleNamespace(x=0., y=0., z=0., w=0.)
        self.angular_velocity = vector(); self.linear_acceleration = vector()


class FakeNode:
    def __init__(self):
        self.messages = []; self.created = []; self.destroyed = []

    def create_publisher(self, message_type, topic, qos):
        # rclpy copies fields to a separate C message before publish returns.
        # Capturing references would incorrectly alias a reused Python message.
        publisher = SimpleNamespace(publish=lambda message: self.messages.append(copy.deepcopy(message)))
        self.created.append((topic, qos, publisher))
        return publisher

    def destroy_publisher(self, publisher):
        self.destroyed.append(publisher)


class FakeFrameTree:
    def __init__(self, translation=(0., 0., 0.), quaternion=(0., 0., 0., 1.), frame_id='imu'):
        self.translation = translation; self.quaternion = quaternion
        self.frame_id = frame_id

    def resolve(self, frame):
        if frame != self.frame_id:
            raise ValueError('Unknown frame')
        return np.array(self.translation), np.array(self.quaternion)


@pytest.fixture
def sensor_factory(monkeypatch, tmp_path):
    monkeypatch.setitem(sys.modules, 'sensor_msgs', SimpleNamespace())
    monkeypatch.setitem(sys.modules, 'sensor_msgs.msg', SimpleNamespace(Imu=FakeMessage))
    monkeypatch.setitem(sys.modules, 'rclpy', SimpleNamespace())
    monkeypatch.setitem(sys.modules, 'rclpy.qos', SimpleNamespace(
        QoSProfile=lambda **kwargs: SimpleNamespace(**kwargs),
        ReliabilityPolicy=SimpleNamespace(BEST_EFFORT='best_effort')))
    def create(config=None, mount=(0., 0., 0.), mount_q=(0., 0., 0., 1.),
               base=(0., 0., 0.), com=(0., 0., 0.), physics_hz=1000):
        path = tmp_path/f'imu{len(list(tmp_path.iterdir()))}.yaml'
        path.write_text(yaml.safe_dump(config or yaml.safe_load(CONFIG.read_text())))
        node = FakeNode()
        sensor = ImuSensor(None, node, path, base_link_offset_m=base,
                           frame_tree=FakeFrameTree(mount, mount_q), physics_hz=physics_hz,
                           center_of_mass_offset_m=com)
        return sensor, node
    return create


def xyz(vector):
    return [vector.x, vector.y, vector.z]


def test_upright_stationary_specific_force_and_first_velocity_warmup(sensor_factory):
    sensor, node = sensor_factory()
    assert sensor.update(0., IDENTITY_POSE, [20, -10, 5, 0, 0, 0]) == 1
    assert xyz(node.messages[-1].linear_acceleration) == pytest.approx([0, 0, 9.81])
    assert node.messages[-1].header.frame_id == 'imu'
    assert sensor.update(.001, IDENTITY_POSE, [20, -10, 5, 0, 0, 0]) == 1
    assert xyz(node.messages[-1].linear_acceleration) == pytest.approx([0, 0, 9.81])
    assert len(node.messages) == 2
    assert sensor.status()['sample_rate_hz'] == 1000


def test_body_and_mount_quaternion_composition_and_vector_axes(sensor_factory):
    s = math.sqrt(.5)
    sensor, node = sensor_factory(mount_q=(s, 0., 0., s))
    sensor.update(0., [0, 0, 0, 0, 0, s, s], [0, 0, 0, 1, 2, 3])
    sample = node.messages[-1]
    assert [sample.orientation.x, sample.orientation.y, sample.orientation.z, sample.orientation.w] == pytest.approx([.5]*4)
    assert xyz(sample.angular_velocity) == pytest.approx([2, 3, 1])
    assert xyz(sample.linear_acceleration) == pytest.approx([0, 9.81, 0], abs=1e-12)


def test_fixed_quaternion_kernel_matches_rotation_composition():
    rng = np.random.default_rng(461)
    for _ in range(1000):
        a, b = rng.normal(size=(2, 4)); a /= np.linalg.norm(a); b /= np.linalg.norm(b)
        np.testing.assert_allclose(quaternion_matrix(quaternion_product(a, b)),
                                   quaternion_matrix(a)@quaternion_matrix(b), atol=2e-15)


def test_random_physical_states_against_world_vector_reference(sensor_factory):
    rng = np.random.default_rng(692)
    mount_q = rng.normal(size=4); mount_q /= np.linalg.norm(mount_q)
    sensor, node = sensor_factory(mount=(.27, -.03, .06), mount_q=mount_q,
                                  base=(-.1, 0, -.042), com=(-.0032, .01, .0054))
    previous_velocity = None
    mount_rotation = quaternion_matrix(mount_q)
    lever = np.array([.1732, -.04, .0126])
    for index in range(1000):
        pose = rng.normal(size=7); pose[3:] /= np.linalg.norm(pose[3:])
        velocity = rng.normal(size=6)
        sensor.update(index*.001, pose, velocity)
        body_rotation = quaternion_matrix(pose[3:]); rotation = body_rotation@mount_rotation
        acceleration = np.zeros(3) if previous_velocity is None else (velocity[:3]-previous_velocity[:3])/.001
        alpha = np.zeros(3) if previous_velocity is None else (velocity[3:]-previous_velocity[3:])/.001
        r = body_rotation@lever; omega = velocity[3:]
        expected = rotation.T@(acceleration+np.cross(alpha, r)+np.cross(omega, np.cross(omega, r))+[0, 0, 9.81])
        np.testing.assert_allclose(xyz(node.messages[-1].linear_acceleration), expected, atol=1e-9, rtol=1e-12)
        np.testing.assert_allclose(xyz(node.messages[-1].angular_velocity), rotation.T@omega, atol=2e-15)
        sample_q = sensor.last_sample['orientation_xyzw']
        np.testing.assert_allclose(quaternion_matrix(sample_q), rotation, atol=2e-15)
        previous_velocity = velocity


def test_com_lever_arm_centripetal_and_tangential_acceleration(sensor_factory):
    # Total sensor lever is x=2m after accounting for base + mount - COM.
    sensor, node = sensor_factory(mount=(2.5, 0, 0), base=(-.1, 0, 0), com=(.4, 0, 0))
    sensor.update(0., IDENTITY_POSE, [0, 0, 0, 0, 0, 1])
    assert xyz(node.messages[-1].linear_acceleration) == pytest.approx([-2, 0, 9.81])
    sensor.update(.01, IDENTITY_POSE, [.03, 0, 0, 0, 0, 1.01])
    # a_COM=3x, alpha=1z; alpha cross r = 2y; omega cross omega cross r=-2.0402x.
    assert xyz(node.messages[-1].linear_acceleration) == pytest.approx([.9598, 2, 9.81])
    sensor.set_center_of_mass_offset_m([2.4, 0, 0])
    sensor.update(.02, IDENTITY_POSE, [.03, 0, 0, 0, 0, 1.01])
    assert xyz(node.messages[-1].linear_acceleration) == pytest.approx([0, 0, 9.81])


def test_linear_acceleration_and_derivative_filter(sensor_factory):
    config = yaml.safe_load(CONFIG.read_text()); config['derivative_lowpass_hz'] = 10.
    sensor, node = sensor_factory(config)
    sensor.update(0., IDENTITY_POSE, [0, 0, 0, 0, 0, 0])
    sensor.update(.001, IDENTITY_POSE, [.1, 0, 0, 0, 0, 0])
    assert xyz(node.messages[-1].linear_acceleration) == pytest.approx([100*(1-math.exp(-2*math.pi*.01)), 0, 9.81])


def test_free_fall_and_stage_distance_units(sensor_factory):
    sensor, node = sensor_factory()
    sensor.meters_per_unit = .01
    sensor.update(0., IDENTITY_POSE, [0]*6)
    # PhysX reports -0.981 centimetres/sec after one millisecond of free fall.
    sensor.update(.001, IDENTITY_POSE, [0, 0, -.981, 0, 0, 0])
    assert xyz(node.messages[-1].linear_acceleration) == pytest.approx([0, 0, 0], abs=1e-12)


def test_noninteger_sampling_ratio_has_no_drift_or_duplicate_ticks(sensor_factory):
    config = yaml.safe_load(CONFIG.read_text()); config['sample_rate_hz'] = 333.
    config['sampling_mode'] = 'physics_state'
    sensor, node = sensor_factory(config)
    for t in np.arange(0., 1.0001, .001):
        assert sensor.update(t, IDENTITY_POSE, [0]*6) in (0, 1)
    stamps = [m.header.stamp.sec+m.header.stamp.nanosec*1e-9 for m in node.messages]
    assert len(stamps) == 334
    assert set(np.round(np.diff(stamps), 8)) == {.003, .004}
    expected = np.arange(334)/333
    assert np.all(np.array(stamps)+1e-10 >= expected)
    assert np.max(np.array(stamps)-expected) < .001+1e-10
    assert sensor.dropped_count == 0


def test_clock_reset_forward_gap_and_repeated_tick_do_not_create_spikes_or_backlog(sensor_factory):
    sensor, node = sensor_factory()
    sensor.update(3600., IDENTITY_POSE, [0]*6)
    sensor.update(3600.001, IDENTITY_POSE, [0]*6)
    count = len(node.messages)
    assert sensor.update(3600.001, IDENTITY_POSE, [200, 0, 0, 0, 0, 0]) == 0
    assert len(node.messages) == count
    assert sensor.update(0., IDENTITY_POSE, [200, 0, 0, 0, 0, 0]) == 1
    assert xyz(node.messages[-1].linear_acceleration) == pytest.approx([0, 0, 9.81])
    assert sensor.update(1., IDENTITY_POSE, [800, 0, 0, 0, 0, 0]) == 1
    assert xyz(node.messages[-1].linear_acceleration) == pytest.approx([0, 0, 9.81])
    assert sensor.clock_reset_count == 2
    assert len(node.messages) == count+2


def test_physics_rate_limit_and_invalid_inputs(sensor_factory, tmp_path):
    config = yaml.safe_load(CONFIG.read_text()); config['sampling_mode'] = 'physics_state'
    with pytest.raises(ValueError, match='physics_hz'):
        sensor_factory(config, physics_hz=200.)
    sensor, node = sensor_factory()
    with pytest.raises(ValueError, match='quaternion'):
        sensor.update(0., [0]*7, [0]*6)
    with pytest.raises(ValueError, match='Invalid IMU'):
        sensor.update(0., IDENTITY_POSE, [0, 0, math.nan, 0, 0, 0])


def test_interpolated_200hz_input_publishes_uniform_1000hz_completed_batches(sensor_factory):
    sensor, node = sensor_factory(physics_hz=200.)
    assert sensor.update(0., IDENTITY_POSE, [0]*6) == 1
    for index in range(1, 201):
        now = index/200
        assert sensor.update(now, IDENTITY_POSE, [0]*6) == 5
        stamps = [m.header.stamp.sec+m.header.stamp.nanosec*1e-9 for m in node.messages[-5:]]
        assert max(stamps) <= now+1e-12
    stamps = [m.header.stamp.sec*1_000_000_000+m.header.stamp.nanosec for m in node.messages]
    assert stamps == list(range(0, 1_000_000_001, 1_000_000))
    assert sensor.dropped_count == 0
    assert sensor.interpolated_sample_count == 800
    assert sensor.status()['sampling_mode'] == 'interpolated_physics_state'
    assert sensor.status()['last_batch_published'] == 5


def test_interpolated_constant_acceleration_and_free_fall(sensor_factory):
    sensor, node = sensor_factory(physics_hz=200.)
    sensor.update(0., IDENTITY_POSE, [0]*6)
    sensor.update(.005, IDENTITY_POSE, [.01, -.005, .0025, 0, 0, 0])
    for sample in node.messages[-5:]:
        assert xyz(sample.linear_acceleration) == pytest.approx([2, -1, 10.31])
    sensor.reset(); sensor.update(0., IDENTITY_POSE, [0]*6)
    sensor.update(.005, IDENTITY_POSE, [0, 0, -.04905, 0, 0, 0])
    for sample in node.messages[-5:]:
        assert xyz(sample.linear_acceleration) == pytest.approx([0, 0, 0], abs=1e-12)


def test_interpolated_rotating_pose_velocity_and_com_lever(sensor_factory):
    s = math.sqrt(.5); mount_q = [s, 0, 0, s]
    sensor, node = sensor_factory(physics_hz=200., mount=(.27, 0, .06), mount_q=mount_q,
                                  base=(-.1, 0, -.042), com=(-.0032, 0, .0054))
    sensor.update(0., IDENTITY_POSE, [0]*6)
    final_q = np.array([0, 0, math.sin(.05), math.cos(.05)])
    sensor.update(.005, [0, 0, 0, *-final_q], [.01, 0, 0, 0, 0, 40])
    for index, sample in enumerate(node.messages[-5:], 1):
        fraction = index/5
        q = np.array([0, 0, fraction*math.sin(.05), 1+fraction*(math.cos(.05)-1)])
        q /= np.linalg.norm(q)
        body_rotation = quaternion_matrix(q); rotation = body_rotation@quaternion_matrix(mount_q)
        lever = body_rotation@np.array([.1732, 0, .0126]); omega = np.array([0, 0, 40*fraction])
        acceleration = np.array([2, 0, 0])+np.cross([0, 0, 8000], lever)+np.cross(omega, np.cross(omega, lever))
        np.testing.assert_allclose(xyz(sample.linear_acceleration), rotation.T@(acceleration+[0, 0, 9.81]), atol=1e-10)
        np.testing.assert_allclose(xyz(sample.angular_velocity), rotation.T@omega, atol=1e-12)
        measured_q = [sample.orientation.x, sample.orientation.y, sample.orientation.z, sample.orientation.w]
        np.testing.assert_allclose(quaternion_matrix(measured_q), rotation, atol=1e-14)


def test_interpolated_noninteger_rate_has_uniform_sample_phase(sensor_factory):
    config = yaml.safe_load(CONFIG.read_text()); config['sample_rate_hz'] = 333.
    sensor, node = sensor_factory(config, physics_hz=200.)
    batches = [sensor.update(index/200, IDENTITY_POSE, [0]*6) for index in range(201)]
    assert set(batches) == {1, 2}
    stamps = np.array([m.header.stamp.sec+m.header.stamp.nanosec*1e-9 for m in node.messages])
    assert len(stamps) == 334
    np.testing.assert_allclose(stamps, np.arange(334)/333, atol=.51e-9, rtol=0)
    assert sensor.dropped_count == 0


def test_interpolated_clock_gaps_reset_without_old_sample_burst(sensor_factory):
    sensor, node = sensor_factory(physics_hz=200.)
    assert sensor.update(10., IDENTITY_POSE, [200, 0, 0, 0, 0, 0]) == 1
    assert sensor.update(10.005, IDENTITY_POSE, [200, 0, 0, 0, 0, 0]) == 5
    assert sensor.update(11., IDENTITY_POSE, [800, 0, 0, 0, 0, 0]) == 1
    assert xyz(node.messages[-1].linear_acceleration) == pytest.approx([0, 0, 9.81])
    assert sensor.update(0., IDENTITY_POSE, [0]*6) == 1
    assert sensor.update(0., IDENTITY_POSE, [0]*6) == 0
    assert sensor.clock_reset_count == 2
    assert len(node.messages) == 8


def test_interpolation_preserves_noise_walk_dropout_sample_clock(sensor_factory):
    config = yaml.safe_load(CONFIG.read_text()); config['dropout_probability'] = .2
    for group, bias, noise, walk in (
            ('orientation', 'bias_rotation_vector_rad', 'noise_stddev_rad', 'bias_random_walk_rad_per_sqrt_s'),
            ('gyroscope', 'bias_rad_s', 'noise_stddev_rad_s', 'bias_random_walk_rad_s_per_sqrt_s'),
            ('accelerometer', 'bias_m_s2', 'noise_stddev_m_s2', 'bias_random_walk_m_s2_per_sqrt_s')):
        config[group][bias] = [.01, -.02, .03]
        config[group][noise] = [.002, .003, .004]
        config[group][walk] = [.003, .002, .001]
    interpolated, interpolated_node = sensor_factory(config, physics_hz=200.)
    strict_config = copy.deepcopy(config); strict_config['sampling_mode'] = 'physics_state'
    strict, strict_node = sensor_factory(strict_config, physics_hz=1000.)
    for index in range(201):
        now = index/200
        interpolated.update(now, IDENTITY_POSE, [2*now, -now, 0, 0, 0, 0])
    for index in range(1001):
        now = index/1000
        strict.update(now, IDENTITY_POSE, [2*now, -now, 0, 0, 0, 0])
    assert interpolated.published_count == strict.published_count
    assert interpolated.dropped_count == strict.dropped_count
    for a, b in zip(interpolated_node.messages, strict_node.messages):
        assert (a.header.stamp.sec, a.header.stamp.nanosec) == (b.header.stamp.sec, b.header.stamp.nanosec)
        np.testing.assert_allclose(xyz(a.angular_velocity), xyz(b.angular_velocity), atol=1e-13)
        np.testing.assert_allclose(xyz(a.linear_acceleration), xyz(b.linear_acceleration), atol=1e-12)
        np.testing.assert_allclose([a.orientation.x, a.orientation.y, a.orientation.z, a.orientation.w],
                                   [b.orientation.x, b.orientation.y, b.orientation.z, b.orientation.w], atol=1e-13)


def test_bias_noise_seed_reset_and_unavailable_orientation(sensor_factory):
    config = yaml.safe_load(CONFIG.read_text())
    config['orientation']['enabled'] = False
    config['gyroscope']['bias_rad_s'] = [1, 2, 3]
    config['gyroscope']['noise_stddev_rad_s'] = [.1, .2, .3]
    config['accelerometer']['bias_m_s2'] = [1, 2, 3]
    config['accelerometer']['bias_random_walk_m_s2_per_sqrt_s'] = [.2, .2, .2]
    sensor, node = sensor_factory(config)
    sensor.update(0., IDENTITY_POSE, [0]*6)
    first = copy.deepcopy(node.messages[-1])
    assert first.orientation_covariance[0] == -1
    assert xyz(first.linear_acceleration) == pytest.approx([1, 2, 12.81])
    sensor.update(.001, IDENTITY_POSE, [0]*6)
    assert xyz(node.messages[-1].linear_acceleration) != pytest.approx([1, 2, 12.81])
    sensor.reset(); sensor.update(0., IDENTITY_POSE, [0]*6)
    assert xyz(node.messages[-1].angular_velocity) == xyz(first.angular_velocity)
    assert xyz(node.messages[-1].linear_acceleration) == xyz(first.linear_acceleration)


def test_dropout_and_reload_lifecycle_keep_tf_external(sensor_factory):
    sensor, node = sensor_factory()
    old_publisher = sensor.publisher
    config = yaml.safe_load(sensor.path.read_text()); config['dropout_probability'] = 1.
    config['sample_rate_hz'] = 500
    sensor.path.write_text(yaml.safe_dump(config))
    sensor.reload(frame_tree=FakeFrameTree((.27, 0, .06)))
    assert node.destroyed == [old_publisher]
    assert sensor.mount_offset_m == pytest.approx([.27, 0, .06])
    assert all(topic == '/imu/data' for topic, _, _ in node.created)
    assert sensor.update(0., IDENTITY_POSE, [0]*6) == 0
    assert sensor.dropped_count == 1
    config['enabled'] = False; sensor.path.write_text(yaml.safe_dump(config))
    sensor.reload(); assert sensor.publisher is None
    assert sensor.update(.001, IDENTITY_POSE, [0]*6) == 0


def test_reused_message_keeps_prior_samples_and_reload_metadata_independent(sensor_factory):
    sensor, node = sensor_factory()
    sensor.update(0., IDENTITY_POSE, [0]*6)
    first = node.messages[-1]; reused = sensor.message
    sensor.update(.001, IDENTITY_POSE, [.001, 0, 0, 0, 0, 0])
    second = node.messages[-1]
    assert sensor.message is reused
    assert first.header.stamp.nanosec == 0
    assert second.header.stamp.nanosec == 1_000_000
    assert xyz(first.linear_acceleration) == pytest.approx([0, 0, 9.81])
    assert xyz(second.linear_acceleration) == pytest.approx([1, 0, 9.81])
    config = yaml.safe_load(sensor.path.read_text())
    config['topic'] = '/imu/changed'; config['frame_id'] = 'imu_changed'
    config['orientation']['enabled'] = False
    for group, variance in [('orientation', .1), ('gyroscope', .2), ('accelerometer', .3)]:
        config[group]['covariance'] = [variance, 0., 0., 0., variance, 0., 0., 0., variance]
    sensor.path.write_text(yaml.safe_dump(config))
    sensor.reload(frame_tree=FakeFrameTree(frame_id='imu_changed'))
    sensor.update(.002, IDENTITY_POSE, [0]*6)
    changed = node.messages[-1]
    assert sensor.message is not reused
    assert node.created[-1][0] == '/imu/changed'
    assert changed.header.frame_id == 'imu_changed'
    assert changed.orientation.w == 1.
    assert changed.orientation_covariance == [-1., 0., 0., 0., 0., 0., 0., 0., 0.]
    assert changed.angular_velocity_covariance == config['gyroscope']['covariance']
    assert changed.linear_acceleration_covariance == config['accelerometer']['covariance']
    assert first.header.frame_id == 'imu'
    assert first.orientation_covariance == [0.]*9
    config['orientation']['enabled'] = True
    sensor.path.write_text(yaml.safe_dump(config)); sensor.reload()
    sensor.update(.003, IDENTITY_POSE, [0]*6)
    assert node.messages[-1].orientation_covariance == config['orientation']['covariance']


def test_invalid_reload_keeps_valid_publisher_and_message(sensor_factory):
    sensor, node = sensor_factory()
    publisher, message = sensor.publisher, sensor.message
    config = yaml.safe_load(sensor.path.read_text())
    config['gyroscope']['covariance'] = [1, 0, 0, 0, -1, 0, 0, 0, 1]
    sensor.path.write_text(yaml.safe_dump(config))
    with pytest.raises(ValueError, match='positive semidefinite'):
        sensor.reload()
    assert sensor.publisher is publisher and sensor.message is message
    assert node.destroyed == []
    sensor.update(0., IDENTITY_POSE, [0]*6)
    assert node.messages[-1].angular_velocity_covariance == [0.]*9


def test_covariance_validation_and_timestamp_carry(tmp_path):
    config = yaml.safe_load(CONFIG.read_text())
    config['accelerometer']['covariance'] = [1, 0, 0, 0, -1, 0, 0, 0, 1]
    path = tmp_path/'imu.yaml'; path.write_text(yaml.safe_dump(config))
    with pytest.raises(ValueError, match='positive semidefinite'):
        load_imu_config(path, 1000)
    stamp = SimpleNamespace()
    stamp_time(stamp, 1.9999999999)
    assert (stamp.sec, stamp.nanosec) == (2, 0)
