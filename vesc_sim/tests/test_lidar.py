"""Geometry/timing checks without importing ROS or starting an Isaac process."""
import math
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lidar import (CollisionRaycaster, PhysicsCoreRaycaster, LidarSensor, beam_geometry, interpolate_pose,
                   load_lidar_config, quaternion_matrix, rpy_quaternion, stamp_time)


CONFIG = Path(__file__).resolve().parents[1]/'config/lidar.yaml'


def test_official_and_recorded_scan_geometry():
    config = load_lidar_config(CONFIG)
    assert config['sample_count'] == 1081
    assert config['scan_time_s'] == .025
    assert config['time_increment_s'] == pytest.approx(1/57600)
    assert (config['sample_count']-1)*config['time_increment_s'] == pytest.approx(.01875)
    assert config['range_min_m'] == .06
    assert config['range_max_m'] == 30
    assert not {'mount', 'parent_frame_id', 'publish_static_tf'} & config.keys()


class FakeQuery:
    def __init__(self, hit=None, hits=()):
        self.hit = hit or {'hit': False}; self.hits = hits

    def raycast_closest(self, origin, direction, maximum, both_sides):
        return self.hit

    def raycast_all(self, origin, direction, maximum, callback, both_sides):
        for hit in self.hits:
            assert callback(SimpleNamespace(**hit))


def caster(query):
    return CollisionRaycaster(query, lambda *xyz: xyz, 1., '/World/RC_Car')


def test_self_hit_filter_does_not_depend_on_all_hit_order():
    query = FakeQuery({'hit': True, 'distance': .1, 'collision': '/World/RC_Car/Chassis'}, [
        {'distance': 8., 'collision': '/World/Wall8', 'rigid_body': ''},
        {'distance': .1, 'collision': '/World/RC_Car/Wheel', 'rigid_body': ''},
        {'distance': 2., 'collision': '/World/Wall2', 'rigid_body': ''},
    ])
    assert caster(query).range([0, 0, 0], [1, 0, 0], .06, 30) == 2.
    # Prefixes must not remove unrelated objects with similar names.
    assert not caster(query).ignored('/World/RC_Car_obstacle')


@pytest.mark.parametrize('distance,expected', [(None, math.inf), (.03, -math.inf), (.06, .06), (31., math.inf)])
def test_out_of_range_values(distance, expected):
    hit = {'hit': distance is not None, 'distance': distance, 'collision': '/World/Wall'}
    assert caster(FakeQuery(hit)).range([0, 0, 0], [1, 0, 0], .06, 30) == expected


def offline_sensor():
    sensor = LidarSensor.__new__(LidarSensor)
    sensor.config = load_lidar_config(CONFIG)
    sensor.meters_per_unit = 1.
    sensor.mount_offset = np.array([.17, 0., .06794])  # base_link + bag laser mounting
    sensor.mount_rotation = quaternion_matrix(rpy_quaternion([0., 0., 0.]))
    angles = np.radians(np.linspace(-135, 135, 1081))
    sensor.directions = np.column_stack([np.cos(angles), np.sin(angles), np.zeros(1081)])
    sensor.chassis_directions = sensor.directions@sensor.mount_rotation.T
    sensor.chassis_offset = sensor.mount_offset/sensor.meters_per_unit
    sensor.samples = []; sensor.scans = []

    def measure(origin, direction, minimum, maximum):
        sensor.samples.append((origin.copy(), direction.copy()))
        # Vertical wall at world x=10. ROS +angle points left (+y).
        return (10-origin[0])/direction[0] if direction[0] > 0 else math.inf

    sensor.caster = SimpleNamespace(range=measure)
    sensor.publish_scan = lambda stamp, ranges: sensor.scans.append((stamp, ranges.copy()))
    sensor.reset()
    return sensor


def test_rolling_scan_rate_stamp_motion_and_ros_axes():
    sensor = offline_sensor()
    for t in np.arange(0., .101, .001):
        sensor.update(t, [t, 0, .07, 0, 0, 0, 1])
    assert len(sensor.scans) == 4
    assert [s[0] for s in sensor.scans] == pytest.approx([0., .025, .05, .075])
    assert len(sensor.scans[0][1]) == 1081
    assert sensor.samples[0][0] == pytest.approx([.17, 0., .13794])
    assert sensor.samples[1080][0][0] == pytest.approx(.17+.01875)
    assert sensor.samples[0][1][1] < 0  # -135 degrees is to the right
    assert sensor.samples[1080][1][1] > 0
    # Mid-scan forward ray at t=9.375ms measures geometry from its own pose.
    assert sensor.scans[0][1][540] == pytest.approx(10-.17-.009375)


def test_mount_follows_chassis_rotation_and_reset_clock():
    sensor = offline_sensor()
    q = rpy_quaternion([0, 0, 90])
    sensor.update(1., [2, 3, .07, *q])
    assert sensor.samples[0][0] == pytest.approx([2., 3.17, .13794])
    sensor.update(0., [0, 0, .07, 0, 0, 0, 1])
    assert sensor.scan_start == 0.
    assert sensor.sample_index == 1


def test_timestamp_nanosecond_carry():
    stamp = SimpleNamespace()
    stamp_time(stamp, 1.9999999999)
    assert (stamp.sec, stamp.nanosec) == (2, 0)


def test_batched_geometry_matches_independent_scalar_transforms():
    # All three orientation axes, a rotated mount, and antipodal quaternion
    # representations catch rotations/sign mistakes hidden by straight runs.
    previous = np.array([1., -2., .5, *rpy_quaternion([20, -15, 175])])
    current = np.array([1.02, -1.98, .51, *-rpy_quaternion([23, -12, -178])])
    mount_rotation = quaternion_matrix(rpy_quaternion([5, 8, -10]))
    fractions = np.linspace(0., 1., 59)
    angles = np.linspace(-2.2, 2.1, len(fractions))
    directions = np.column_stack([np.cos(angles), np.sin(angles), np.zeros_like(angles)])@mount_rotation.T
    offset = np.array([.17, 0., .06794])
    origins, transformed = beam_geometry(previous, current, fractions, offset, directions)
    for index, fraction in enumerate(fractions):
        interpolated = interpolate_pose(previous, current, fraction)
        rotation = quaternion_matrix(interpolated[3:])
        np.testing.assert_allclose(origins[index], interpolated[:3]+rotation@offset, atol=1e-14)
        np.testing.assert_allclose(transformed[index], rotation@directions[index], atol=1e-14)


def test_long_clock_beam_boundary_and_discontinuity():
    sensor = offline_sensor()
    pose = [0, 0, .07, 0, 0, 0, 1]
    start = 3600.
    step = sensor.config['time_increment_s']
    sensor.update(start, pose)
    # An exact endpoint and one immediately either side, using the same
    # 1e-10s tolerance as the sensor's original scalar scheduling.
    for elapsed in [step-2e-10, step, 2*step+2e-10, .01, .019, .025]:
        now = start+elapsed
        sensor.update(now, pose)
        all_times = [start+period+index*step for period in (0., .025)
                     for index in range(1081)]
        assert len(sensor.samples) == sum(t <= now+1e-10 for t in all_times)
    assert len(sensor.scans) == 1
    sensor.update(start+1., pose)  # Gap > one scan: no historical scans.
    assert sensor.scan_start == start+1.
    assert sensor.sample_index == 1
    assert len(sensor.scans) == 1


class StubFrameTree:
    base_frame = 'base_link'

    def __init__(self, translation, quaternion):
        self.translation = translation; self.quaternion = quaternion

    def resolve(self, frame_id):
        if frame_id != 'laser':
            raise ValueError('Unknown frame')
        return self.translation, self.quaternion


def test_reload_uses_central_frame_tree_and_never_publishes_tf(monkeypatch):
    # No ROS installation is needed to verify publisher ownership and config
    # reload. A frame tree may resolve multiple mount links before this API.
    qos = SimpleNamespace(QoSProfile=lambda **kwargs: kwargs,
                          ReliabilityPolicy=SimpleNamespace(BEST_EFFORT='best_effort'))
    monkeypatch.setitem(sys.modules, 'rclpy', SimpleNamespace(qos=qos))
    monkeypatch.setitem(sys.modules, 'rclpy.qos', qos)
    created = []; destroyed = []

    def create_publisher(message_type, topic, options):
        publisher = SimpleNamespace(topic=topic, options=options)
        created.append(publisher)
        return publisher

    sensor = LidarSensor.__new__(LidarSensor)
    sensor.path = CONFIG; sensor.publisher = None; sensor.message_type = object
    sensor.base_link_offset_m = np.array([-.1, 0, -.04206])
    sensor.vehicle_root = '/World/RC_Car'; sensor.meters_per_unit = 1.
    sensor.query = FakeQuery(); sensor.vector_type = lambda *xyz: xyz
    sensor.node = SimpleNamespace(create_publisher=create_publisher,
                                  destroy_publisher=destroyed.append,
                                  get_logger=lambda: SimpleNamespace(warning=lambda message: None))
    sensor.frame_tree = StubFrameTree([.27, 0, .11], [0, 0, 0, 1])
    sensor.reload()
    assert sensor.mount_offset == pytest.approx([.17, 0, .06794])
    assert sensor.mount_rotation == pytest.approx(np.eye(3))
    assert sensor.scan_start is None
    assert not hasattr(sensor, 'tf_publisher')
    next_tree = StubFrameTree([.3, .02, .15], rpy_quaternion([0, 0, 90]))
    sensor.scan_start = 12.; sensor.sample_index = 5
    sensor.reload(frame_tree=next_tree)
    assert sensor.frame_tree is next_tree
    assert sensor.mount_offset == pytest.approx([.2, .02, .10794])
    assert sensor.chassis_directions[540] == pytest.approx([0, 1, 0], abs=1e-14)
    assert sensor.scan_start is None and sensor.sample_index == 0
    assert [publisher.topic for publisher in created] == ['/scan', '/scan']
    assert destroyed == [created[0]]
    sensor.close()
    assert destroyed == created
    sensor.close()  # Safe during a second adapter cleanup.
    assert destroyed == created


class CoreFakeQuery:
    def __init__(self, hit=None, hits=()):
        self.hit = hit; self.hits = hits; self.queries = []

    def raycast_closest(self, origin, direction, maximum, both_sides):
        self.queries.append((origin, direction, maximum, both_sides))
        return self.hit is not None, SimpleNamespace(**self.hit) if self.hit else None

    def raycast_all(self, origin, direction, maximum, callback, both_sides):
        for hit in self.hits:
            assert callback(SimpleNamespace(**hit))


def test_core_integer_id_self_filter_nearest_external_and_units():
    # Collision and body IDs are independently filtered. All-hit order is
    # deliberately reversed; stage units are cm to exercise conversion.
    query = CoreFakeQuery({'distance': 10., 'collision': 99, 'rigid_body': 101}, [
        {'distance': 800., 'collision': 8, 'rigid_body': 0},
        {'distance': 5., 'collision': 100, 'rigid_body': 0},
        {'distance': 250., 'collision': 2, 'rigid_body': 0},
    ])
    sensor_caster = PhysicsCoreRaycaster(query, lambda *xyz: xyz, .01, {100, 101})
    assert sensor_caster.range([1, 2, 3], [1, 0, 0], .06, 30) == 2.5
    assert query.queries == [((1., 2., 3.), (1., 0., 0.), 3000., True)]


@pytest.mark.parametrize('distance,expected', [(None, math.inf), (.03, -math.inf), (.06, .06), (30., 30.), (31., math.inf)])
def test_core_scalar_and_batch_match_legacy_range_rules(distance, expected):
    query = CoreFakeQuery(None if distance is None else
                          {'distance': distance, 'collision': 1, 'rigid_body': 0})
    sensor_caster = PhysicsCoreRaycaster(query, lambda *xyz: xyz, 1., {100})
    origins = np.array([[1.23, -.9, .5], [-1., 2., .6]])
    directions = np.array([[1., 0., 0.], [0., -1., 0.]])
    scalar = [sensor_caster.range(o, d, .06, 30) for o, d in zip(origins, directions)]
    scalar_queries = query.queries.copy(); query.queries.clear()
    batch = sensor_caster.ranges(origins, directions, .06, 30)
    assert scalar == batch == [expected, expected]
    assert scalar_queries == query.queries


def test_rolling_batch_query_keeps_beam_pose_and_publication(monkeypatch):
    sensor = offline_sensor()
    scalar_measure = sensor.caster.range
    batches = []

    def measure_batch(origins, directions, minimum, maximum):
        batches.append(len(origins))
        return [scalar_measure(o, d, minimum, maximum) for o, d in zip(origins, directions)]

    sensor.caster.ranges = measure_batch
    for t in np.arange(0., .026, .001):
        sensor.update(t, [t, 0, .07, 0, 0, 0, 1])
    assert len(sensor.scans) == 1
    assert sensor.scans[0][0] == 0.
    assert len(sensor.scans[0][1]) == 1081
    assert sensor.scans[0][1][540] == pytest.approx(10-.17-.009375)
    assert sum(batches) == 1082  # One new first beam at t=25ms.


def test_core_unavailable_uses_legacy_fallback_with_warning(monkeypatch):
    monkeypatch.setitem(sys.modules, 'omni.physics.core', None)
    warnings = []
    sensor = LidarSensor.__new__(LidarSensor)
    sensor.node = SimpleNamespace(get_logger=lambda: SimpleNamespace(warning=warnings.append))
    sensor.query = FakeQuery(); sensor.vector_type = lambda *xyz: xyz
    sensor.vehicle_root = '/World/RC_Car'; sensor.meters_per_unit = 1.
    config = load_lidar_config(CONFIG); config['backend'] = 'physics_core'
    result = sensor.make_raycaster(config)
    assert result.backend == 'physx'
    assert result.range([0, 0, 0], [1, 0, 0], .06, 30) == math.inf
    assert len(warnings) == 1
    assert 'using physx' in warnings[0]
