# IMU model

`imu.py` publishes `sensor_msgs/Imu` on `/imu/data` in frame `imu`.
`config/imu.yaml` defaults to 1000 Hz output with
`sampling_mode: interpolated_physics_state`, as requested. Physics can run
at 200 Hz; intermediate outputs are reconstructed from completed physics
intervals and are not independent 1 kHz physical measurements. The recorded
`0821/test_2_0.mcap` instead contains approximately 100 Hz on `/imu/data`, with
frame `imu` and 0.01 on all three covariance diagonals. Its
`/sensors/imu/raw` is approximately 50 Hz but has empty frame IDs and zero
measurements, so it is not used as a physical sensor reference. No bag is
modified or replayed by this implementation.

## Frame and acceleration conventions

Input pose is PhysX tensor XYZ + XYZW, mapping chassis axes into the world.
Input velocity is world-frame linear COM velocity followed by world-frame
angular velocity. The installed Isaac 6.2 tensor API explicitly documents the
linear component as the velocity of the center of mass:
`extscache/omni.physics.tensors-110.4.4+110.4.0.lx64.r.cp312.u7f4/omni/physics/tensors/api.py`,
`RigidBodyView.get_velocities()` (line 3319 in this installation).
`get_coms()[0,:3]` supplies the body-local COM translation; integration must
call `set_center_of_mass_offset_m()` after creating the tensor view.
Linear velocity is multiplied by the USD stage's `metersPerUnit`; offsets
passed to the constructor or setter are already in meters.

The common `FrameTree.resolve(frame_id)` returns base-to-sensor translation
and XYZW rotation. Sensor mounting numbers are **only** in `config/tf.yaml`;
the IMU configuration has only a frame ID. With chassis-to-base translation
`b`, base-to-IMU translation `s`, and chassis-to-COM translation `c`, the
world lever arm is `r = R_world_chassis * (b + s - c)`. Thus the sensor's
world acceleration is:

```
a_sensor = a_COM + alpha × r + omega × (omega × r)
specific_force_sensor = R_world_sensor.T * (a_sensor - gravity_world)
gyro_sensor = R_world_sensor.T * omega
q_world_sensor = q_world_chassis * q_chassis_sensor
```

Both tangential and centripetal accelerations are included. An upright,
stationary sensor reports approximately `[0, 0, +9.81] m/s²`; a freely falling
sensor reports approximately zero. The reported orientation is physical
world orientation, with no estimator drift, magnetometer simulation, or
automatic map/localization alignment. Orientation error, when enabled, is
a right-multiplied sensor-local rotation vector, not an Euler-angle sum.

`gravity_world_m_s2` must match the physics scene. The current scene uses
`[0, 0, -9.81] m/s²`. The ROS message units and covariance conventions were
checked against the installed official definition:
`/opt/ros/humble/share/sensor_msgs/msg/Imu.msg`. Gyro is rad/s, acceleration
is m/s², and quaternion fields are XYZW. All-zero covariance means
**unknown** in ROS, not a claim of measured zero uncertainty. Setting
`orientation.enabled: false` publishes covariance element 0 as -1.

## Sampling and reset behavior

Call `update(sim_time, chassis_pose_xyzw, velocity_world6)` exactly once after
each physics step. Two sampling modes are available in the sensor YAML:

- `interpolated_physics_state` is the default. After a 200 Hz physics step
  completes, it emits the five 1 ms sample timestamps within that completed
  5 ms interval. Translation and COM/angular velocity use linear
  interpolation; orientation uses shortest-path normalized quaternion lerp.
  COM acceleration and angular acceleration are the velocity slopes between
  the two actual physics states, optionally filtered at the physical step
  rate. Each sample then uses its own interpolated rotation and angular
  velocity for sensor-axis projection and tangential/centripetal terms.
- `physics_state` uses only actual provided physics states and rejects an
  output rate above `physics_hz`. A phase-locked sample index avoids drift at
  noninteger ratios, but output timestamps are quantized to the actual
  physics ticks. For example, 333 Hz with 1000 Hz physics alternates 3 ms and
  4 ms intervals.

Interpolation never extrapolates into a future physics state. It adds
within-step output latency: at 200 Hz physics, the five 1 kHz readings are
published together after their interval finishes, with timestamp ages from
0 to 4 ms. They are not five independent solver updates or five independent
measurements of high-frequency contact dynamics. Their uniform timestamp
grid also works at noninteger rate ratios, such as 333 Hz output with 200 Hz
physics. `status()` reports the mode, physics/output rates, cumulative count
of published interior interpolated samples, and the last batch's publication
count.

The first available physics state after startup/reset/clock discontinuity
produces just one current-time sample; no unknown earlier interval is filled.
Its first derivative is zero rather
than an invented acceleration from an assumed initial velocity. Constant
angular velocity still contributes the known centripetal term immediately.
Repeated timestamps produce no duplicate samples. A backward jump or a gap
larger than `clock_jump_reset_s` restarts derivative history and the sampling
phase, without bursting historical measurements or interpolating across the
gap. `reset()` is intended for
Stop; `reload()` rereads both YAML and the supplied FrameTree at Play.
No TF is published here; the project's common TF publisher owns that tree.

## Optional imperfections

Defaults have zero bias, zero sample noise, no dropout, and no derivative
filter. This is an ideal physical-state sensor, **not a calibrated model of
the real IMU**. YAML can change world derivative low-pass frequency, fixed
sensor-axis biases, independent Gaussian noise per published sample,
Gaussian bias random walks per square-root second, covariance matrices,
dropout probability, and seed. Bias random walks use elapsed output-sample
simulation time, including intentionally dropped samples. Thus 1 kHz
interpolated samples have the same seeded sample-noise/random-walk/dropout
sequence as 1 kHz actual samples on the same timestamp grid. Reload/reset restores the
initial bias and seed; cumulative publication/drop/reset counters remain
available through `status()`.

The implementation differentiates solver COM/angular velocities; contact
impulses and coarse solver dynamics can produce sharp accelerometer peaks.
The optional low-pass filters derivatives in world coordinates before
lever-arm projection. It does not reproduce the real IMU's internal fusion,
analog bandwidth, clipping, thermal effects, or measured noise spectrum.

## Integration API

```
sensor = ImuSensor(stage, ros_node, config_path,
                   base_link_offset_m=rear_axle_offset_m,
                   frame_tree=frame_tree, physics_hz=physics_hz)
sensor.set_center_of_mass_offset_m(body.get_coms()[0, :3] * meters_per_unit)
sensor.update(sim_time, body.get_transforms()[0], body.get_velocities()[0])
sensor.reload(frame_tree=frame_tree, physics_hz=physics_hz)
sensor.reset()
sensor.status()
sensor.close()
```

Unit tests cover stationary gravity/warmup, independent body and mount
rotations, COM-relative tangential and centripetal acceleration, derivative
filtering, noninteger scheduling, long clock/reset/gaps, rate rejection,
repeatable noise/bias random walks, dropout, covariance validation, timestamp
carry, and publisher reload without any TF publisher. Randomized checks
also exercise 1000 quaternion compositions and 1000 moving physical states
against independent rotation-matrix/vector references. Interpolation tests
cover exact 200-to-1000 Hz timestamps and five-sample batches without future
output, noninteger rate ratios, rotating mounts with COM lever arms, known
acceleration/free fall, clock gaps, and equivalent nonzero-noise/bias-walk
and dropout sequences on the output-sample clock.

## Small-array computation cost

The three-component cross product and four-component Hamilton product use
fixed-size scalar formulas. NumPy's general axis/broadcast dispatch cost is
large compared with these tiny calculations. Reload/reset caches which
noise, bias, and bias-random-walk groups are nonzero; the default ideal path
skips their allocations, random-number checks, and identity orientation-error
composition. Active noise groups keep the original random-number sequence.

A local five-trial benchmark before the interpolation feature, using 1000
updates with a simple fake ROS publisher,
measured median 0.15435 s before and 0.03352 s after these changes (4.60x).
This excludes real ROS transport, physics tensor reads, rendering, and the
rest of the GUI, so it is not a simulator real-time performance claim.
An independent before/after comparison used 1500 randomized pose/velocity
states for each of three configurations: ideal 1000 Hz; nonzero orientation
bias/noise/random walk plus gyro/acceleration noise and derivative filtering;
and mixed zero/nonzero noise with 333 Hz scheduling and dropout. Gyro and
acceleration differences were zero, quaternion component differences were
at most 3.4e-16, and publication/drop counts were identical.

The Python `Imu` object and its frame/covariance metadata are also reused
between samples and rebuilt at each reload. Each sample changes only its
timestamp and measured values. This is safe for the single physics callback:
the installed Isaac Jazzy `rclpy/publisher.py` calls the C++ publish method
synchronously, and the official Jazzy sources allocate and convert a
separate C message for each call before publishing. See
[Publisher::publish](https://github.com/ros2/rclpy/blob/jazzy/rclpy/src/rclpy/publisher.cpp#L114),
[convert_from_py](https://github.com/ros2/rclpy/blob/jazzy/rclpy/src/rclpy/utils.cpp#L65),
and the [generated field conversion template](https://github.com/ros2/rosidl_python/blob/jazzy/rosidl_generator_py/resource/_msg_support.c.em).
The Python message is mutated only after `publish()` returns; no concurrent
writer is introduced. This avoids Python object construction and constant
metadata setters, but does not remove the C conversion/serialization cost.

A separate benchmark on the server using its actual Jazzy generated `Imu`
class measured 10,000 constructor/metadata/value updates at median 0.11844 s,
versus 0.01515 s when reusing the Python object. No ROS node or GUI operation
was involved. Serializing the reused object before and after mutation also
verified that the first serialized measurement remained independent. Test
publishers deep-copy messages to reflect the real rclpy copy boundary, and
tests check prior-sample independence and metadata changes across reloads.
