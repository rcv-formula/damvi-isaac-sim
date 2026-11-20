# Hokuyo UST-10LX simulator

The requested “UST-LX10” is interpreted as **Hokuyo UST-10LX**. Official
[manufacturer specifications](https://www.hokuyo-aut.jp/search/single.php?serial=167)
and [Hokuyo USA specification sheet](https://hokuyo-usa.com/application/files/6716/8451/4574/Hokuyo_-_UST-10LX_-_Spec_Sheet_-_Final.pdf)
specify 270° visible arc, 0.25° increment, 25 ms rotation, 0.06 m minimum,
10 m guaranteed detection on white targets, 4 m at 10% reflectance, and a
30 m maximum detection range. These are different range claims.

## Recorded reference

Read from `/home/shin/Desktop/red-damvi dataset/0821/test_2_0.mcap`:

| /scan property | First recorded message |
|---|---|
| frame_id | laser |
| angle_min / angle_max | −2.356194496 / +2.356194496 rad |
| angle_increment | 0.004363323096 rad |
| ranges count | 1081 |
| scan_time | 0.02500000037 s |
| time_increment | 0.00001736111517 s |
| range_min / range_max | 0.02 / 30 m |
| intensities | empty |

The YAML follows the **physical minimum 0.06 m**, while keeping recorded
maximum 30 m. Setting `range_min_m: 0.02` reproduces the driver's advertised
metadata but does not represent the manufacturer's guaranteed minimum.

The same bag contains two conflicting `base_link -> laser` static transforms:
`[0.27, 0, 0.11]` then `[0.27, 0, 0]`, both with identity rotation. These are
recorded reference evidence, not a second source of runtime extrinsics.
Per the user's selected source, the **`red_damvi` branch overrides these bag
heights**: its
[`tf_publisher.py`](https://github.com/rcv-formula/f1_stack_for_damvi/blob/red_damvi/f1tenth_stack/f1tenth_stack/tf_publisher.py)
sets `base_link -> laser` to `[0.27, 0, 0.16]` m and `base_link -> imu` to
`[0.27, 0, 0.06]` m, both with identity rotation. The source file blob SHA is
`ea4ed59309d4d575aba8bf4f62b55674eaaf3c59`.
The single runtime source for mounting is **`config/tf.yaml`**, shared by all
sensors. `lidar.yaml` only names its `frame_id`; the central `FrameTree`
resolves that frame into the rear-axle-centre base frame, including chained
translations and rotations. The lidar adds the existing chassis-relative
base_link offset to this resolved translation. With the selected source,
base_link offset `[-0.1, 0, -0.04206]` plus laser translation `[0.27, 0, 0.16]`
gives chassis-relative `[0.17, 0, 0.11794]` m. For comparison, base_link offset
`[-0.1, 0, -0.04206]` plus an explicitly configured laser translation
`[0.27, 0, 0.11]` gives chassis-relative `[0.17, 0, 0.06794]` m. The example
does not override the transforms selected in `tf.yaml`.

## Implementation and limits

`lidar.py` uses the installed
[`omni.physx` scene query API](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/108.1/extensions/runtime/source/omni.physx/docs/dev_guide/scene_queries.html)
against the **currently running GUI's collision geometry**, after physics
steps. No separate Isaac instance or custom UI is created. Installed Isaac
6.2 `omni.physx` 110.4.4 source and Python binding declarations were checked.

This is a geometric range sensor, not an RTX optical sensor: rendered CAD
details without colliders are invisible, collider approximations affect
ranges, and material reflectivity, glass transmission, sunlight, multipath,
beam divergence and a calibrated measurement-noise model are not simulated.
Existing map colliders are measured directly. No intensities are fabricated.
The 30 m geometric cutoff consequently does not reproduce target-dependent
real detection probability. Self hits under the configured vehicle root are
excluded; if the closest hit is self geometry, unordered all-hit results are
searched for the nearest external collider.

`lidar.yaml` selects `backend: physics_core` by default after live validation;
`backend: physx` retains the original implementation. Both query the same live
physics collision scene with identical ray geometry, range limits and
nearest-external-hit rules. The newer `omni.physics.core` API returns a
`(bool, RaycastHit)` pair with integer collider/body path IDs instead of a
dictionary of strings. On Play, all vehicle descendants, including instance
proxies, are encoded with `PhysicsSchemaTools.sdfPathToInt` for self filtering.
The core backend also converts NumPy vectors to Python floats in bulk before
constructing the same `carb.Float3` arguments. It retains per-ray PhysX
queries; it is not an alternate scan model. If the newer API cannot be
imported or is unavailable, reload logs a warning and uses the original
backend. Status records requested and active backends. The live-scene check
used 1087 identical rays, including six from inside the vehicle, and found
zero range differences. Median query time was 12.06 ms versus 5.38 ms.

Each rotation produces 1081 beams in ROS order (−135° to +135°,
counterclockwise viewed from +Z, +X forward, +Y left). The header timestamp
is simulation time of the **first beam**, not publication time. Beam period
is `0.025 / 1440` s, giving 18.75 ms from first to last visible sample and a
6.25 ms blind sector. Each physics callback acquires the due beams; adjacent
rigid-body tensor poses interpolate the moving sensor's position and
orientation. Collision geometry for other moving objects remains at the
current physics tick, so reducing physics frequency reduces their temporal
accuracy. At the current 200 Hz physics this quantization is at most 5 ms.
After a clock reversal or a forward discontinuity over one scan period,
the scanner resets rather than inventing past measurements. No return is
`+inf`; a detected obstruction below minimum range is `-inf` (REP 117).
Scan QoS is best-effort, volatile. This sensor never publishes `/tf_static`:
the adapter's common frame-tree publisher owns the complete static tree,
avoiding multiple independent latched sensor writers. Its lifecycle and QoS
are documented with the central TF implementation.

## Adapter API

```python
from lidar import LidarSensor
self.lidar = LidarSensor(
    self.stage, self.bridge.node, HERE/'config/lidar.yaml',
    chassis_prim=self.config['scene']['chassis'],
    vehicle_root=self.config['scene']['vehicle_root'],
    base_link_offset_m=self.config['scene']['rear_axle_offset_m'],
    frame_tree=self.frame_tree)

# After each physics step, using the same body's live physics tensor:
self.lidar.update(self.elapsed, self.body.get_transforms()[0])
# Pose layout: [x, y, z, qx, qy, qz, qw], positions in stage units.

# On Play: adapter first reloads config/tf.yaml and the common TF publisher;
# then re-read scan YAML, resolve the new extrinsics and reset the scan:
self.lidar.reload(frame_tree=self.frame_tree)
# reload() without arguments reuses the previously supplied FrameTree.
# On timeline Stop:
self.lidar.reset()
# Before destroying/replacing the ROS node, or closing the adapter:
self.lidar.close()
# Optional live status:
report['lidar'] = self.lidar.status()
```

The parent adapter owns GUI lifecycle and sensor integration; this module
does not change physics, stage time, or vehicle control.
