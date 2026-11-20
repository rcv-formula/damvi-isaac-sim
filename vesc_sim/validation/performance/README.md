# GUI performance investigation

The final configuration is physics **200 Hz**, viewport target **30 Hz**,
IMU **1000 Hz interpolated output**, and LiDAR **40 Hz / 1081 beams**.
No extra headless simulator is running. Process inspection found one car
GUI and one unrelated cabinet-project GUI. The latter was left running.

## Measurements

| Configuration / experiment | Real-time factor |
|---|---:|
| Initial 1000 Hz physics, all sensors | 0.301 |
| Initial configuration with LiDAR skipped | 0.370 |
| Initial configuration with IMU skipped | 0.360 |
| Initial configuration with both sensors skipped | 0.552 |
| Initial configuration with sensors and repeated trace serialization skipped | 0.599 |
| Optimized sensors, 200 Hz physics, viewport on at initial 60 Hz target | 0.865–0.876 |
| Same setup with viewport updates temporarily off | 1.275 |
| 30 Hz target but six physics steps capped per frame | 0.847 |
| **Final wall-clock loop, 200 Hz physics and 30 Hz viewport target** | **1.00038** |

`profile.json`, `viewport_trials.json`, and
`render30_with_six_step_limit.json` preserve the relevant earlier checks.
The final `realtime_check.json` measured **10.00117 wall seconds** and
**10.00500 simulation seconds**, with **10005 IMU messages** and **400 scans**.
Actual viewport throughput was about 27.90 frames/s; physics catches up in
5 ms steps instead of slowing simulation time whenever a GUI frame is late.
One-second measurement windows varied from approximately 0.94 to 1.06 as
the loop caught up. This is average real-time playback, not a hard real-time
latency guarantee or a guarantee for heavier scenes.

## Changes supported by the measurements

- Physics was reduced to the user's 200 Hz upper limit. The 5000 Hz speed
  controller runs 25 internal substeps with each physical feedback sample.
- IMU small-array operations and zero-noise branches were simplified; ROS
  message metadata is reused. Its 1 kHz output interpolates completed 200 Hz
  physics intervals, with within-step batch latency up to about 4 ms.
- LiDAR uses integer-ID `omni.physics.core` queries. The legacy and new paths
  produced identical ranges for 1087 rays, including self-filter cases.
  Median query time was 12.06 ms versus 5.38 ms; rolling scan timing remains.
- Whole-history trace JSON serialization was removed from the recurring GUI
  update. Samples remain in memory and validation explicitly saves a snapshot.
- Small-scene CPU broadphase and main-thread PhysX execution avoid unnecessary
  GPU dispatch and worker scheduling. Thread count alone had only a small gain.
- The display target is independent of physics. `minFrameRate=30` capped
  200 Hz physics to six 5 ms steps per frame; 30 Hz display requires six or
  seven. `min_frame_rate: 20` permits that extra step.
- Isaac's automatic manual-loop-on-Play preference is disabled for real-time
  mode. The loop uses actual elapsed time while keeping physical steps at
  200 Hz. Pending ROS commands are drained so a 50 Hz input does not build
  latency behind a 30 Hz GUI callback.

Viewport resolution reductions did not give a consistent improvement, so
the original 1600×1000 render texture was retained. Fabric and a new native
raycast sensor were not needed for the final configuration.

## Recovery during investigation

An experimental switch to PhysX's internal dispatcher with one worker
stopped the original car GUI responding. Its process was replaced with a
GUI loading the same original car/map asset, then the project's session
changes and adapters were reconstructed. The original USD file was not
saved or replaced. The final configuration uses the Carbonite dispatcher;
the unsafe experiment scripts were removed. Saved partial trial results
and `gui_recovery_note.txt` retain this incident rather than treating the
trial as successful.

The final 200 Hz configuration passed forward/reverse turning, stop, IMU
gyro consistency, LaserScan timing, physical steering saturation, and
Play/Stop config reload checks. See the adjacent 200 Hz validation reports.
