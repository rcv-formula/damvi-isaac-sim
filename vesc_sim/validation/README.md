# Recorded validation

These are simulator checks, not a claim of complete real-car calibration.

- `physics_forward_reverse.*`: early flat-scene motor forward/reverse/stop test.
- `physics_wheelspin.*`: constrained chassis with rotating wheels; ERPM remains nonzero.
- `gui_ros_check.json`: ROS VESC command and feedback in the existing GUI.
- `gui_steering_scan_check.json`: stationary physical front-wheel sweep, max error 0.143°, including physical command saturation, 371 LaserScan messages with 1081 beams at 40 Hz simulation time.
- `gui_turning_check.json`: short forward/reverse turn with correct yaw directions, about 0.74 m/s chassis speed, steering error below 0.133°, 280 scans; IMU gyro agrees with physical yaw in both directions. After 3–3.45 seconds of electric/passive braking, median residual is -0.087 ERPM and 0.001015 m/s.
- `gui_turning_initial_short_settle.json`: initial check failed its stop threshold after only 1–1.45 seconds of braking (median -32.3 ERPM, 0.0222 m/s). The check was extended to measure settling after three seconds. No controller or friction parameter was changed to pass it; a one-second stopping requirement has not been demonstrated.
- `gui_sensors_check.json`: common user-selected TF, 2976 IMU samples (median period 1 ms), mean stationary acceleration `[0.000822, 0.00000189, 9.809989]` m/s², and 119 LiDAR scans. IMU noise/filter defaults were off.
- `gui_reload_check.json`: existing Play/Stop reloads controller, sensor YAML, and shared TF with no custom UI.

The reports above retain the earlier 1000 Hz physics checks. Their sensor
rates refer to simulation timestamps; the old three-second sensor check
took about 9.8 seconds of wall time.

Current 200 Hz physics validation is recorded in `gui_sensors_check_200hz.json`,
`gui_turning_check_200hz_realtime.json`, and
`gui_steering_scan_check_200hz_realtime.json`. IMU output remains 1 kHz by
interpolation, with unchanged common TF. The final ten-second measurement
in `performance/realtime_check.json` gives an average real-time factor of
1.00038. See `performance/README.md` for the measured bottlenecks, fixes,
limitations, and GUI recovery during an unsuccessful scheduler experiment.
