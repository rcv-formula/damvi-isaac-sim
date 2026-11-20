# Dataset-conditioned tire and steering calibration — 2026-11-19

The tuned defaults are applied to the SSH simulator. One headless process is
running the 0821 map, physics 200 Hz, target speed zero. No original USD asset
was saved. VESC XML gains/current limits, ERPM gain 1538 and wheel-derived
telemetry are retained.

## Applied model

- `config/runtime.yaml`: spinning-wheel angular limit 1000 rad/s, converted to
  USD degrees/s. The previous schema default was 5729.58 degrees/s = 100 rad/s,
  only 2.794 m/s at the 0.02794 m radius. A solver cap was corrupting high-speed
  rolling behavior. This change permits rotation; it does not prescribe it.
- Analytic cylinder **normal contacts**, with native tangential tire friction
  disabled on a private material. Convex 64/127-sided alternatives were tested:
  their rotating facets caused contact/load intermittency at high speed.
  The selected analytic/contact-force combination passed the new replay and
  30-second circle checks; this differs from the earlier analytic+Coulomb mode.
- C99 combined-slip tire forces, tanh lateral saturation, a friction ellipse,
  implicit force response and a coupled contact-mobility bound. Forces are
  applied once to physical wheels at contact points. Rolling torque follows
  from this application; it is not added twice. Normal loads are measured from
  PhysX, with geometry checks rejecting stale airborne loads. No runtime body
  pose/velocity or wheel-speed commands are used to enforce a trajectory.
- Shared effective per-wheel cornering stiffness: front 51.55015, rear
  387.10305 N/rad at reference loads 6.867/10.3005 N. These are model coefficients
  transferred and checked in the articulated simulator, not uniquely measured
  rubber properties. Rear stiffness was poorly bounded in the reduced fit.
- The observed servo saturation and static mapping are unchanged. Actuator
  time constant changes from 0.07 to 0.02885965 s. No additional command delay
  is applied: a separate 0.05 s delay worsened actual PhysX replay metrics.
  The reduced-model delay includes effects that should not be counted twice.
- `sim_support.py` now supports the exact monotone-PCHIP inverse for 2–64 knots.
  It retains the current two-knot behavior. Static map gain alone cancels through
  this inverse in the full control path and was not used to hide oscillation.

## One constant surface coefficient per dataset

Edit `../../config/surface_friction.yaml`:

```yaml
selected_dataset: '0821'
datasets:
  '0821':
    coefficient: 0.5377581441296462
```

The selected scalar is fixed across all positions, times and speeds during a
run, and is applied equally to all four tires. Actual force still varies with
normal load and slip. Common tire direction response stays in `tires.yaml`:
the surface scalar scales both longitudinal and lateral peak coefficients by
`coefficient / reference_coefficient`. At the reference coefficient 0.5231599,
the shared baseline longitudinal/lateral peaks are 0.8/0.5231599. They are not
independently varied by dataset. To choose equal peak coefficients in both
directions, set both baseline tire coefficients to the surface reference.

| Dataset | Constant coefficient | Qualification |
| --- | ---: | --- |
| 0821 | 0.537758 | Weak evidence: 27.10 s, 3 independent time blocks |
| 0821-1 | 0.527108 | 366.30 s, 22 blocks |
| ifac | 0.563850 | 110.92 s, 8 blocks |
| ifac1 | 0.634100 | 372.18 s, 26 blocks |
| ifac2 | 0.435539 | 51.36 s, 11 blocks |

0521/0814/0822 remain `null` and explicitly use the default coefficient, marked
uncalibrated. All values are effective response estimates, not direct surface
measurements. The current profile is 0821. Files are read on startup and Play.
For a running headless session, pause/play reloads the coefficients; changing
the normal-contact/tire-force mode requires adapter reinstall while stopped.

Live reload was verified by switching 0821 → ifac2 → 0821: all four native
lateral peak coefficients changed together and the original YAML bytes/hash
were restored. See `surface_reload_check.json`.

## Validation with actual PhysX

Five input windows were reserved before plant parameter search. Actual recorded
`/commands/motor/speed` ERPM and executed post-driver steering were replayed;
the bag's legacy `/drive.speed` conversion was not mistaken for physical m/s.
Only initial state was prescribed, then forces/torques evolved the vehicle.

The comparison below uses the native-contact baseline with the angular cap
corrected and analytic geometry, versus the tuned tire model and constant
dataset surface profiles. Thus it separates much of the tire-response change
from the independently necessary high-speed solver-cap correction.

| Reserved window | Baseline yaw RMSE | Tuned yaw RMSE | Body-speed comparison |
| --- | ---: | ---: | --- |
| Low / ifac2 | 0.291 | 0.084 rad/s | Valid |
| Medium / 0821 | 0.570 | 0.139 rad/s | Valid |
| High / 0821 | 0.381 | 0.133 rad/s | Valid |
| Very high / ifac1 | 0.266 | 0.212 rad/s | Valid, 90% speed agreement |
| Low / ifac1 | 0.278 | 0.201 rad/s | Speed mismatch: only 57% agreement |

Weighted RMSE over the four speed-comparable windows is **0.408 → 0.143 rad/s**
(65% reduction). All five completed without the sampled penetration/rollover
guard failing. The fifth window is excluded from that aggregate, rather than
treating slower/faster motion as better tire identification.

The real recordings have different current-control behavior. In particular,
the ifac1 tuning window has a roughly 25 A plateau, while other recordings
exceed 36/41 A and show different integral/history effects. The user-selected
62 A, Ki=0 XML profile remains unchanged. A single surface coefficient cannot
legitimately remove all such longitudinal differences.

The windows were held out from **actual plant parameter search**. The shared
reduced-model prior used the broader recordings, so this is not completely
new data collection or an independent full-recording holdout. Separate whole-
recording cross-validation of the reduced model is in the identification report.

## Closed-loop stability checks

Thirty-second PP-like circle recovery checks at 0.8, 3, 6 and 8 m/s all passed
at 200 Hz. These use simulated measured pose as feedback to isolate the plant;
they are not a new full ROS/PF-stack verification. Circle radius is 20 m and
initial outward error is 0.5 m. No pose or velocity corrections occur during
the run.

| Speed | Inward overshoot, baseline → tuned | Late path RMSE, tuned |
| --- | ---: | ---: |
| 0.8 m/s | 2.87 → 2.63 cm | 0.09 cm |
| 3 m/s | 5.09 → 5.96 cm | 2.06 cm |
| 6 m/s | 13.56 → 8.73 cm | 7.45 cm |
| 8 m/s | 18.08 → 1.50 cm | 14.50 cm |

The 3/8 m/s tuned rows use the active 0821 surface (0.537758); 0.8/6 m/s use
the shared reference surface (0.523160), with the same shared tire/steering
coefficients. Each independent episode uses one constant surface throughout.
The high-speed steady outward path bias is consistent with understeer and
is larger than the native baseline; all path metrics did not improve. The
3 m/s inward overshoot increased by 0.87 cm. Controller tuning to eliminate
these remaining path errors is separate from identifying realistic tire forces.

## Checks, artifacts and limits

- 181 unit/differential tests passed, including native C mapping/inverse,
  delay, surface selection, physical force limits and passivity checks.
- Surface reload and same-session native-material restoration checks passed.
- Runtime verification: headless=true, physics=200 Hz, approximately 1.00 real
  time, IMU/LiDAR publishing, no adapter error and zero speed target.
- Physics state guards sample at 50 Hz in the batch harness; solver rate is
  200 Hz. PASS does not prove absence of every sub-sample contact excursion.
- Initial synthetic experiment files `baseline/` and `custom_initial/` used
  measured joint positions instead of authored suspension targets. This
  experimental setup error was fixed before the replay A/B and validation
  checks. Those early synthetic failures are not evidence about production.

Artifacts: `final_validation_summary.json`, `final_validation.png`,
`frozen_dataset_validation_scores.json`, `frozen_dataset_validation/`,
`dataset_surface_friction_candidates.json`, and `applied_configuration.json`.
Before-default snapshots are in `before_tuned_defaults/`. Reproduction tools
are `../../tools/tune_lateral_headless.py`, `score_lateral_replays.py`, and
`summarize_lateral_tuning.py`.

The adapter follows the documented normal-contact and force-at-position tensor
interfaces: [Omni Physics Python API](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/109.0/extensions/runtime/source/omni.physics.tensors/docs/api/python.html).
