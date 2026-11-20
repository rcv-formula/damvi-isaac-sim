# VESC speed controller reference

Upstream reference: [`vedderb/bldc`, e9e46af990ff2809c862c56f9c226d2be9fb1fcb](https://github.com/vedderb/bldc/tree/e9e46af990ff2809c862c56f9c226d2be9fb1fcb), checked 2026-11-13. Its `conf_general.h` identifies 7.01, test version 1. This is a pinned research reference, **not a claim about the real vehicle's firmware**. Firmware version and VESC Tool motor/app configuration must be obtained before calibration.

## Implemented subset

`native/vesc_speed.c` implements the signed outer speed loop with electrical RPM input/output: speed-mode entry initializes the ramp at measured RPM, both acceleration and deceleration slew by `s_pid_ramp_erpms_s * dt`, optional ERPM setpoint limits, P/I/D, derivative EMA, output and integral clamping, braking enable, and minimum setpoint behavior. It returns a requested phase q-axis current; the physics adapter must apply electrical and motor limits and convert current to torque.

For error `e = ramped_target - selected_measured_erpm`:

```
P = e * kp / 20
D_raw = (e - previous_error) * kd / (20 * dt)
D_filtered += kd_filter * (D_raw - D_filtered)
output = clamp(P + old_I + D_filtered, -1, 1)
I = clamp(old_I + e * ki * dt / 20, -1, 1)
if ki < 1e-9: I = 0
Iq_request = output * current_max * current_max_scale
```

The ordering of old I versus newly integrated I follows firmware. This is a clamped integrator, not back-calculation anti-windup. When braking is disabled, measured ERPM above +20 suppresses negative output and below -20 suppresses positive output. A negative target uses the same signed loop; there is no separate mandatory stop-before-reverse state machine in this loop.

Sources: [speed command/mode entry](https://github.com/vedderb/bldc/blob/e9e46af990ff2809c862c56f9c226d2be9fb1fcb/motor/mcpwm_foc.c#L754-L775), [speed PID](https://github.com/vedderb/bldc/blob/e9e46af990ff2809c862c56f9c226d2be9fb1fcb/motor/foc_math.c#L501-L589), [D EMA](https://github.com/vedderb/bldc/blob/e9e46af990ff2809c862c56f9c226d2be9fb1fcb/util/ema.h#L78-L90), [PID thread rate](https://github.com/vedderb/bldc/blob/e9e46af990ff2809c862c56f9c226d2be9fb1fcb/motor/mcpwm_foc.c#L4503-L4535).

The `speed_source` field accepts PLL=0, FAST=1, FASTER=2. The caller supplies the selected measurement. Electrical observers, sensor startup, encoder-index acquisition, and the no-index openloop setpoint clamp are not emulated. Simulated mechanical rotation is directly available. All caller values must use a consistent motor sign convention; physical mount-axis and configured direction inversion belong in the adapter. ERPM limits are likewise motor-signed.

## Braking and zero speed

With `s_pid_allow_braking=true`, the speed loop can command torque opposite the current rotation and thus decelerate. This alone does not model battery energy acceptance. Firmware applies phase and input current limits downstream, with battery voltage, power and BMS restrictions. See [current limits](https://github.com/vedderb/bldc/blob/e9e46af990ff2809c862c56f9c226d2be9fb1fcb/motor/mcpwm_foc.c#L3657-L3682) and [regen voltage/BMS limits](https://github.com/vedderb/bldc/blob/e9e46af990ff2809c862c56f9c226d2be9fb1fcb/motor/mc_interface.c#L2481-L2519).

**A zero speed command is not simply coast in upstream FOC.** Below `s_pid_min_erpm`, the outer loop clears I and requests zero Iq, but the inner FOC switches to duty=0 electrical braking. Explicit brake-current mode also shorts phases around zero duty or sign changes to prevent reversal, and otherwise uses `Iq = -sign(speed) * abs(brake_current)`. Phase shorting is electrical braking, but is not purely energy returned to the battery. Sources: [shorting and low-speed branch](https://github.com/vedderb/bldc/blob/e9e46af990ff2809c862c56f9c226d2be9fb1fcb/motor/mcpwm_foc.c#L3347-L3377), [brake-current sign](https://github.com/vedderb/bldc/blob/e9e46af990ff2809c862c56f9c226d2be9fb1fcb/motor/mcpwm_foc.c#L3451-L3454).

This module returns `low_speed_brake_requested=1` and zero current below the minimum. The adapter implements the chosen low-speed policy. A strict regen-only/no-short/no-active-hold policy is an intentional simplification relative to firmware. Distinguish that policy from merely having no mechanical friction brake.

## Defaults and calibration

All following numbers are generic upstream defaults, **uncalibrated for this vehicle**, and may be overridden by board configuration. [Default definitions](https://github.com/vedderb/bldc/blob/e9e46af990ff2809c862c56f9c226d2be9fb1fcb/motor/mcconf_default.h#L38-L172).

| Parameter | Default |
|---|---:|
| kp / ki / kd | 0.004 / 0.004 / 0.0001 |
| kd_filter | 0.2 |
| min_erpm | 900 |
| ramp_erpm_per_s | 25000 |
| allow_braking | true |
| speed source | PLL |
| outer-loop rate | 1000 Hz |
| phase current maximum / minimum | 60 / -60 A |
| input current maximum / minimum | 99 / -60 A |
| current scale | 1 |
| ERPM minimum / maximum | -100000 / 100000 |

The native controller implements the phase maximum as request scaling only; actual downstream phase/input current, voltage, loss and regeneration limits belong to the adapter. Do not infer motor torque constant, gear ratio, battery acceptance or measured acceleration from these defaults. The user-supplied 1538 ERPM per m/s calibration belongs to the wheel-rotation conversion and does not require an additional pole-pair multiplication.

## Selected real-vehicle export and 6.05 comparison

The user selected `/media/shin/A0C8B261C8B2357E/Users/tlswo/Desktop/RCV-formula-motor-settings_default.xml`. It contains `kp=.009`, `ki=0`, `kd=1.5e-5`, `kd_filter=0`, minimum 800 ERPM, ramp 50000 ERPM/s, braking enabled, phase max/min +62/-62 A, and `sp_pid_loop_rate=7`. **Rate index 7 is 5000 Hz**, in both 6.05 and the newer reference. These replace generic defaults when loading the selected export. A zero derivative filter coefficient retains its previous filtered state; starting from reset, D stays zero. It must not be reinterpreted as an unfiltered derivative.

The accompanying VESC Tool 6.05 executable suggests checking 6.05, but does not establish the actual flashed version. Upstream has no 6.05 tag. A pinned late 6.05 release reference is [`b3db0c1182e857c11c5749dd2c4dadec2381a9a6`](https://github.com/vedderb/bldc/tree/b3db0c1182e857c11c5749dd2c4dadec2381a9a6), immediately before the 6.06 version bump; `conf_general.h` says 6.05 test 0. Its [speed PID](https://github.com/vedderb/bldc/blob/b3db0c1182e857c11c5749dd2c4dadec2381a9a6/motor/foc_math.c#L486-L581) uses the same equations, update ordering, minimum threshold and brake enable. The older `UTILS_LP_FAST` macro is algebraically identical to the newer EMA. Newer firmware corrects the asymmetric ERPM clamp when direction is inverted; this module expects already motor-signed limits, so direction conversion remains the adapter's responsibility.

The selected XML's `foc_speed_soure=1` (upstream spelling) is **not** `s_pid_speed_source`: it chooses observer/corrected phase speed, not PLL/FAST/FASTER PID feedback. The XML lacks `s_pid_speed_source`. Early 6.05 read `motor_state.speed_rad_s`; selectable PID feedback was added in [9eb978ef](https://github.com/vedderb/bldc/commit/9eb978ef) on 2023-07-01. Thus this export does not determine exact firmware feedback source. The simulation uses mechanical rotation directly and documents the electrical observer as omitted.

`m_invert_direction=1` describes the physical motor convention. ROS positive-forward command/telemetry should remain positive; physical wheel axis mapping belongs separately in the adapter. XML gear ratio and tire diameter are provenance, and must not silently override the configured USD wheel geometry or the requested 1538 ERPM/(m/s) calibration.

Run native behavioral tests with `python3 -m unittest discover -s damvi_vesc_sim/tests -p test_speed_native.py -v`. Tests compile a temporary C99 shared library and exercise signed ramps, PID numerical values, braking, reverse, zero/minimum speed, saturation, and atomic rejection of invalid inputs.
