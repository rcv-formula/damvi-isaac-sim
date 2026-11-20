#ifndef DAMVI_VESC_SPEED_H
#define DAMVI_VESC_SPEED_H

#ifdef __cplusplus
extern "C" {
#endif

/* ABI v1: fields are doubles, in declaration order, for simple ctypes use.
 * Defaults are upstream firmware defaults, NOT measured vehicle calibration.
 * Inputs/outputs use motor-signed electrical RPM, never chassis velocity.
 */
typedef struct {
    double kp, ki, kd, kd_filter;
    double min_erpm, ramp_erpm_per_s, allow_braking;
    double current_max, current_max_scale;
    double erpm_min, erpm_max;
    /* 0=PLL, 1=FAST, 2=FASTER. Caller supplies the selected measurement;
     * this module does not emulate the electrical observer/PLL. */
    double speed_source;
} VescSpeedConfig;

typedef struct {
    double initialized, target_erpm, integral, previous_error, derivative_filtered;
} VescSpeedState;

typedef struct {
    double current_a, target_erpm, normalized_output;
    /* Firmware requests duty=0 electrical braking below min_erpm. The
     * adapter must choose how to handle this when enforcing regen-only. */
    double low_speed_brake_requested;
    double output_saturated, integral_saturated, braking_suppressed;
} VescSpeedOutput;

void vesc_speed_defaults(VescSpeedConfig *config);
int vesc_speed_validate(const VescSpeedConfig *config);
/* Reset when leaving speed mode or restarting simulation. */
void vesc_speed_reset(VescSpeedState *state);
/* Returns 0 on success, -1 for invalid/nonfinite arguments or overflow.
 * Error leaves state and output untouched. dt is seconds; ramp is ERPM/s.
 * First step seeds the ramp from measured_erpm, like speed-mode entry.
 * current_a is a request: caller must apply phase/battery/regen limits.
 * erpm_min/max are in the same motor sign convention as input values.
 * Encoder-index-not-found startup/openloop and observer are not simulated.
 */
int vesc_speed_step(const VescSpeedConfig *config, VescSpeedState *state,
                    double target_erpm, double measured_erpm, double dt,
                    VescSpeedOutput *output);

#ifdef __cplusplus
}
#endif
#endif
