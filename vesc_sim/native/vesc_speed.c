/* SPDX-License-Identifier: GPL-3.0-or-later
 * Speed-loop behavior adapted from VESC firmware motor/foc_math.c.
 * Upstream copyright 2016-2022 Benjamin Vedder, benjamin@vedder.se.
 * See firmware_notes.md for pinned sources and LICENSE for license terms.
 */
#include "vesc_speed.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

static double clamp(double x, double low, double high) {
    return x < low ? low : (x > high ? high : x);
}

void vesc_speed_defaults(VescSpeedConfig *c) {
    if (!c) return;
    *c = (VescSpeedConfig){0.004, 0.004, 0.0001, 0.2,
                          900.0, 25000.0, 1.0, 60.0, 1.0,
                          -100000.0, 100000.0, 0.0};
}

int vesc_speed_validate(const VescSpeedConfig *c) {
    if (!c) return -1;
    if (!isfinite(c->kp) || !isfinite(c->ki) || !isfinite(c->kd) ||
        !isfinite(c->kd_filter) || !isfinite(c->min_erpm) ||
        !isfinite(c->ramp_erpm_per_s) || !isfinite(c->allow_braking) ||
        !isfinite(c->current_max) || !isfinite(c->current_max_scale) ||
        !isfinite(c->erpm_min) || !isfinite(c->erpm_max) ||
        !isfinite(c->speed_source)) return -1;
    if (c->kp < 0 || c->ki < 0 || c->kd < 0 || c->kd_filter < 0 ||
        c->kd_filter > 1 || c->min_erpm < 0 || c->ramp_erpm_per_s < 0 ||
        (c->allow_braking != 0 && c->allow_braking != 1) ||
        c->current_max <= 0 || c->current_max_scale < 0 ||
        c->current_max_scale > 1 || c->erpm_min > 0 || c->erpm_max < 0 ||
        c->erpm_min >= c->erpm_max ||
        (c->speed_source != 0 && c->speed_source != 1 && c->speed_source != 2))
        return -1;
    return 0;
}

void vesc_speed_reset(VescSpeedState *s) {
    if (s) memset(s, 0, sizeof(*s));
}

int vesc_speed_step(const VescSpeedConfig *c, VescSpeedState *s,
                    double target, double measured, double dt,
                    VescSpeedOutput *out) {
    if (!s || !out || vesc_speed_validate(c) != 0 ||
        !isfinite(target) || !isfinite(measured) || !isfinite(dt) || dt <= 0 ||
        !isfinite(s->initialized) || !isfinite(s->target_erpm) ||
        !isfinite(s->integral) || !isfinite(s->previous_error) ||
        !isfinite(s->derivative_filtered) ||
        (s->initialized != 0 && s->initialized != 1)) return -1;
    VescSpeedState next = *s;
    VescSpeedOutput result = {0};
    if (!next.initialized) {
        next.target_erpm = measured;
        next.initialized = 1;
    }
    if (c->ramp_erpm_per_s > 0) {
        double step = c->ramp_erpm_per_s * dt;
        double difference = target - next.target_erpm;
        if (!isfinite(step) || !isfinite(difference)) return -1;
        next.target_erpm += clamp(difference, -step, step);
        next.target_erpm = clamp(next.target_erpm, c->erpm_min, c->erpm_max);
    } else {
        /* Upstream only applies this setpoint clamp in the ramp branch. */
        next.target_erpm = target;
    }
    double error = next.target_erpm - measured;
    if (!isfinite(error)) return -1;
    result.target_erpm = next.target_erpm;
    if (fabs(next.target_erpm) < c->min_erpm) {
        next.integral = 0;
        next.previous_error = error;
        result.low_speed_brake_requested = 1;
    } else {
        double proportional = error * c->kp / 20.0;
        double derivative = (error - next.previous_error) * (c->kd / dt) / 20.0;
        next.derivative_filtered -= c->kd_filter * (next.derivative_filtered - derivative);
        next.previous_error = error;
        double raw = proportional + next.integral + next.derivative_filtered;
        double integral = next.integral + error * c->ki * dt / 20.0;
        if (!isfinite(proportional) || !isfinite(derivative) ||
            !isfinite(next.derivative_filtered) || !isfinite(raw) ||
            !isfinite(integral)) return -1;
        result.normalized_output = clamp(raw, -1, 1);
        result.output_saturated = raw < -1 || raw > 1;
        next.integral = clamp(integral, -1, 1);
        result.integral_saturated = integral < -1 || integral > 1;
        if (c->ki < 1e-9) next.integral = 0;
        if (!c->allow_braking &&
            ((measured > 20 && result.normalized_output < 0) ||
             (measured < -20 && result.normalized_output > 0))) {
            result.normalized_output = 0;
            result.braking_suppressed = 1;
        }
        result.current_a = result.normalized_output * c->current_max * c->current_max_scale;
        if (!isfinite(result.current_a)) return -1;
    }
    *s = next;
    *out = result;
    return 0;
}
