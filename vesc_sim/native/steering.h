#ifndef DAMVI_STEERING_H
#define DAMVI_STEERING_H
#include <stdint.h>
typedef struct {
    double input_limit, angle_limit;
    double wheelbase, track, time_constant, rate_limit;
} SteeringConfig;
typedef struct { double center_angle; } SteeringState;
typedef struct { double target_center, center, left, right; } SteeringOutput;
/* Separate state keeps the original steering configuration/state ABI unchanged.
   Zero-initialize this structure when starting or changing the delay. */
#define STEERING_DELAY_CAPACITY 1024u
typedef struct {
    double clock, clock_compensation;
    double timestamps[STEERING_DELAY_CAPACITY];
    double commands[STEERING_DELAY_CAPACITY];
    uint32_t head, count;
} SteeringDelayState;
/* Sample input at the current left edge t, return its zero-order-held value at
   t-delay, then advance t by dt. Before time zero the command is zero. There is
   no interpolation between command samples. delay is in [0, .5] seconds;
   positive delay requires dt >= .0005 s. Zero delay passes input through and
   clears history. Returns -1 on invalid input/state, -2 on capacity overflow. */
int steering_delay_step(SteeringDelayState *s, double input_rad, double dt,
                        double delay_seconds, double *delayed_input_rad);
int steering_validate(const SteeringConfig *c);
int steering_step(const SteeringConfig *c, SteeringState *s, double input_rad,
                  double dt, const double *input_knots, const double *angle_knots,
                  uint32_t knot_count, SteeringOutput *out);
#endif
