#include "tire_model.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

static double clip(double value, double low, double high) {
    return fmin(high, fmax(low, value));
}

int damvi_tire_validate(const DamviTireConfig *c) {
    if (!c) return -1;
    const double values[] = {c->radius_m, c->longitudinal_stiffness_n,
        c->lateral_stiffness_n_per_rad, c->reference_load_n,
        c->stiffness_load_exponent, c->friction_longitudinal,
        c->friction_lateral, c->slip_speed_floor_mps,
        c->longitudinal_relaxation_length_m, c->lateral_relaxation_length_m,
        c->inverse_effective_mass_longitudinal_per_kg,
        c->inverse_effective_mass_lateral_per_kg};
    for (size_t i = 0; i < sizeof(values)/sizeof(values[0]); ++i)
        if (!isfinite(values[i])) return -1;
    if (c->radius_m <= 0. || c->reference_load_n <= 0. ||
        c->longitudinal_stiffness_n < 0. || c->lateral_stiffness_n_per_rad < 0. ||
        c->stiffness_load_exponent < 0. || c->stiffness_load_exponent > 2. ||
        c->friction_longitudinal < 0. || c->friction_lateral < 0. ||
        c->slip_speed_floor_mps <= 0. ||
        c->longitudinal_relaxation_length_m < 0. ||
        c->lateral_relaxation_length_m < 0. ||
        c->inverse_effective_mass_longitudinal_per_kg <= 0. ||
        c->inverse_effective_mass_lateral_per_kg <= 0.) return -1;
    return 0;
}

/* Solve an implicit relaxation + velocity response, separately on either
 * tangent axis. Slip is defined so a positive force opposes positive slip:
 * long = R*omega-vx, lateral = -vy_contact.
 *
 * F = (1-alpha)*F_previous + alpha*static_force(slip-dt*inverse_mass*F).
 *
 * The no-reversal impulse cap and the nonnegative force magnitude make the
 * update passive for the supplied effective mass, including at zero speed.
 * Projection of combined Fx/Fy can only reduce this impulse. */
static double implicit_force(double slip, double speed_scale, double stiffness,
                             double force_limit, double previous_force,
                             double inverse_mass, double relaxation_length,
                             double dt, int lateral) {
    if (slip == 0. || stiffness == 0. || force_limit == 0.) return 0.;
    const double direction = slip > 0. ? 1. : -1.;
    const double magnitude = fabs(slip);
    const double alpha = relaxation_length > 0.
        ? -expm1(-dt*speed_scale/relaxation_length) : 1.;
    const double previous = clip(previous_force*direction, 0., force_limit);
    const double upper = fmin(force_limit, magnitude/(dt*inverse_mass));
    if (upper <= 0.) return 0.;
    if (!lateral) {
        const double gain = stiffness/speed_scale;
        const double force = ((1.-alpha)*previous + alpha*gain*magnitude) /
            (1.+alpha*gain*dt*inverse_mass);
        return direction*clip(force, 0., upper);
    }
    double low = 0., high = upper;
    /* Monotonic scalar solve; 32 iterations gives sub-microNewton precision
     * for the physical loads of this 3.5 kg car. */
    for (unsigned i = 0; i < 32; ++i) {
        const double force = .5*(low+high);
        const double future_slip = fmax(0., magnitude-dt*inverse_mass*force);
        const double steady = force_limit*tanh(
            stiffness*atan(future_slip/speed_scale)/force_limit);
        const double desired = (1.-alpha)*previous + alpha*steady;
        if (force < desired) low = force; else high = force;
    }
    return direction*.5*(low+high);
}

int damvi_tire_step(const DamviTireConfig *c, DamviTireState *state,
                    const DamviTireInput *input, double dt,
                    DamviTireOutput *out) {
    if (damvi_tire_validate(c) || !state || !input || !out ||
        !isfinite(dt) || dt <= 0. || dt > .05) return -1;
    const double values[] = {input->normal_load_n, input->longitudinal_speed_mps,
        input->lateral_contact_speed_mps, input->wheel_angular_speed_rad_s,
        state->longitudinal_force_n, state->lateral_force_n};
    for (size_t i = 0; i < sizeof(values)/sizeof(values[0]); ++i)
        if (!isfinite(values[i])) return -1;
    DamviTireOutput result;
    memset(&result, 0, sizeof(result));
    if (input->normal_load_n <= 0.) {
        memset(state, 0, sizeof(*state));
        *out = result;
        return 0;
    }
    const double load = input->normal_load_n;
    const double tread_speed = c->radius_m*input->wheel_angular_speed_rad_s;
    const double slip_long = tread_speed-input->longitudinal_speed_mps;
    const double slip_lateral = -input->lateral_contact_speed_mps;
    const double scale_long = fmax(c->slip_speed_floor_mps,
        fmax(fabs(input->longitudinal_speed_mps), fabs(tread_speed)));
    const double scale_lateral = fmax(c->slip_speed_floor_mps,
        fabs(input->longitudinal_speed_mps));
    const double load_factor = pow(load/c->reference_load_n,
                                   c->stiffness_load_exponent);
    const double limit_long = c->friction_longitudinal*load;
    const double limit_lateral = c->friction_lateral*load;
    if (!isfinite(tread_speed) || !isfinite(load_factor) ||
        !isfinite(limit_long) || !isfinite(limit_lateral)) return -1;
    double fx = implicit_force(slip_long, scale_long,
        c->longitudinal_stiffness_n*load_factor, limit_long,
        state->longitudinal_force_n,
        c->inverse_effective_mass_longitudinal_per_kg,
        c->longitudinal_relaxation_length_m, dt, 0);
    double fy = implicit_force(slip_lateral, scale_lateral,
        c->lateral_stiffness_n_per_rad*load_factor, limit_lateral,
        state->lateral_force_n,
        c->inverse_effective_mass_lateral_per_kg,
        c->lateral_relaxation_length_m, dt, 1);
    const double normalized_long = limit_long > 0. ? fx/limit_long : 0.;
    const double normalized_lateral = limit_lateral > 0. ? fy/limit_lateral : 0.;
    const double utilization = hypot(normalized_long, normalized_lateral);
    if (utilization > 1.) { fx /= utilization; fy /= utilization; }
    result.longitudinal_force_n = fx;
    result.lateral_force_n = fy;
    result.longitudinal_slip_ratio = slip_long/scale_long;
    result.lateral_slip_angle_rad = atan2(input->lateral_contact_speed_mps,
                                          scale_lateral);
    result.dissipated_contact_power_w = fx*(-slip_long)+
                                        fy*input->lateral_contact_speed_mps;
    result.friction_ellipse_utilization = fmin(1., utilization);
    result.reaction_torque_nm = -c->radius_m*fx;
    result.effective_normal_load_n = load;
    if (!isfinite(fx) || !isfinite(fy) ||
        !isfinite(result.dissipated_contact_power_w)) return -1;
    state->longitudinal_force_n = fx;
    state->lateral_force_n = fy;
    *out = result;
    return 0;
}

int damvi_tire_step_four(const DamviTireConfig config[4], DamviTireState state[4],
                         const DamviTireInput input[4], double dt,
                         DamviTireOutput output[4]) {
    if (!config || !state || !input || !output) return -1;
    /* Transactional validation/update avoids half-updated wheel state. */
    DamviTireState next_state[4];
    DamviTireOutput next_output[4];
    memcpy(next_state, state, sizeof(next_state));
    for (unsigned i = 0; i < 4; ++i)
        if (damvi_tire_step(config+i, next_state+i, input+i, dt,
                            next_output+i)) return -1;
    memcpy(state, next_state, sizeof(next_state));
    memcpy(output, next_output, sizeof(next_output));
    return 0;
}
