#ifndef DAMVI_TIRE_MODEL_H
#define DAMVI_TIRE_MODEL_H

/* Optional dissipative tire model. Contact normal support is supplied by PhysX;
 * the caller must disable the default tire/road tangential friction before
 * applying these forces. Forces are applied ONCE at the wheel contact point.
 * reaction_torque_nm is diagnostic: do not apply it a second time. */
typedef struct {
    double radius_m;
    double longitudinal_stiffness_n;
    double lateral_stiffness_n_per_rad;
    double reference_load_n;
    double stiffness_load_exponent;
    double friction_longitudinal;
    double friction_lateral;
    double slip_speed_floor_mps;
    double longitudinal_relaxation_length_m;
    double lateral_relaxation_length_m;
    double inverse_effective_mass_longitudinal_per_kg;
    double inverse_effective_mass_lateral_per_kg;
} DamviTireConfig;

typedef struct {
    double longitudinal_force_n;
    double lateral_force_n;
} DamviTireState;

typedef struct {
    double normal_load_n;
    double longitudinal_speed_mps;
    double lateral_contact_speed_mps;
    double wheel_angular_speed_rad_s;
} DamviTireInput;

typedef struct {
    double longitudinal_force_n;
    double lateral_force_n;
    double longitudinal_slip_ratio;
    double lateral_slip_angle_rad;
    double dissipated_contact_power_w;
    double friction_ellipse_utilization;
    double reaction_torque_nm;
    double effective_normal_load_n;
} DamviTireOutput;

int damvi_tire_validate(const DamviTireConfig *config);
int damvi_tire_step(const DamviTireConfig *config, DamviTireState *state,
                    const DamviTireInput *input, double dt_s,
                    DamviTireOutput *output);
int damvi_tire_step_four(const DamviTireConfig config[4], DamviTireState state[4],
                         const DamviTireInput input[4], double dt_s,
                         DamviTireOutput output[4]);
#endif
