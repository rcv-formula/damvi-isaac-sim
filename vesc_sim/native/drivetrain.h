#ifndef DAMVI_DRIVETRAIN_H
#define DAMVI_DRIVETRAIN_H
/* Wheel order FL, FR, RL, RR. SI units. This is an averaged electrical and
 * compliant mechanical model, not a PWM/FOC or battery electrochemistry solver. */
typedef struct {
    double radius, erpm_gain, pole_pairs, torque_constant, phase_resistance;
    double current_time_constant, current_drive_max, current_brake_max;
    double battery_voltage, battery_drive_max, battery_regen_max;
    double gear_efficiency, front_diff_damping, rear_diff_damping;
    double center_stiffness, center_damping, coupling_torque_max;
    double wheel_coulomb, wheel_viscous, wheel_inertia;
} DriveConfig;
typedef struct { double current, center_twist, regen_joules; } DriveState;
typedef struct {
    double torque[4];
    double erpm, shaft_omega, current, battery_current, regen_power;
    double center_speed_error, center_torque, electrical_brake;
} DriveOutput;
int drive_validate(const DriveConfig *c);
int drive_step(const DriveConfig *c, DriveState *s, const double omega[4],
               double requested_current, double short_brake, double dt,
               DriveOutput *out);
#endif
