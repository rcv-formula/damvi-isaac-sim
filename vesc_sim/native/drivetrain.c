#include "drivetrain.h"
#include <math.h>
#include <string.h>
static double clip(double x, double lo, double hi) { return fmin(hi, fmax(lo, x)); }
static double sign(double x) { return (x > 0) - (x < 0); }
int drive_validate(const DriveConfig *c) {
    if (!c) return -1;
    const double v[] = {c->radius,c->erpm_gain,c->pole_pairs,c->torque_constant,
        c->phase_resistance,c->current_time_constant,c->current_drive_max,
        c->current_brake_max,c->battery_voltage,c->battery_drive_max,
        c->battery_regen_max,c->gear_efficiency,c->front_diff_damping,
        c->rear_diff_damping,c->center_stiffness,c->center_damping,
        c->coupling_torque_max,c->wheel_coulomb,c->wheel_viscous,c->wheel_inertia};
    for (unsigned i=0;i<sizeof(v)/sizeof(v[0]);++i) if (!isfinite(v[i]) || v[i]<0) return -1;
    if (c->radius<=0 || c->erpm_gain<=0 || c->pole_pairs<=0 ||
        c->torque_constant<=0 || c->phase_resistance<=0 || c->battery_voltage<=0 ||
        c->gear_efficiency<=0 || c->gear_efficiency>1 || c->wheel_inertia<=0) return -1;
    return 0;
}
int drive_step(const DriveConfig *c, DriveState *s, const double w[4],
               double req, double short_brake, double dt, DriveOutput *o) {
    if (drive_validate(c) || !s || !w || !o || !isfinite(dt) || dt<=0 ||
        !isfinite(req) || !isfinite(short_brake) || !isfinite(s->current) ||
        !isfinite(s->center_twist) || !isfinite(s->regen_joules)) return -1;
    for (int j=0;j<4;++j) if (!isfinite(w[j])) return -1;
    memset(o,0,sizeof(*o));
    const double pi=3.14159265358979323846;
    double avg=(w[0]+w[1]+w[2]+w[3])*0.25;
    /* Effective ratio follows the user's calibrated ERPM/m/s on this USD's
       physical wheel radius. Do not also multiply XML gearing or pole pairs. */
    double ratio=c->erpm_gain*c->radius*(2*pi)/(60*c->pole_pairs);
    o->erpm=c->erpm_gain*c->radius*avg;
    o->shaft_omega=ratio*avg;
    if (short_brake>0.5) {
        req=-c->torque_constant*o->shaft_omega/c->phase_resistance;
        o->electrical_brake=1;
    }
    int braking=req*o->shaft_omega<0;
    double limit=braking?c->current_brake_max:c->current_drive_max;
    req=clip(req,-limit,limit);
    /* Average voltage/current envelope. Regeneration is limited by battery
       power; short-circuit braking dissipates energy in the motor instead. */
    if (short_brake<=0.5) {
        double emf=c->torque_constant*o->shaft_omega;
        double voltage_limit=fmax(0,(c->battery_voltage-fabs(emf))/c->phase_resistance);
        if (!braking) req=clip(req,-voltage_limit,voltage_limit);
        double p=emf*req+c->phase_resistance*req*req;
        double max_p=c->battery_voltage*c->battery_drive_max;
        double min_p=-c->battery_voltage*c->battery_regen_max;
        if (p>max_p || p<min_p) {
            double lo=0,hi=1;
            for (int n=0;n<60;++n) {
                double scale=(lo+hi)*0.5, i=req*scale;
                double power=emf*i+c->phase_resistance*i*i;
                if (power>max_p || power<min_p) hi=scale;
                else lo=scale;
            }
            req*=lo;
        }
    }
    double alpha=c->current_time_constant>0 ? -expm1(-dt/c->current_time_constant):1;
    s->current+=alpha*(req-s->current);
    /* When short braking crosses zero, an old current must not accelerate the
       opposite direction. Unlike a mechanical holding brake this vanishes at rest. */
    if (short_brake>0.5 && s->current*o->shaft_omega>=0) s->current=0;
    o->current=s->current;
    double motor_torque=c->torque_constant*s->current;
    double efficiency=motor_torque*o->shaft_omega<0 ? 1/c->gear_efficiency:c->gear_efficiency;
    double wheel_torque=motor_torque*ratio*efficiency;
    double delta=(w[0]+w[1]-w[2]-w[3])*0.5;
    /* Backward-Euler torsional coupling: no independently driven front/rear
       inputs. A finite shaft compliance is deliberate, not an ideal lock. */
    double inv_effective_mass=1/c->wheel_inertia;
    double k=c->center_stiffness, d=c->center_damping;
    double transfer=(k*(s->center_twist+dt*delta)+d*delta)/
        (1+inv_effective_mass*dt*(d+k*dt));
    transfer=clip(transfer,-c->coupling_torque_max,c->coupling_torque_max);
    s->center_twist+=dt*(delta-inv_effective_mass*dt*transfer);
    o->center_speed_error=delta;
    o->center_torque=transfer;
    double f=c->front_diff_damping*(w[0]-w[1])/
        (1+2*c->front_diff_damping*dt/c->wheel_inertia);
    double r=c->rear_diff_damping*(w[2]-w[3])/
        (1+2*c->rear_diff_damping*dt/c->wheel_inertia);
    o->torque[0]=wheel_torque/4-transfer/2-f;
    o->torque[1]=wheel_torque/4-transfer/2+f;
    o->torque[2]=wheel_torque/4+transfer/2-r;
    o->torque[3]=wheel_torque/4+transfer/2+r;
    for(int j=0;j<4;++j) {
        double friction=c->wheel_coulomb*sign(w[j])+c->wheel_viscous*w[j];
        friction=clip(friction,-fabs(w[j])*c->wheel_inertia/dt,fabs(w[j])*c->wheel_inertia/dt);
        o->torque[j]-=friction;
    }
    double power=motor_torque*o->shaft_omega+c->phase_resistance*s->current*s->current;
    o->battery_current=short_brake>0.5 ? 0:power/c->battery_voltage;
    o->regen_power=fmax(0,-o->battery_current*c->battery_voltage);
    s->regen_joules+=o->regen_power*dt;
    return 0;
}
