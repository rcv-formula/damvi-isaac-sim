#include "steering.h"
#include <float.h>
#include <math.h>
static double clip(double x,double a,double b) { return fmin(b,fmax(a,x)); }
int steering_delay_step(SteeringDelayState *s, double input, double dt,
                        double delay, double *delayed_input) {
    if (!s || !delayed_input || !isfinite(input) || !isfinite(dt) || dt<=0 ||
        !isfinite(delay) || delay<0 || delay>.5 || !isfinite(s->clock) || s->clock<0 ||
        !isfinite(s->clock_compensation) ||
        s->head>=STEERING_DELAY_CAPACITY || s->count>STEERING_DELAY_CAPACITY) return -1;
    if (delay==0) {
        /* Keep the no-delay path independent of history size and solver dt. */
        s->clock=0.;s->clock_compensation=0.;s->head=0;s->count=0;*delayed_input=input;
        return 0;
    }
    /* Compensated accumulation prevents a fixed delay from slipping a solver
       tick after thousands of equal timesteps due to clock summation error. */
    double increment=dt-s->clock_compensation;
    double next_clock=s->clock+increment;
    double compensation=(next_clock-s->clock)-increment;
    if (dt<.0005 || !isfinite(next_clock) || next_clock<=s->clock ||
        !isfinite(compensation)) return -1;
    uint32_t head=s->head,count=s->count;
    double previous=-1.;
    for (uint32_t j=0;j<count;++j) {
        uint32_t i=(head+j)%STEERING_DELAY_CAPACITY;
        double t=s->timestamps[i];
        if (!isfinite(t) || t<0 || t>s->clock || t<=previous ||
            !isfinite(s->commands[i])) return -1;
        previous=t;
    }
    if (count && previous>=s->clock) return -1;
    double query=s->clock-delay;
    /* Only compensate roundoff at a sample boundary, not a fraction of dt. */
    double tolerance=16*DBL_EPSILON*fmax(1.,fabs(s->clock));
    while (count>1 && s->timestamps[(head+1)%STEERING_DELAY_CAPACITY]<=query+tolerance) {
        head=(head+1)%STEERING_DELAY_CAPACITY;--count;
    }
    if (count==STEERING_DELAY_CAPACITY) return -2;
    uint32_t tail=(head+count)%STEERING_DELAY_CAPACITY;
    s->timestamps[tail]=s->clock;s->commands[tail]=input;++count;
    /* A strictly positive delay never selects the command just sampled at t,
       even if its tiny delay is within the timestamp boundary tolerance. */
    *delayed_input=head!=tail && s->timestamps[head]<=query+tolerance?s->commands[head]:0.;
    s->head=head;s->count=count;s->clock=next_clock;s->clock_compensation=compensation;
    return 0;
}
int steering_validate(const SteeringConfig *c) {
    if (!c) return -1;
    const double v[]={c->input_limit,
        c->angle_limit,c->wheelbase,c->track,c->time_constant,c->rate_limit};
    for (unsigned i=0;i<sizeof(v)/sizeof(v[0]);++i) if (!isfinite(v[i])) return -1;
    if (c->input_limit<=0 ||
        c->angle_limit<=0 || c->angle_limit>=1.4 || c->wheelbase<=0 || c->track<=0 ||
        c->time_constant<0 || c->rate_limit<=0 ||
        tan(c->angle_limit)*c->track/2>=c->wheelbase) return -1;
    return 0;
}
/* Fritsch-Butland monotone cubic Hermite derivatives, as used by PCHIP.
   Endpoint formulas preserve shape and avoid overshoot at a flat/deadband knot. */
static double slope(const double *x,const double *y,uint32_t n,uint32_t i) {
    if (n==2) return (y[1]-y[0])/(x[1]-x[0]);
    if (i==0 || i==n-1) {
        uint32_t a=i==0?0:n-2,b=i==0?1:n-3;
        double h0=x[a+1]-x[a],h1=x[b+1]-x[b];
        double d0=(y[a+1]-y[a])/h0,d1=(y[b+1]-y[b])/h1;
        double d=((2*h0+h1)*d0-h0*d1)/(h0+h1);
        if (d*d0<=0) return 0;
        if (d0*d1<=0 && fabs(d)>3*fabs(d0)) return 3*d0;
        return d;
    }
    double h0=x[i]-x[i-1],h1=x[i+1]-x[i];
    double d0=(y[i]-y[i-1])/h0,d1=(y[i+1]-y[i])/h1;
    if (d0*d1<=0) return 0;
    double w0=2*h1+h0,w1=h1+2*h0;
    return (w0+w1)/(w0/d0+w1/d1);
}
static double interpolate(const double *x,const double *y,uint32_t n,double query) {
    if (query<=x[0]) return y[0];
    if (query>=x[n-1]) return y[n-1];
    uint32_t j=0;
    while (j+1<n && query>x[j+1]) ++j;
    double h=x[j+1]-x[j],t=(query-x[j])/h,t2=t*t,t3=t2*t;
    return (2*t3-3*t2+1)*y[j]+(t3-2*t2+t)*h*slope(x,y,n,j)+
        (-2*t3+3*t2)*y[j+1]+(t3-t2)*h*slope(x,y,n,j+1);
}
int steering_step(const SteeringConfig *c, SteeringState *s, double input,
                  double dt, const double *x,const double *y,uint32_t n,SteeringOutput *out) {
    if (steering_validate(c) || !s || !out || !isfinite(input) || !isfinite(dt) ||
        dt<=0 || !isfinite(s->center_angle) || !x || !y || n<2 || n>64) return -1;
    for (uint32_t i=0;i<n;++i) {
        if (!isfinite(x[i]) || !isfinite(y[i]) || fabs(y[i])>c->angle_limit+1e-7) return -1;
        if (i && (x[i]<=x[i-1] || y[i]<y[i-1])) return -1;
    }
    input=clip(input,-c->input_limit,c->input_limit);
    double target=clip(interpolate(x,y,n,input),-c->angle_limit,c->angle_limit);
    double alpha=c->time_constant>0 ? -expm1(-dt/c->time_constant):1;
    s->center_angle+=clip((target-s->center_angle)*alpha,-c->rate_limit*dt,c->rate_limit*dt);
    double curvature=tan(s->center_angle)/c->wheelbase;
    out->target_center=target;out->center=s->center_angle;
    out->left=atan2(c->wheelbase*curvature,1-c->track*curvature/2);
    out->right=atan2(c->wheelbase*curvature,1+c->track*curvature/2);
    return 0;
}
