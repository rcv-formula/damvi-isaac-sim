"""Small ctypes interface; all control/drivetrain arithmetic runs in C99."""
import ctypes as C
from pathlib import Path


def structure(name, names):
    return type(name, (C.Structure,), {'_fields_': [(x,C.c_double) for x in names.split()]})


SpeedConfig=structure('SpeedConfig','kp ki kd kd_filter min_erpm ramp_erpm_per_s allow_braking current_max current_max_scale erpm_min erpm_max speed_source')
SpeedState=structure('SpeedState','initialized target_erpm integral previous_error derivative_filtered')
SpeedOutput=structure('SpeedOutput','current_a target_erpm normalized_output low_speed_brake_requested output_saturated integral_saturated braking_suppressed')
DriveConfig=structure('DriveConfig','radius erpm_gain pole_pairs torque_constant phase_resistance current_time_constant current_drive_max current_brake_max battery_voltage battery_drive_max battery_regen_max gear_efficiency front_diff_damping rear_diff_damping center_stiffness center_damping coupling_torque_max wheel_coulomb wheel_viscous wheel_inertia')
DriveState=structure('DriveState','current center_twist regen_joules')


class DriveOutput(C.Structure):
    _fields_=[('torque',C.c_double*4)]+[(n,C.c_double) for n in 'erpm shaft_omega current battery_current regen_power center_speed_error center_torque electrical_brake'.split()]


class NativeController:
    def __init__(self,config):
        self.config=config
        self.lib=C.CDLL(str(Path(__file__).parent/'build/libdamvi_vesc.so'))
        self.lib.vesc_speed_validate.argtypes=[C.POINTER(SpeedConfig)]
        self.lib.drive_validate.argtypes=[C.POINTER(DriveConfig)]
        self.lib.vesc_speed_step.argtypes=[C.POINTER(SpeedConfig),C.POINTER(SpeedState),C.c_double,C.c_double,C.c_double,C.POINTER(SpeedOutput)]
        self.lib.drive_step.argtypes=[C.POINTER(DriveConfig),C.POINTER(DriveState),C.POINTER(C.c_double),C.c_double,C.c_double,C.c_double,C.POINTER(DriveOutput)]
        self.speed_config=SpeedConfig(**config['controller'])
        d=dict(config['drivetrain']);d['erpm_gain']=config['speed_to_erpm_gain']
        self.drive_config=DriveConfig(**d)
        if self.lib.vesc_speed_validate(C.byref(self.speed_config)) or self.lib.drive_validate(C.byref(self.drive_config)):
            raise ValueError('Invalid native configuration')
        self.reset()

    def reset(self):
        self.speed=SpeedState();self.drive=DriveState()
        self.speed_output=SpeedOutput();self.output=DriveOutput()

    def erpm(self,omega):
        return self.config['speed_to_erpm_gain']*self.drive_config.radius*sum(omega)/4

    def step(self,target_erpm,omega,dt,brake_current=None,coast=False):
        measured=self.erpm(omega)
        count=round(self.config['controller_hz']*dt)
        if count<1 or abs(count/self.config['controller_hz']-dt)>1e-9:
            raise ValueError('physics_hz must divide controller_hz')
        if coast:
            self.speed=SpeedState()
            current=0.;short=0.
        elif brake_current is None:
            for _ in range(count):
                if self.lib.vesc_speed_step(C.byref(self.speed_config),C.byref(self.speed),target_erpm,measured,dt/count,C.byref(self.speed_output)):
                    raise ValueError('Invalid PID state or input')
            current=self.speed_output.current_a
            short=self.speed_output.low_speed_brake_requested
        else:
            self.speed=SpeedState()
            current=-(1 if measured>0 else -1 if measured<0 else 0)*abs(brake_current)
            short=abs(measured)<self.speed_config.min_erpm and brake_current>0
        if self.lib.drive_step(C.byref(self.drive_config),C.byref(self.drive),(C.c_double*4)(*omega),current,short,dt,C.byref(self.output)):
            raise ValueError('Invalid drivetrain state or input')
        return self.output
