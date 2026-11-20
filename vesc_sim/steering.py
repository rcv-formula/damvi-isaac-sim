"""Configuration and ctypes adapter for calibrated front-wheel steering."""
import ctypes as C
import math
from pathlib import Path
import yaml
from native import structure
from surface_config import load_surface_configuration

Config=structure('SteeringConfig','input_limit angle_limit wheelbase track time_constant rate_limit')
State=structure('SteeringState','center_angle')
Output=structure('SteeringOutput','target_center center left right')
DELAY_CAPACITY=1024


class DelayState(C.Structure):
    _fields_=[('clock',C.c_double),('clock_compensation',C.c_double),('timestamps',C.c_double*DELAY_CAPACITY),
              ('commands',C.c_double*DELAY_CAPACITY),('head',C.c_uint32),('count',C.c_uint32)]


def load_configuration(path, *, surface_dataset=None):
    path=Path(path);config=yaml.safe_load(path.read_text())
    steering_path=path.parent/config.get('steering_config','steering.yaml')
    config['_steering']=yaml.safe_load(steering_path.read_text())
    config['_steering_path']=str(steering_path)
    config['_lidar_path']=str(path.parent/config.get('lidar_config','lidar.yaml'))
    config['_imu_path']=str(path.parent/config.get('imu_config','imu.yaml'))
    config['_tf_path']=str(path.parent/config.get('tf_config','tf.yaml'))
    tires_path=path.parent/config.get('tires_config','tires.yaml')
    config['_tires']=yaml.safe_load(tires_path.read_text())
    config['_tires_path']=str(tires_path)
    surface_file=config.get('surface_friction_config')
    config['_surface']=None;config['_surface_path']=None
    if surface_file is not None:
        surface_path=path.parent/surface_file
        surface=load_surface_configuration(surface_path, surface_dataset)
        config['_surface']=surface;config['_surface_path']=str(surface_path)
        config['_tires']['surface_friction_coefficient']=surface['coefficient']
        config['_tires']['reference_surface_friction_coefficient']=surface['reference_coefficient']
    runtime_path=path.parent/config.get('runtime_config','runtime.yaml')
    config['_runtime']=yaml.safe_load(runtime_path.read_text())
    config['_runtime_path']=str(runtime_path)
    return config


class SteeringController:
    def __init__(self,config):
        self.config=config
        self.lib=C.CDLL(str(Path(__file__).parent/'build/libdamvi_steering.so'))
        self.transport_delay=float(config.get('actuator_transport_delay_s',0.))
        if not math.isfinite(self.transport_delay) or not 0<=self.transport_delay<=.5:
            raise ValueError('Steering transport delay must be in [0, 0.5] seconds')
        if config['model']!='monotone_pchip':raise ValueError('Expected calibrated nonlinear PCHIP steering map')
        self.params=Config(config['input_limit_rad'],config['center_angle_limit_rad'],config['simulation_wheelbase_m'],
            config['simulation_front_track_m'],config['actuator_time_constant_s'],config['actuator_rate_limit_rad_s'])
        self.lib.steering_validate.argtypes=[C.POINTER(Config)]
        x=config['input_knots_rad'];y=config['wheel_angle_knots_rad']
        if len(x)!=len(y) or not 2<=len(x)<=64:raise ValueError('Invalid steering knot count')
        self.count=len(x);self.x=(C.c_double*len(x))(*x);self.y=(C.c_double*len(y))(*y)
        self.lib.steering_step.argtypes=[C.POINTER(Config),C.POINTER(State),C.c_double,C.c_double,C.POINTER(C.c_double),C.POINTER(C.c_double),C.c_uint32,C.POINTER(Output)]
        self.lib.steering_delay_step.argtypes=[C.POINTER(DelayState),C.c_double,C.c_double,C.c_double,C.POINTER(C.c_double)]
        self.lib.steering_delay_step.restype=C.c_int
        if self.lib.steering_validate(C.byref(self.params)):
            raise ValueError('Invalid steering mapping, geometry or dynamics')
        self.reset()
        # Validate the static map directly: this tiny step must not populate a
        # transport queue or require a solver-sized validation dt.
        if self.lib.steering_step(C.byref(self.params),C.byref(self.state),0.,1e-9,
                self.x,self.y,self.count,C.byref(self.output)):
            raise ValueError('Invalid steering input or state')
        self.reset()

    def reset(self):
        self.state=State();self.output=Output();self.delay_state=DelayState()
        self._delayed_input=C.c_double()

    def step(self,command,dt):
        if not self.config.get('enabled',True):
            self.reset();return self.output
        result=self.lib.steering_delay_step(C.byref(self.delay_state),command,dt,
            self.transport_delay,C.byref(self._delayed_input))
        if result:
            reason='queue capacity exceeded' if result==-2 else 'invalid delay input, state or timestep'
            raise ValueError('Steering transport delay: '+reason)
        if self.lib.steering_step(C.byref(self.params),C.byref(self.state),self._delayed_input.value,dt,self.x,self.y,self.count,C.byref(self.output)):
            raise ValueError('Invalid steering input or state')
        return self.output
