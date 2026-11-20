"""Optional C99 tire force kernel. This module does not edit a USD or apply force.

The caller supplies measured, filtered ground-contact normal loads and physical
wheel states. It must replace native tire tangential friction, not overlay this
force on Coulomb friction, and apply each output force once at the wheel contact
point. The returned rolling reaction torque is diagnostic only.
"""
import ctypes as C
import math
from pathlib import Path

import yaml


def _structure(name, fields):
    return type(name, (C.Structure,), {'_fields_': [(key, C.c_double) for key in fields.split()]})


Config = _structure('DamviTireConfig', 'radius_m longitudinal_stiffness_n '
    'lateral_stiffness_n_per_rad reference_load_n stiffness_load_exponent '
    'friction_longitudinal friction_lateral slip_speed_floor_mps '
    'longitudinal_relaxation_length_m lateral_relaxation_length_m '
    'inverse_effective_mass_longitudinal_per_kg inverse_effective_mass_lateral_per_kg')
State = _structure('DamviTireState', 'longitudinal_force_n lateral_force_n')
Input = _structure('DamviTireInput', 'normal_load_n longitudinal_speed_mps '
    'lateral_contact_speed_mps wheel_angular_speed_rad_s')
Output = _structure('DamviTireOutput', 'longitudinal_force_n lateral_force_n '
    'longitudinal_slip_ratio lateral_slip_angle_rad dissipated_contact_power_w '
    'friction_ellipse_utilization reaction_torque_nm effective_normal_load_n')
WHEEL_NAMES = ('FL', 'FR', 'RL', 'RR')


class TireModel:
    """Four-wheel batch wrapper, with FL/FR/RL/RR order throughout.

    `step` takes four Input objects or dictionaries with the same fields. Speeds
    are in each tire's tangent frame. `longitudinal_speed_mps` is wheel-center
    speed along cross(axle,normal); `wheel_angular_speed_rad_s` is physical wheel
    angular velocity projected onto its axle. `lateral_contact_speed_mps` is the
    actual contact-point speed along the axle projected onto the tangent plane.
    It includes body roll contributions, not only wheel-center translation.

    Effective inverse masses are conservative absolute row-sum bounds on the
    coupled rigid-vehicle tangent response, including body translation/rotation
    and each freely rotating wheel's R²/I contribution. This accounts for four
    contacts acting simultaneously. It is not the isolated 75 g wheel mass.
    The parent can refresh actual contact positions and tangent directions with
    `set_contact_geometry`, or supply a more accurate bound directly through
    `set_effective_inverse_masses`.
    """
    def __init__(self, config, library_path=None):
        if isinstance(config, (str, Path)):
            config = yaml.safe_load(Path(config).read_text())
        self.config = config
        if config.get('model') != 'implicit_combined_slip':
            raise ValueError('Expected implicit_combined_slip tire model')
        self.enabled = bool(config.get('enabled', False))
        self.surface_coefficient = float(config.get('surface_friction_coefficient',
            config.get('reference_surface_friction_coefficient',1.)))
        self.surface_reference = float(config.get('reference_surface_friction_coefficient',1.))
        if not math.isfinite(self.surface_coefficient) or self.surface_coefficient<0 or \
                not math.isfinite(self.surface_reference) or self.surface_reference<=0:
            raise ValueError('Surface coefficient must be finite/nonnegative and its reference positive')
        self.surface_scale = self.surface_coefficient/self.surface_reference
        self.lib = C.CDLL(str(library_path or Path(__file__).parent/'build/libdamvi_tires.so'))
        self.lib.damvi_tire_validate.argtypes = [C.POINTER(Config)]
        self.lib.damvi_tire_step_four.argtypes = [C.POINTER(Config), C.POINTER(State),
                                                C.POINTER(Input), C.c_double, C.POINTER(Output)]
        self.lib.damvi_tire_validate.restype = C.c_int
        self.lib.damvi_tire_step_four.restype = C.c_int
        mass = float(config['vehicle_mass_kg'])
        iz = float(config['vehicle_yaw_inertia_kg_m2'])
        radius = float(config['wheel_radius_m'])
        wheel_inertia = float(config['wheel_axial_inertia_kg_m2'])
        if not all(math.isfinite(x) and x > 0 for x in (mass, iz, radius, wheel_inertia)):
            raise ValueError('Vehicle mass, yaw inertia, wheel radius/inertia must be positive')
        self.params = (Config*4)()
        nominal_positions = []
        for i, name in enumerate(WHEEL_NAMES):
            front = name[0] == 'F'
            axle = config['front' if front else 'rear']
            x = float(config['front_axle_x_m' if front else 'rear_axle_x_m'])
            track = float(config['front_track_m' if front else 'rear_track_m'])
            if not math.isfinite(x) or not math.isfinite(track) or track <= 0:
                raise ValueError('Axle positions and track widths must be finite')
            # At zero steer: longitudinal yaw lever is half-track; lateral yaw
            # lever is axle x. Update these with actual steering each physics tick
            # when high steering accuracy is needed; see set_effective_inverse_masses.
            self.params[i] = Config(radius,
                float(axle['longitudinal_stiffness_n']),
                float(axle['lateral_stiffness_n_per_rad']),
                float(axle['reference_normal_load_n']),
                float(config['stiffness_load_exponent']),
                float(axle['friction_longitudinal'])*self.surface_scale,
                float(axle['friction_lateral'])*self.surface_scale,
                float(config['slip_speed_floor_mps']),
                float(config['longitudinal_relaxation_length_m']),
                float(config['lateral_relaxation_length_m']),
                1./mass + (.5*track)**2/iz + radius*radius/wheel_inertia,
                1./mass + x*x/iz)
            if self.lib.damvi_tire_validate(C.byref(self.params[i])):
                raise ValueError(f'Invalid tire parameters for {name}')
            nominal_positions.append((x, (.5 if name[1] == 'L' else -.5)*track,
                                      float(config.get('wheel_contact_z_m', -.07))))
        self.set_contact_geometry(nominal_positions, [(1., 0., 0.)]*4,
                                  [(0., 1., 0.)]*4)
        self.reset()

    def reset(self):
        self.state = (State*4)()
        self.output = (Output*4)()

    def set_effective_inverse_masses(self, longitudinal, lateral):
        if len(longitudinal) != 4 or len(lateral) != 4:
            raise ValueError('Expected four effective inverse masses on each axis')
        values = tuple(longitudinal)+tuple(lateral)
        if not all(math.isfinite(float(v)) and float(v) > 0 for v in values):
            raise ValueError('Effective inverse masses must be finite and positive')
        for i in range(4):
            self.params[i].inverse_effective_mass_longitudinal_per_kg = float(longitudinal[i])
            self.params[i].inverse_effective_mass_lateral_per_kg = float(lateral[i])

    def set_contact_geometry(self, positions_body, longitudinal_body, lateral_body):
        """Update a conservative force-response bound from body-frame vectors.

        Positions are contact positions relative to the aggregate vehicle CG.
        Directions must be unit tangent vectors. For the 8 tangent force axes,
        A_ij = di·dj/M + (ri×di)·I^-1(rj×dj), plus R²/Iwheel on each rolling
        axis. Absolute row sums D make D-A positive semidefinite. Combined with
        the kernel impulse cap this prevents an independent four-wheel update
        from injecting energy into this rigid-body approximation.

        A contact should be disabled by measured zero load; retaining its axis
        in this bound is conservative. The diagonal inertia must be expressed
        about the aggregate CG in this same body frame.
        """
        groups = (positions_body, longitudinal_body, lateral_body)
        if any(len(group) != 4 for group in groups):
            raise ValueError('Expected four contact positions and tangent directions')
        if not all(len(v) == 3 and all(math.isfinite(float(x)) for x in v)
                   for group in groups for v in group):
            raise ValueError('Contact geometry must contain finite 3D vectors')
        for group in (longitudinal_body, lateral_body):
            for d in group:
                if abs(sum(float(v)**2 for v in d)-1.) > 1e-4:
                    raise ValueError('Tangent directions must be unit vectors')
        inertia = self.config.get('vehicle_inertia_diagonal_kg_m2',
                                   [self.config['vehicle_yaw_inertia_kg_m2']]*3)
        if len(inertia) != 3 or not all(math.isfinite(float(v)) and float(v) > 0 for v in inertia):
            raise ValueError('Body diagonal inertia must be finite and positive')
        mass = float(self.config['vehicle_mass_kg'])
        def cross(a, b):
            return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])
        directions, levers = [], []
        for i in range(4):
            for group in (longitudinal_body, lateral_body):
                directions.append(group[i])
                levers.append(cross(positions_body[i], group[i]))
        inverse = []
        for i in range(8):
            value = 0.
            for j in range(8):
                response = sum(float(directions[i][k])*float(directions[j][k])/mass +
                    levers[i][k]*levers[j][k]/float(inertia[k]) for k in range(3))
                value += abs(response)
            if i % 2 == 0:
                value += float(self.config['wheel_radius_m'])**2/float(self.config['wheel_axial_inertia_kg_m2'])
            inverse.append(value)
        self.set_effective_inverse_masses(inverse[::2], inverse[1::2])

    def step(self, inputs, dt):
        if len(inputs) != 4:
            raise ValueError('Expected FL/FR/RL/RR tire inputs')
        native_inputs = (Input*4)(*(row if isinstance(row, Input) else Input(**row)
                                   for row in inputs))
        if self.lib.damvi_tire_step_four(self.params, self.state, native_inputs,
                                       float(dt), self.output):
            raise ValueError('Invalid tire input, state or time step')
        return self.output

    def status(self):
        return {'enabled': self.enabled, 'model': self.config['model'],
                'surface_friction_coefficient': self.surface_coefficient,
                'surface_friction_reference': self.surface_reference,
                'surface_friction_scale': self.surface_scale,
                'calibration_status': self.config.get('calibration_status', 'unknown'),
                'wheel_order': list(WHEEL_NAMES),
                'latest': [{key: getattr(row, key) for key, _ in Output._fields_}
                           for row in self.output]}
