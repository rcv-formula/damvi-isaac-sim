"""Extract only motor configuration; never change the source Windows XML."""
import argparse
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET


def import_config(path):
    root = ET.parse(path).getroot()
    def number(tag, default=None):
        text = root.findtext(tag)
        if text is None and default is None:
            raise ValueError(f'Missing motor parameter {tag}')
        return float(text) if text is not None else default
    cfg = {
        'provenance': {'motor_xml': str(Path(path).resolve()),
                       'sha256': hashlib.sha256(Path(path).read_bytes()).hexdigest(),
                       'firmware_version': 'not encoded in XML; 6.05-compatible speed loop',
                       'calibration': 'PID/current/electrical from user-selected XML; drivetrain friction/compliance uncalibrated'},
        'speed_to_erpm_gain': 1538.0,
        'steering_config': 'steering.yaml',
        'lidar_config': 'lidar.yaml',
        'imu_config': 'imu.yaml',
        'tf_config': 'tf.yaml',
        'runtime_config': 'runtime.yaml',
        'speed_to_erpm_offset': 0.0,
        'erpm_sign': 1,
        'controller_hz': {0:25,1:50,2:100,3:250,4:500,5:1000,6:2500,7:5000,8:10000}[int(number('sp_pid_loop_rate',5))],
        'physics_hz': 200,
        'controller': {
            'kp':number('s_pid_kp'), 'ki':number('s_pid_ki'),
            'kd':number('s_pid_kd'), 'kd_filter':number('s_pid_kd_filter'),
            'min_erpm':number('s_pid_min_erpm'),
            'ramp_erpm_per_s':number('s_pid_ramp_erpms_s'),
            'allow_braking':number('s_pid_allow_braking'),
            'current_max':number('l_current_max'),
            'current_max_scale':number('l_current_max_scale',1),
            'erpm_min':number('l_min_erpm'), 'erpm_max':number('l_max_erpm'),
            'speed_source':number('s_pid_speed_source',0)},
        'drivetrain': {
            'radius':0.02794,
            'pole_pairs':number('si_motor_poles')/2,
            'torque_constant':1.5*(number('si_motor_poles')/2)*number('foc_motor_flux_linkage'),
            'phase_resistance':number('foc_motor_r'),
            'current_time_constant':number('foc_motor_l')/number('foc_motor_r'),
            'current_drive_max':number('l_current_max')*number('l_current_max_scale',1),
            'current_brake_max':abs(number('l_current_min')*number('l_current_min_scale',1)),
            'battery_voltage':12.0,
            'battery_drive_max':number('l_in_current_max'),
            'battery_regen_max':abs(number('l_in_current_min')),
            'gear_efficiency':0.9,
            'front_diff_damping':0.002,'rear_diff_damping':0.002,
            'center_stiffness':0.5,'center_damping':0.02,'coupling_torque_max':2.0,
            'wheel_coulomb':0.0002,'wheel_viscous':0.00001,
            'wheel_inertia':0.5*0.075*0.02794**2},
        'xml_geometry_reference': {
            'wheel_diameter_m':number('si_wheel_diameter'),
            'gear_ratio':number('si_gear_ratio'),
            'motor_invert_direction':number('m_invert_direction'),
            'note':'USD wheel diameter is 0.05588 m. Effective gearing is derived from gain and USD radius; XML gearing is not multiplied again. Invert_direction is hardware orientation, not a negative ROS forward speed.'},
        'ros': {'command_topic':'/commands/motor/speed', 'brake_topic':'/commands/motor/brake',
                'output_topic':'/sensors/core', 'publish_rate_hz':50.0,
                'command_timeout_s':0.5, 'publish_when_stopped':False,
                'stop_threshold_erpm':1.0},
        'scene': {'vehicle_root':'/World/RC_Car','chassis':'/World/RC_Car/Chassis',
                  'rear_axle_offset_m':[-0.1,0.0,-0.04206]},
    }
    return cfg


if __name__ == '__main__':
    import yaml
    p=argparse.ArgumentParser();p.add_argument('xml',type=Path);p.add_argument('-o','--output',type=Path,required=True)
    a=p.parse_args();a.output.write_text(yaml.safe_dump(import_config(a.xml),sort_keys=False,allow_unicode=True))
    print(a.output)
