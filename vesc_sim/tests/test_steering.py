import copy,csv,math
from pathlib import Path
import sys,unittest
import yaml
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from steering import SteeringController


class SteeringTests(unittest.TestCase):
    def setUp(self):self.config=yaml.safe_load((ROOT/'config/steering.yaml').read_text())

    def test_c_interpolator_matches_independent_scipy_reference(self):
        config=copy.deepcopy(self.config);config['actuator_time_constant_s']=0;config['actuator_rate_limit_rad_s']=1000
        ctrl=SteeringController(config)
        with (ROOT/'calibration/steering/pchip_reference.csv').open() as f:
            for row in csv.DictReader(f):
                out=ctrl.step(float(row['input_rad']),1.)
                self.assertAlmostEqual(out.center,float(row['angle_rad']),places=12)

    def test_ackermann_wheels_share_one_turn_center(self):
        config=copy.deepcopy(self.config);config['actuator_time_constant_s']=0;config['actuator_rate_limit_rad_s']=1000
        ctrl=SteeringController(config);L=config['simulation_wheelbase_m'];T=config['simulation_front_track_m']
        for command in [-.8,-.4,0,.4,.8]:
            out=ctrl.step(command,1.)
            left=math.tan(out.left)/(L+T*math.tan(out.left)/2)
            right=math.tan(out.right)/(L-T*math.tan(out.right)/2)
            self.assertAlmostEqual(left,right,places=12)
            self.assertAlmostEqual(left,math.tan(out.center)/L,places=12)

    def test_actuator_rate_and_convergence(self):
        ctrl=SteeringController(self.config);last=0.
        for _ in range(2000):
            out=ctrl.step(.5,.001)
            self.assertLessEqual(abs(out.center-last),self.config['actuator_rate_limit_rad_s']*.001+1e-12)
            self.assertLessEqual(abs(out.center),self.config['center_angle_limit_rad']+1e-12)
            last=out.center
        self.assertAlmostEqual(out.center,out.target_center,places=8)

    def test_reversed_knots_rejected(self):
        config=copy.deepcopy(self.config);config['wheel_angle_knots_rad'][-1]=config['wheel_angle_knots_rad'][0]-.01
        with self.assertRaises(ValueError):SteeringController(config)

    def test_observed_servo_command_saturation(self):
        config=copy.deepcopy(self.config);config['actuator_time_constant_s']=0;config['actuator_rate_limit_rad_s']=1000
        ctrl=SteeringController(config);cal=config['calibration']
        gain=cal['servo_command_gain_observed'];offset=cal['servo_command_offset_observed']
        lo=(cal['servo_normalized_min']-offset)/gain
        hi=(cal['servo_normalized_max']-offset)/gain
        for edge,outer in [(lo,-.872664626),(hi,.872664626)]:
            expected=ctrl.step(edge,1.).target_center
            self.assertAlmostEqual(ctrl.step(outer,1.).target_center,expected,places=12)

    def test_disabled_outputs_zero_without_trim_offset(self):
        config=copy.deepcopy(self.config);config['enabled']=False
        out=SteeringController(config).step(.5,.01)
        self.assertEqual([out.center,out.left,out.right],[0.,0.,0.])

    def test_nonfinite_command_rejected(self):
        ctrl=SteeringController(self.config)
        with self.assertRaises(ValueError):ctrl.step(float('nan'),.001)


if __name__=='__main__':unittest.main()
