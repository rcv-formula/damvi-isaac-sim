"""Physical invariants of the averaged electrical/mechanical model."""
import copy
import ctypes as C
from pathlib import Path
import sys
import unittest
import yaml
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from native import NativeController,DriveOutput,DriveState
CFG=Path(__file__).resolve().parents[1]/'config/vesc_sim.yaml'


class DrivetrainTests(unittest.TestCase):
    def setUp(self):
        self.config=yaml.safe_load(CFG.read_text())
        self.ctrl=NativeController(self.config)

    def call(self,omega,current=0,short=0,dt=.001):
        out=DriveOutput()
        code=self.ctrl.lib.drive_step(C.byref(self.ctrl.drive_config),C.byref(self.ctrl.drive),
            (C.c_double*4)(*omega),current,short,dt,C.byref(out))
        self.assertEqual(code,0)
        return out

    def test_wheel_speed_and_slip_are_not_chassis_speed(self):
        w=1.0/self.ctrl.drive_config.radius
        self.assertAlmostEqual(self.call([w]*4).erpm,1538.)
        self.assertAlmostEqual(self.call([-w]*4).erpm,-1538.)
        self.assertEqual(self.call([0]*4).erpm,0.)

    def test_passive_differentials_dissipate_and_conserve_torque(self):
        c=self.ctrl.drive_config;c.wheel_coulomb=0;c.wheel_viscous=0
        omega=[120.,30.,20.,40.]
        o=self.call(omega)
        self.assertAlmostEqual(sum(o.torque),0.)
        self.assertLess(sum(t*w for t,w in zip(o.torque,omega)),0.)

    def test_short_brake_opposes_both_directions_without_battery_charge(self):
        for sign in (-1,1):
            self.ctrl.reset();omega=[sign*30.]*4
            o=self.call(omega,short=1)
            self.assertLess(sum(t*w for t,w in zip(o.torque,omega)),0.)
            self.assertEqual(o.battery_current,0.)
            self.assertEqual(o.regen_power,0.)
        self.ctrl.reset()
        self.assertEqual(sum(self.call([0]*4,short=1).torque),0.)

    def test_regen_can_be_energy_positive_into_battery(self):
        c=self.ctrl.drive_config;c.current_time_constant=0
        o=self.call([100.]*4,current=-5.)
        self.assertGreater(o.regen_power,0.)
        self.assertLess(o.battery_current,0.)
        self.assertGreater(self.ctrl.drive.regen_joules,0.)

    def test_battery_current_limits_are_respected_in_steady_current_model(self):
        c=self.ctrl.drive_config;c.current_time_constant=0
        c.battery_regen_max=.1;c.battery_drive_max=.1
        for current in (-30.,30.):
            self.ctrl.reset()
            o=self.call([100.]*4,current=current)
            self.assertLessEqual(abs(o.battery_current),.10000001)

    def test_gain_has_one_owner_and_ignores_redundant_config_field(self):
        config=copy.deepcopy(self.config);config['speed_to_erpm_gain']=2000
        ctrl=NativeController(config)
        self.assertEqual(ctrl.drive_config.erpm_gain,2000)

if __name__=='__main__':unittest.main()
