"""Behavioral checks for the firmware speed-controller subset (stdlib only)."""
import ctypes as ct
import math
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class Config(ct.Structure):
    _fields_ = [(name, ct.c_double) for name in (
        "kp ki kd kd_filter min_erpm ramp_erpm_per_s allow_braking "
        "current_max current_max_scale erpm_min erpm_max speed_source").split()]


class State(ct.Structure):
    _fields_ = [(name, ct.c_double) for name in
                "initialized target_erpm integral previous_error derivative_filtered".split()]


class Output(ct.Structure):
    _fields_ = [(name, ct.c_double) for name in (
        "current_a target_erpm normalized_output low_speed_brake_requested "
        "output_saturated integral_saturated braking_suppressed").split()]


class NativeSpeedTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        compiler = shutil.which("cc")
        if not compiler:
            raise unittest.SkipTest("C compiler unavailable")
        cls.tmp = tempfile.TemporaryDirectory()
        library = Path(cls.tmp.name) / "libspeed.so"
        subprocess.run([compiler, "-std=c99", "-Wall", "-Wextra", "-Werror",
                        "-shared", "-fPIC", str(ROOT / "native/vesc_speed.c"),
                        "-lm", "-o", str(library)], check=True)
        cls.lib = ct.CDLL(str(library))
        cls.lib.vesc_speed_defaults.argtypes = [ct.POINTER(Config)]
        cls.lib.vesc_speed_defaults.restype = None
        cls.lib.vesc_speed_reset.argtypes = [ct.POINTER(State)]
        cls.lib.vesc_speed_reset.restype = None
        cls.lib.vesc_speed_step.argtypes = [ct.POINTER(Config), ct.POINTER(State),
            ct.c_double, ct.c_double, ct.c_double, ct.POINTER(Output)]
        cls.lib.vesc_speed_step.restype = ct.c_int

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def setUp(self):
        self.config, self.state, self.out = Config(), State(), Output()
        self.lib.vesc_speed_defaults(ct.byref(self.config))

    def step(self, target, measured, dt=.001):
        rc = self.lib.vesc_speed_step(ct.byref(self.config), ct.byref(self.state),
                                    target, measured, dt, ct.byref(self.out))
        self.assertEqual(rc, 0)
        return self.out

    def test_signed_ramp_seeds_measured_and_limits_deceleration(self):
        self.config.min_erpm = 0
        self.config.ramp_erpm_per_s = 1000
        self.assertEqual(self.step(2000, 1000, .1).target_erpm, 1100)
        self.assertEqual(self.step(-2000, 1000, .1).target_erpm, 1000)
        self.assertEqual(self.step(-2000, 1000, .1).target_erpm, 900)

    def test_pid_numeric_reference_and_old_integral_order(self):
        self.config.ramp_erpm_per_s = 0
        # e=1000; P=.2, D_raw=5, filtered D=1, old I=0 => clamp 1.
        first = self.step(2000, 1000)
        self.assertEqual(first.normalized_output, 1)
        self.assertEqual(first.output_saturated, 1)
        self.assertAlmostEqual(self.state.integral, .0002)
        # D decays to .8; P=.2; output includes old I=.0002, still clamped.
        self.step(2000, 1000)
        self.assertAlmostEqual(self.state.derivative_filtered, .8)
        self.assertAlmostEqual(self.state.integral, .0004)
        self.config.kd = 0
        self.state.derivative_filtered = 0
        self.assertAlmostEqual(self.step(2000, 1000).current_a, .2004 * 60)

    def test_braking_and_reverse(self):
        self.config.ramp_erpm_per_s = 0
        self.config.kd = self.config.ki = 0
        self.assertLess(self.step(1000, 2000).current_a, 0)
        self.lib.vesc_speed_reset(ct.byref(self.state))
        self.assertGreater(self.step(-1000, -2000).current_a, 0)
        self.lib.vesc_speed_reset(ct.byref(self.state))
        self.assertLess(self.step(-2000, 0).current_a, 0)
        self.config.allow_braking = 0
        self.lib.vesc_speed_reset(ct.byref(self.state))
        out = self.step(1000, 2000)
        self.assertEqual(out.current_a, 0)
        self.assertEqual(out.braking_suppressed, 1)

    def test_low_speed_requests_adapter_brake_and_resets_integral(self):
        self.config.ramp_erpm_per_s = 0
        self.state.integral = .7
        out = self.step(0, 2000)
        self.assertEqual(out.current_a, 0)
        self.assertEqual(out.low_speed_brake_requested, 1)
        self.assertEqual(self.state.integral, 0)
        # min threshold is strict: the boundary is a PID-controlled target.
        self.assertEqual(self.step(900, 900).low_speed_brake_requested, 0)

    def test_saturation_and_direction_symmetry(self):
        self.config.min_erpm = 0
        self.config.ramp_erpm_per_s = 0
        self.config.ki = 100
        positive = self.step(10000, 0, 1).current_a
        self.assertEqual(self.out.integral_saturated, 1)
        self.lib.vesc_speed_reset(ct.byref(self.state))
        negative = self.step(-10000, 0, 1).current_a
        self.assertEqual(positive, -negative)

    def test_selected_vehicle_zero_d_filter_is_not_unfiltered(self):
        self.config.kp, self.config.ki, self.config.kd = .009, 0, 1.5e-5
        self.config.kd_filter = 0
        self.config.min_erpm = 800
        self.config.ramp_erpm_per_s = 50000
        self.config.current_max = 62
        # The selected export requests 5000 Hz. Rate step is 10 ERPM.
        first = self.step(3000, 1000, 1 / 5000)
        self.assertEqual(first.target_erpm, 1010)
        self.assertEqual(self.state.derivative_filtered, 0)
        self.assertAlmostEqual(first.current_a, 10 * .009 / 20 * 62)
        self.assertEqual(self.state.integral, 0)

    def test_invalid_is_atomic(self):
        self.step(2000, 1000)
        before_state, before_out = bytes(self.state), bytes(self.out)
        for target, measured, dt in [(math.nan, 0, .01), (0, math.inf, .01),
                                      (0, 0, 0), (0, 0, -1), (0, 0, math.inf)]:
            rc = self.lib.vesc_speed_step(ct.byref(self.config), ct.byref(self.state),
                target, measured, dt, ct.byref(self.out))
            self.assertEqual(rc, -1)
            self.assertEqual(bytes(self.state), before_state)
            self.assertEqual(bytes(self.out), before_out)
        self.config.kd_filter = 1.1
        self.assertEqual(self.lib.vesc_speed_step(ct.byref(self.config),
            ct.byref(self.state), 0, 0, .01, ct.byref(self.out)), -1)


if __name__ == "__main__":
    unittest.main()
