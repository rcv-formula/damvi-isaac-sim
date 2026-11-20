import unittest
from collections import deque
from types import SimpleNamespace
from ros_bridge import RosBridge


class LatestCommandTests(unittest.TestCase):
    def bridge(self, values):
        bridge=RosBridge.__new__(RosBridge)
        bridge.config={'ros':{'command_timeout_s':.5},'erpm_sign':1,'speed_to_erpm_gain':1538.}
        bridge.time=0.;bridge.received=None;bridge.target=0.;bridge.brake=None
        bridge.steering_received=None;bridge.steering_input=0.;bridge.callback_count=0
        pending=deque(values)
        def spin_once(timeout_sec):
            if pending:bridge.command(SimpleNamespace(data=pending.popleft()))
        bridge.executor=SimpleNamespace(spin_once=spin_once)
        return bridge

    def test_command_queue_drains_to_latest_in_one_gui_frame(self):
        bridge=self.bridge([100.,200.,300.,400.])
        self.assertEqual(bridge.spin(1.),(400.,None))
        self.assertEqual(bridge.callback_count,4)
        self.assertEqual(bridge.spin(1.6),(0.,None))

    def test_newest_invalid_command_stops_older_valid_input(self):
        bridge=self.bridge([100.,200.,float('nan')])
        self.assertEqual(bridge.spin(1.),(0.,None))


if __name__=='__main__':unittest.main()
