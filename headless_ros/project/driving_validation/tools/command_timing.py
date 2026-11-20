"""Read-only steering pipeline timestamps from the observer's wall clock and ROS headers."""
import time

from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist
from rosgraph_msgs.msg import Clock


class CommandTiming:
    def __init__(self, node, qos):
        self.start_wall = time.monotonic()
        self.traces = {name: [] for name in ('clock', 'pf', 'gt', 'pp', 'co_driver', 'support', 'mux')}
        node.create_subscription(Clock, '/clock', self.on_clock, qos)
        node.create_subscription(AckermannDriveStamped, '/drive_main', self.on_pp, qos)
        node.create_subscription(AckermannDriveStamped, '/drive_autonomy', self.on_co_driver, qos)
        node.create_subscription(Twist, '/autonomy/cmd_vel', self.on_support, qos)

    def row(self, name, stamp=None, *fields):
        ros_s = None if stamp is None else stamp.sec + 1e-9 * stamp.nanosec
        self.traces[name].append([time.monotonic()-self.start_wall, ros_s, *fields])

    def on_clock(self, m):
        self.row('clock', m.clock)

    def on_pp(self, m):
        self.row('pp', m.header.stamp, m.drive.speed, m.drive.steering_angle)

    def on_co_driver(self, m):
        self.row('co_driver', m.header.stamp, m.drive.speed, m.drive.steering_angle)

    def on_support(self, m):
        self.row('support', None, m.linear.x, m.angular.z)
