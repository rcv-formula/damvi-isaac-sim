#!/usr/bin/env python3
"""Use the operator mux headlessly and ramp the initial autonomous speed in sim time."""
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from std_srvs.srv import SetBool
from damvi_teleop_ui.app import TeleopNode


class HeadlessOperator(TeleopNode):
    def __init__(self):
        super().__init__()
        self.launch_acceleration = self.declare_parameter('launch_acceleration_mps2', 1.5).value
        if not math.isfinite(self.launch_acceleration) or self.launch_acceleration < 0:
            raise ValueError('launch_acceleration_mps2 must be finite and nonnegative')
        self.launch_time_ns = None
        self.last_launch_command_ns = None

    def set_mode(self, mode):
        self.launch_time_ns = None
        self.last_launch_command_ns = None
        super().set_mode(mode)

    def receive(self, msg):
        # The planner/co-driver have already run while stopped. Release their
        # latest request gradually from the first valid nonzero autonomous input.
        if self.control.mode == 'auto' and self.launch_acceleration > 0:
            now = self.get_clock().now().nanoseconds
            if self.last_launch_command_ns is not None and now < self.last_launch_command_ns:
                self.set_mode('stop')
                return
            self.last_launch_command_ns = now
            requested = msg.linear.x
            if math.isfinite(requested) and requested != 0.:
                if now <= 0:
                    msg.linear.x = 0.
                else:
                    if self.launch_time_ns is None:
                        self.launch_time_ns = now
                    elapsed = (now - self.launch_time_ns) * 1e-9
                    ceiling = elapsed * self.launch_acceleration
                    msg.linear.x = math.copysign(min(abs(requested), ceiling), requested)
        super().receive(msg)


def main():
    rclpy.init()
    node = HeadlessOperator()

    def enable_autonomy(request, response):
        mode = 'auto' if request.data else 'stop'
        if mode != node.control.mode:
            node.set_mode(mode)
        response.success = True
        response.message = mode
        return response

    node.create_service(SetBool, '/validation/enable_autonomy', enable_autonomy)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        if rclpy.ok():
            raise
    finally:
        if rclpy.ok():
            node.set_mode('stop')
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
