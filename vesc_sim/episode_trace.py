"""Read-only, bounded physics samples for validating ROS-to-wheel response."""
from collections import deque
import math


COLUMNS = ['t', 'chassis_x', 'chassis_y', 'yaw', 'ground_speed_mps', 'received_steer',
           'servo_target', 'steer_left', 'steer_right', 'yaw_rate', 'target_erpm',
           'wheel_erpm', 'motor_current_a']


def encode_row(row):
    qx, qy, qz, qw = row['orientation_xyzw']
    yaw = math.atan2(2*(qw*qz+qx*qy), 1-2*(qy*qy+qz*qz))
    return [row['time'], *row['position'][:2], yaw, math.hypot(*row['velocity'][:2]),
            row['steering_input_rad'], row['steering_center_target_rad'],
            *row['steering_actual_left_right_rad'], row['angular_velocity_rad_s'][2],
            row['target_erpm'], row['erpm'], row['current_a']]


class EpisodeTrace:
    def __init__(self, controller):
        import omni.kit.app
        self.controller = controller
        self.before = deque(maxlen=15)
        self.rows = deque(maxlen=20000)  # 400 simulated seconds at 50 Hz
        self.last_t = -1.
        self.started = False
        self.command_receipts = deque(maxlen=20000)
        controller.bridge.validation_command_receipts=self.command_receipts
        events = omni.kit.app.get_app().get_update_event_stream()
        self.subscription = events.create_subscription_to_pop(self.on_frame,name='damvi_validation_physics_trace')

    def close(self):
        self.subscription = None
        if self.controller.bridge is not None:
            self.controller.bridge.validation_command_receipts=None

    def on_frame(self, event=None):
        recent = []
        for row in reversed(self.controller.rows):
            if row['time'] <= self.last_t:
                break
            recent.append(row)
        for row in reversed(recent):
            self.last_t = row['time']
            if not self.started and abs(row['target_erpm']) > 1:
                self.started = True
                self.rows.extend(self.before)
                self.before.clear()
            target = self.rows if self.started else self.before
            target.append(encode_row(row))
