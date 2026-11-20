"""Circle-following command generator for an isolated plant stability test.

Feedback is the *measured rear-axle pose*, supplied by the caller. This module
does not import Isaac or ROS, write state, or claim full-stack localization
validation. Commands go through the normal steering actuator and tire physics.
"""

import math


def _clip(value, low, high):
    return max(low, min(high, value))


class CircleController:
    """Match the configured production PP geometry/filter for a circular path.

    ``step(t_s, (rear_x, rear_y, yaw_rad), speed_target_mps)`` returns a nominal
    steering command. It updates at 50 Hz and holds the last output between
    updates. ``direction=1`` travels counterclockwise; ``-1`` clockwise. The
    caller owns speed control, chassis-to-rear-axle conversion and physics steps.
    """

    PERIOD_S = .02
    K_P = .25
    PHYSICAL_STEERING_LIMIT_RAD = .26

    def __init__(self, steering_config, radius_m=20., center_xy=(0., 0.), direction=1,
                 path_tangent_gain=0., understeer_rad_per_mps2=0., pose_preview_s=0.,
                 min_lookahead=.6, max_lookahead=1.2, lookahead_ratio=2.):
        self.radius_m = float(radius_m)
        self.center_xy = tuple(float(v) for v in center_xy)
        if (not math.isfinite(self.radius_m) or self.radius_m <= 0 or
                len(self.center_xy) != 2 or
                not all(math.isfinite(v) for v in self.center_xy) or
                direction not in (-1, 1)):
            raise ValueError('Invalid circle geometry or direction')
        self.direction = int(direction)
        self.pose_preview_s = float(pose_preview_s)
        self.min_lookahead = float(min_lookahead)
        self.max_lookahead = float(max_lookahead)
        self.lookahead_ratio = float(lookahead_ratio)
        self.last_pose_time = None
        self.last_pose_yaw = None
        self.filtered_yaw_rate = 0.
        self.path_tangent_gain = float(path_tangent_gain)
        self.understeer_rad_per_mps2 = float(understeer_rad_per_mps2)
        x = tuple(float(v) for v in steering_config['input_knots_rad'])
        y = tuple(float(v) for v in steering_config['wheel_angle_knots_rad'])
        self.input_limit = float(steering_config['input_limit_rad'])
        if (steering_config.get('model', 'monotone_pchip') != 'monotone_pchip' or
                not steering_config.get('enabled', True) or len(x) != 2 or len(y) != 2 or
                not all(math.isfinite(v) for v in (*x, *y, self.input_limit)) or
                x[1] <= x[0] or y[1] <= y[0] or self.input_limit <= 0):
            raise ValueError('Circle test requires the current increasing two-knot steering map')
        self.gain = (y[1]-y[0])/(x[1]-x[0])
        self.bias = y[0]-self.gain*x[0]
        self._last_t = None
        self._last_update_t = None
        self._physical_output = 0.
        self._nominal_output = 0.
        self._status = {'isolated_gt_feedback_test': True, 'initialized': False}

    def cross_track_error(self, rear_axle_pose):
        """Signed radial error in meters: positive outside the reference circle."""
        x, y = float(rear_axle_pose[0]), float(rear_axle_pose[1])
        if not math.isfinite(x) or not math.isfinite(y):
            raise ValueError('Rear-axle position must be finite')
        return math.hypot(x-self.center_xy[0], y-self.center_xy[1])-self.radius_m

    def step(self, t_s, rear_axle_pose, speed_target_mps, wheel_speed_mps=0.):
        t_s, speed_target_mps = float(t_s), float(speed_target_mps)
        x, y, yaw = (float(v) for v in rear_axle_pose)
        if (not all(math.isfinite(v) for v in (t_s, speed_target_mps, x, y, yaw)) or
                t_s < 0 or speed_target_mps < 0):
            raise ValueError('Circle test needs finite time/pose and nonnegative forward speed')
        if self._last_t is not None and t_s < self._last_t-1e-12:
            raise ValueError('Circle controller time went backwards; construct a new controller')
        self._last_t = t_s
        radial_x, radial_y = x-self.center_xy[0], y-self.center_xy[1]
        radius = math.hypot(radial_x, radial_y)
        if radius < 1e-9:
            raise ValueError('Nearest circle projection is undefined at the circle center')
        self._status.update(measured_time_s=t_s, cross_track_error_m=radius-self.radius_m,
                            rear_axle_pose=[x, y, yaw], speed_target_mps=speed_target_mps)
        # PF publishes localized pose and finite-difference yaw twist at 100 Hz.
        # Use the instantaneous measured pose with zero transport age in this fixture.
        if self.pose_preview_s > 0. and (self.last_pose_time is None or t_s-self.last_pose_time >= .01-1e-12):
            if self.last_pose_time is not None:
                dt = t_s - self.last_pose_time
                yaw_rate = math.remainder(yaw - self.last_pose_yaw, 2.*math.pi) / dt
                if dt > .2:
                    self.filtered_yaw_rate = 0.
                elif abs(yaw_rate) < 7.:
                    self.filtered_yaw_rate += -math.expm1(-dt/.045)*(yaw_rate-self.filtered_yaw_rate)
            self.last_pose_time = t_s
            self.last_pose_yaw = yaw
        if self._last_update_t is not None and t_s-self._last_update_t < self.PERIOD_S-1e-12:
            return self._nominal_output

        if self.pose_preview_s > 0.:
            horizon = min(.35, self.pose_preview_s)
            half_turn = .5*horizon*self.filtered_yaw_rate
            sinc = 1. if abs(half_turn) < 1e-6 else math.sin(half_turn)/half_turn
            travel = wheel_speed_mps * horizon * sinc
            x += travel*math.cos(yaw + half_turn)
            y += travel*math.sin(yaw + half_turn)
            yaw = math.remainder(yaw + 2.*half_turn, 2.*math.pi)
            radial_x, radial_y = x-self.center_xy[0], y-self.center_xy[1]
        lookahead = _clip(self.max_lookahead*speed_target_mps/self.lookahead_ratio,
                          self.min_lookahead, self.max_lookahead)
        theta = math.atan2(radial_y, radial_x)
        target_theta = theta+self.direction*lookahead/self.radius_m
        target_x = self.center_xy[0]+self.radius_m*math.cos(target_theta)
        target_y = self.center_xy[1]+self.radius_m*math.sin(target_theta)
        dx, dy = target_x-x, target_y-y
        cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
        local_x = cos_yaw*dx+sin_yaw*dy
        local_y = -sin_yaw*dx+cos_yaw*dy
        distance_squared = dx*dx+dy*dy
        curvature = 2.*local_y/max(distance_squared, 1e-12)
        path_heading = theta + self.direction*math.pi/2.
        path_curvature = self.direction/self.radius_m
        correction = (self.path_tangent_gain*math.sin(path_heading-yaw) +
                      self.understeer_rad_per_mps2*wheel_speed_mps**2*path_curvature)
        desired = _clip(self.K_P*curvature + correction,
                        -self.PHYSICAL_STEERING_LIMIT_RAD, self.PHYSICAL_STEERING_LIMIT_RAD)
        if self._last_update_t is None:
            alpha = 1.  # Production PP seeds its first output from the target.
            self._physical_output = desired
        else:
            change = abs(desired-self._physical_output)
            alpha = .6+(.8-.6)*_clip(change/math.radians(10.), 0., 1.)
            self._physical_output += alpha*(desired-self._physical_output)
        self._physical_output = _clip(self._physical_output,
            -self.PHYSICAL_STEERING_LIMIT_RAD, self.PHYSICAL_STEERING_LIMIT_RAD)
        self._nominal_output = _clip((self._physical_output-self.bias)/self.gain,
                                     -self.input_limit, self.input_limit)
        self._last_update_t = t_s
        self._status.update(initialized=True, control_time_s=t_s,
            lookahead_m=lookahead, target_world_xy=[target_x, target_y],
            target_local_xy=[local_x, local_y], curvature_per_m=curvature,
            physical_target_rad=desired, physical_output_rad=self._physical_output,
            nominal_steering_rad=self._nominal_output, latest_blend=alpha)
        return self._nominal_output

    def status(self):
        return dict(self._status)
