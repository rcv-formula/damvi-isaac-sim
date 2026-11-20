"""Mode logic independent of ROS and Qt; steering is an angle in radians."""
import math


class Controller:
    def __init__(self, speed=2.0, max_steering=0.6730769230769231, timeout=0.5):
        if not all(math.isfinite(x) for x in (speed, max_steering, timeout)):
            raise ValueError('Parameters must be finite')
        if speed < 0 or max_steering <= 0 or timeout < 0:
            raise ValueError('Invalid speed, steering limit or timeout')
        self.speed = speed
        self.max_steering = max_steering
        self.timeout = timeout
        self.mode = 'stop'
        self.keys = set()
        self.auto = None
        self.received_at = None

    def set_mode(self, mode):
        if mode not in ('stop', 'manual', 'auto'):
            raise ValueError(mode)
        self.mode = mode
        self.keys.clear()
        # A new autonomous session must receive a new command.
        self.auto = None
        self.received_at = None

    def receive(self, values, now):
        if self.mode != 'auto':
            return
        if not all(math.isfinite(x) for x in values):
            self.auto = None
            self.received_at = None
            return
        self.auto = tuple(values)
        self.received_at = now

    def output(self, now):
        if self.mode == 'manual':
            speed = self.speed * (int('up' in self.keys) - int('down' in self.keys))
            steer = self.max_steering * (int('left' in self.keys) - int('right' in self.keys))
            return (speed, 0.0, 0.0, 0.0, 0.0, steer), '방향키를 누르는 동안 출력'
        if self.mode == 'auto':
            if self.auto is None:
                status = '새 자율주행 입력 대기'
            elif self.timeout and now - self.received_at > self.timeout:
                status = '입력 시간 초과 → 0 출력'
            else:
                return self.auto, '자율주행 입력 전달 중'
        else:
            status = '정지 → 속도·조향 0 출력'
        return (0.0,) * 6, status
