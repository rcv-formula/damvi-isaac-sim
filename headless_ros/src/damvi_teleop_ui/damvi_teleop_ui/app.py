import math
import signal
import sys
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist
from PyQt5 import QtCore, QtGui, QtWidgets

from .control import Controller


class TeleopNode(Node):
    def __init__(self):
        super().__init__('damvi_teleop_ui')
        defaults = dict(input_topic='/cmd_vel', output_topic='/ackermann_drive',
                        output_type='ackermann', manual_speed=2.0,
                        max_steering_angle=0.6730769230769231, publish_rate=50.0,
                        auto_timeout=0.5, frame_id='base_link')
        params = {k: self.declare_parameter(k, v).value for k, v in defaults.items()}
        self.control = Controller(params['manual_speed'], params['max_steering_angle'],
                                  params['auto_timeout'])
        rate = params['publish_rate']
        if not math.isfinite(rate) or not 1 <= rate <= 200:
            raise ValueError('publish_rate must be between 1 and 200 Hz')
        self.output_type = params['output_type']
        if self.output_type not in ('twist', 'ackermann'):
            raise ValueError('output_type must be twist or ackermann')
        if self.resolve_topic_name(params['input_topic']) == self.resolve_topic_name(params['output_topic']):
            raise ValueError('Input and output topics must differ (feedback loop)')
        self.frame_id = params['frame_id']
        message_type = Twist if self.output_type == 'twist' else AckermannDriveStamped
        self.publisher = self.create_publisher(message_type, params['output_topic'], 1)
        # Compatible with both best-effort simulation and reliable controllers.
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.subscription = self.create_subscription(Twist, params['input_topic'], self.receive, qos)
        self.last_values = (0.0,) * 6
        self.status = ''
        # A steady clock is essential when a simulator pauses /clock.
        from rclpy.clock import Clock, ClockType
        self.timer = self.create_timer(1.0 / rate, self.publish_command,
                                      clock=Clock(clock_type=ClockType.STEADY_TIME))

    def receive(self, msg):
        self.control.receive((msg.linear.x, msg.linear.y, msg.linear.z,
                              msg.angular.x, msg.angular.y, msg.angular.z), time.monotonic())

    def publish_command(self):
        self.last_values, self.status = self.control.output(time.monotonic())
        v = self.last_values
        if self.output_type == 'twist':
            msg = Twist()
            msg.linear.x, msg.linear.y, msg.linear.z = v[:3]
            msg.angular.x, msg.angular.y, msg.angular.z = v[3:]
        else:
            msg = AckermannDriveStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id
            msg.drive.speed, msg.drive.steering_angle = v[0], v[5]
        self.publisher.publish(msg)

    def set_mode(self, mode):
        self.control.set_mode(mode)
        self.publish_command()


class HistoryPlot(QtWidgets.QWidget):
    def __init__(self, steering_limit):
        super().__init__()
        self.samples = deque(maxlen=400)
        self.steering_limit = steering_limit
        self.setMinimumSize(460, 170)

    def add(self, speed, steering):
        self.samples.append((time.monotonic(), speed, steering))
        self.update()

    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        painter.fillRect(self.rect(), QtGui.QColor('#ffffff'))
        now = time.monotonic()
        for index, title, minimum in (
                (1, '속도 (m/s)', 2.0),
                (2, '조향 (rad)', self.steering_limit)):
            top = 6 + (index - 1) * self.height() / 2
            area = QtCore.QRectF(105, top + 14, self.width() - 120, self.height() / 2 - 29)
            values = [abs(s[index]) for s in self.samples if now - s[0] <= 10]
            scale = max([minimum] + values)
            painter.setPen(QtGui.QColor('#cccccc'))
            painter.drawLine(QtCore.QPointF(area.left(), area.center().y()),
                             QtCore.QPointF(area.right(), area.center().y()))
            painter.setPen(QtGui.QColor('#000000'))
            painter.drawText(8, int(top + 23), title)
            painter.drawText(8, int(top + 43), f'±{scale:.2f}')
            path = QtGui.QPainterPath()
            first = True
            for sample in self.samples:
                age = now - sample[0]
                if age > 10:
                    continue
                x = area.right() - age / 10 * area.width()
                y = area.center().y() - sample[index] / scale * area.height() / 2
                if first:
                    path.moveTo(x, y)
                    first = False
                else:
                    path.lineTo(x, y)
            painter.setPen(QtGui.QPen(QtGui.QColor('#000000'), 2))
            painter.drawPath(path)
        painter.setPen(QtGui.QColor('#666666'))
        painter.drawText(self.width() - 130, self.height() - 3, '최근 10초 → 현재')


class TeleopWindow(QtWidgets.QWidget):
    KEY_NAMES = {QtCore.Qt.Key_Up: 'up', QtCore.Qt.Key_Down: 'down',
                 QtCore.Qt.Key_Left: 'left', QtCore.Qt.Key_Right: 'right'}

    def __init__(self, node):
        super().__init__()
        self.node = node
        self.setWindowTitle('DAMVI · 주행 모드')
        self.resize(530, 470)
        # Keep native focus/selection accents monochrome as well as custom paint.
        palette = self.palette()
        for group in (QtGui.QPalette.Active, QtGui.QPalette.Inactive, QtGui.QPalette.Disabled):
            for role in range(QtGui.QPalette.NColorRoles):
                color = palette.color(group, role)
                gray = QtGui.qGray(color.rgb())
                palette.setColor(group, role, QtGui.QColor(gray, gray, gray, color.alpha()))
        self.setPalette(palette)
        self.setStyleSheet('QWidget { background:#ffffff; color:#000000; font-size:14px; }'
                           'QPushButton { padding:13px; border:1px solid #999999; border-radius:7px; }'
                           'QPushButton:checked { background:#dddddd; border:2px solid #000000; font-weight:bold; }'
                           'QDoubleSpinBox { padding:7px; background:#ffffff; '
                           'selection-background-color:#dddddd; selection-color:#000000; }')
        layout = QtWidgets.QVBoxLayout(self)
        title = QtWidgets.QLabel('DAMVI  /  SIMULATION CONTROL')
        title.setStyleSheet('font-size:18px; font-weight:bold; padding:5px;')
        layout.addWidget(title)
        row = QtWidgets.QHBoxLayout()
        self.buttons = {}
        self.group = QtWidgets.QButtonGroup(self)
        for mode, label in [('stop', '정지'), ('manual', '수동주행'), ('auto', '자율주행')]:
            button = QtWidgets.QPushButton(label)
            button.setCheckable(True)
            button.setFocusPolicy(QtCore.Qt.NoFocus)
            button.clicked.connect(lambda checked, m=mode: self.change_mode(m))
            self.group.addButton(button)
            self.buttons[mode] = button
            row.addWidget(button)
        self.buttons['stop'].setChecked(True)
        layout.addLayout(row)
        speed_row = QtWidgets.QHBoxLayout()
        speed_row.addWidget(QtWidgets.QLabel('수동 속도 (m/s)'))
        self.speed = QtWidgets.QDoubleSpinBox()
        self.speed.setRange(0.0, max(20.0, node.control.speed))
        self.speed.setDecimals(2)
        self.speed.setSingleStep(0.1)
        self.speed.setValue(node.control.speed)
        self.speed.valueChanged.connect(self.speed_changed)
        speed_row.addWidget(self.speed)
        layout.addLayout(speed_row)
        hint = QtWidgets.QLabel('↑ 전진   ↓ 후진   ← 최대 좌향   → 최대 우향\n'
                                '키를 놓으면 해당 축은 0 · 반대 키 동시 입력은 0')
        hint.setStyleSheet('color:#666666; padding:5px;')
        layout.addWidget(hint)
        self.readout = QtWidgets.QLabel()
        self.readout.setStyleSheet('font-size:20px; padding:8px;')
        layout.addWidget(self.readout)
        self.plot = HistoryPlot(node.control.max_steering)
        layout.addWidget(self.plot)
        self.status = QtWidgets.QLabel()
        layout.addWidget(self.status)
        topics = QtWidgets.QLabel(f'입력  {node.subscription.topic_name}  [Twist]\n'
                                  f'출력  {node.publisher.topic_name}  [{node.output_type}]\n'
                                  '표시값: 출력 명령 (실측 속도·조향 피드백 아님)')
        topics.setStyleSheet('color:#666666; font-size:12px;')
        layout.addWidget(topics)
        self.setFocusPolicy(QtCore.Qt.StrongFocus)
        QtWidgets.QApplication.instance().installEventFilter(self)
        self.pump = QtCore.QTimer(self)
        self.pump.timeout.connect(self.tick)
        self.pump.start(5)
        self.display_timer = QtCore.QTimer(self)
        self.display_timer.timeout.connect(self.refresh)
        self.display_timer.start(50)
        self.node.publish_command()
        self.refresh()

    def speed_changed(self, value):
        self.node.control.speed = value
        self.node.publish_command()

    def change_mode(self, mode):
        self.node.set_mode(mode)
        self.setFocus()
        self.refresh()

    def eventFilter(self, obj, event):
        kind = event.type()
        if kind == QtCore.QEvent.ApplicationDeactivate:
            if self.node.control.mode == 'manual':
                self.buttons['stop'].setChecked(True)
                self.change_mode('stop')
        if kind in (QtCore.QEvent.KeyPress, QtCore.QEvent.KeyRelease) and event.key() in self.KEY_NAMES:
            if not self.isActiveWindow():
                return False
            # Spinbox arrows edit numbers; they never drive the vehicle.
            if self.speed.hasFocus():
                if self.node.control.keys:
                    self.node.control.keys.clear()
                    self.node.publish_command()
                return False
            if not event.isAutoRepeat() and self.node.control.mode == 'manual':
                key = self.KEY_NAMES[event.key()]
                if kind == QtCore.QEvent.KeyPress:
                    self.node.control.keys.add(key)
                else:
                    self.node.control.keys.discard(key)
                self.node.publish_command()
            return True
        if kind == QtCore.QEvent.FocusIn and obj is self.speed:
            self.node.control.keys.clear()
            self.node.publish_command()
        return super().eventFilter(obj, event)

    def tick(self):
        if not rclpy.ok():
            self.close()
            return
        rclpy.spin_once(self.node, timeout_sec=0.0)

    def refresh(self):
        speed, steering = self.node.last_values[0], self.node.last_values[5]
        self.readout.setText(f'출력  {speed:+.2f} m/s     {math.degrees(steering):+.2f}°')
        self.status.setText(self.node.status)
        self.plot.add(speed, steering)

    def closeEvent(self, event):
        self.pump.stop()
        self.display_timer.stop()
        if rclpy.ok():
            self.node.set_mode('stop')
        QtWidgets.QApplication.instance().removeEventFilter(self)
        event.accept()


def main(args=None):
    from rclpy.signals import SignalHandlerOptions
    # Let Qt exit before tearing down ROS, so Ctrl-C/SIGTERM can send zero.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    app = QtWidgets.QApplication([sys.argv[0]])
    previous_handlers = {}
    for sig in (signal.SIGINT, signal.SIGTERM):
        previous_handlers[sig] = signal.signal(sig, lambda signum, frame: app.quit())
    node = None
    try:
        node = TeleopNode()
        window = TeleopWindow(node)
        window.show()
        app.exec_()
    finally:
        if node is not None:
            if rclpy.ok():
                node.set_mode('stop')
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for sig, handler in previous_handlers.items():
            signal.signal(sig, handler)
