"""Real ROS graph + offscreen Qt keyboard integration, no physical hardware."""
import os
import time

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

import pytest
import rclpy
from PyQt5 import QtCore, QtWidgets, QtTest
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped
from damvi_teleop_ui.app import TeleopNode, TeleopWindow


@pytest.mark.parametrize('output_type', ['ackermann', 'twist'])
def test_ros_keyboard_modes_and_shutdown(output_type, tmp_path):
    rclpy.init(args=['--ros-args', '-p', f'output_type:={output_type}',
                     '-p', 'input_topic:=/test_damvi/input',
                     '-p', 'output_topic:=/test_damvi/output'])
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    node = TeleopNode()
    observer = rclpy.create_node('damvi_ui_test_observer')
    publisher = observer.create_publisher(Twist, '/test_damvi/input', 1)
    received = []
    message_type = Twist if output_type == 'twist' else AckermannDriveStamped
    sub = observer.create_subscription(message_type, '/test_damvi/output', received.append, 10)
    window = TeleopWindow(node)
    window.show()
    window.activateWindow()
    window.setFocus()

    def pump(duration=.12):
        until = time.monotonic() + duration
        while time.monotonic() < until:
            app.processEvents()
            rclpy.spin_once(observer, timeout_sec=0.)
            time.sleep(.002)

    def current():
        assert received
        m = received[-1]
        return ((m.linear.x, m.angular.z) if output_type == 'twist'
                else (m.drive.speed, m.drive.steering_angle))

    try:
        pump(.8)
        assert publisher.get_subscription_count() == 1
        assert observer.count_publishers('/test_damvi/output') == 1
        assert current() == (0., 0.)
        QtTest.QTest.mouseClick(window.buttons['manual'], QtCore.Qt.LeftButton)
        QtTest.QTest.keyPress(window, QtCore.Qt.Key_Up)
        QtTest.QTest.keyPress(window, QtCore.Qt.Key_Left)
        pump()
        assert current() == pytest.approx((2., .35 / .52))
        window.speed.setValue(3.5)
        pump()
        assert current()[0] == 3.5
        QtTest.QTest.keyRelease(window, QtCore.Qt.Key_Up)
        QtTest.QTest.keyRelease(window, QtCore.Qt.Key_Left)
        pump()
        assert current() == (0., 0.)
        QtTest.QTest.keyPress(window, QtCore.Qt.Key_Down)
        QtTest.QTest.keyPress(window, QtCore.Qt.Key_Right)
        pump()
        assert current() == pytest.approx((-3.5, -.35 / .52))
        app.sendEvent(app, QtCore.QEvent(QtCore.QEvent.ApplicationDeactivate))
        pump()
        assert node.control.mode == 'stop'
        assert current() == (0., 0.)
        QtTest.QTest.mouseClick(window.buttons['auto'], QtCore.Qt.LeftButton)
        pump()
        assert current() == (0., 0.)
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.linear.z = 1.25, .2, .3
        msg.angular.x, msg.angular.y, msg.angular.z = .4, .5, -.2
        publisher.publish(msg)
        pump()
        assert current() == pytest.approx((1.25, -.2))
        if output_type == 'twist':
            assert received[-1] == msg
        pump(.6)
        assert current() == (0., 0.)
        publisher.publish(msg)
        pump()
        QtTest.QTest.mouseClick(window.buttons['stop'], QtCore.Qt.LeftButton)
        publisher.publish(msg)
        pump()
        assert current() == (0., 0.)
        QtTest.QTest.mouseClick(window.buttons['auto'], QtCore.Qt.LeftButton)
        publisher.publish(msg)
        pump()
        assert current()[0] == 1.25
        window.grab().save(str(tmp_path / f'{output_type}.png'))
        window.close()
        pump()
        assert current() == (0., 0.)
    finally:
        window.close()
        node.destroy_node()
        observer.destroy_node()
        rclpy.shutdown()
