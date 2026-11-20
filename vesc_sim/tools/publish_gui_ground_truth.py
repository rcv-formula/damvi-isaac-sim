"""Execute in the running Isaac GUI: publish measured rear-axle poses for review.

This publishes validation-only topics. It does not publish TF, initialize the
particle filter, or change the stage, vehicle, sensors, or controller.
"""
import math
import omni.kit.app
import live_gui
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as GroundTruthPathMessage
from rclpy.qos import QoSProfile, DurabilityPolicy

validation_previous_poses = []
if 'validation_gt_hook' in globals():
    validation_previous_poses = list(validation_gt_hook.poses)
    validation_gt_hook.close()


class ValidationGroundTruth:
    def __init__(self, controller):
        self.controller = controller
        self.node = controller.bridge.node
        self.pose_pub = self.node.create_publisher(PoseStamped, '/validation/ground_truth', 20)
        self.path_pub = self.node.create_publisher(
            GroundTruthPathMessage, '/validation/ground_truth_path',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.last_stamp = -1.
        self.last_path = -1.
        self.poses = validation_previous_poses[-3000:]
        self.subscription = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(
            self.on_frame, name='validation_measured_rear_axle_pose')

    def on_frame(self, event):
        rows = self.controller.rows
        if not rows:
            return
        if rows[-1]['time'] < self.last_stamp:
            self.last_stamp = self.last_path = -1.
            self.poses.clear()
        offset = self.controller.config['scene']['rear_axle_offset_m']
        for row in rows:
            if row['time'] <= self.last_stamp:
                continue
            qx, qy, qz, qw = row['orientation_xyzw']
            # Rotate the chassis->rear-axle offset by the measured 3-D attitude.
            vx, vy, vz = offset
            tx = 2 * (qy*vz - qz*vy)
            ty = 2 * (qz*vx - qx*vz)
            tz = 2 * (qx*vy - qy*vx)
            rx = vx + qw*tx + qy*tz - qz*ty
            ry = vy + qw*ty + qz*tx - qx*tz
            rz = vz + qw*tz + qx*ty - qy*tx
            pose = PoseStamped()
            sec = int(row['time'])
            nsec = round((row['time'] - sec)*1e9)
            if nsec >= 1000000000:
                sec += 1
                nsec -= 1000000000
            pose.header.stamp.sec = sec
            pose.header.stamp.nanosec = nsec
            pose.header.frame_id = 'map'
            pose.pose.position.x = row['position'][0] + rx
            pose.pose.position.y = row['position'][1] + ry
            pose.pose.position.z = row['position'][2] + rz
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
            self.pose_pub.publish(pose)
            # Avoid a startup burst of old trace samples.
            if self.last_stamp >= 0:
                previous = self.poses[-1] if self.poses else None
                moved = previous is None or math.hypot(
                    pose.pose.position.x-previous.pose.position.x,
                    pose.pose.position.y-previous.pose.position.y) >= .005
                yaw = math.atan2(2*(qw*qz+qx*qy), 1-2*(qy*qy+qz*qz))
                old = previous.pose.orientation if previous is not None else None
                old_yaw = math.atan2(2*(old.w*old.z+old.x*old.y),
                                     1-2*(old.y*old.y+old.z*old.z)) if old else yaw
                turned = abs(math.atan2(math.sin(yaw-old_yaw),math.cos(yaw-old_yaw))) >= .005
                if moved or turned:
                    self.poses.append(pose)
            self.last_stamp = row['time']
        self.poses = self.poses[-3000:]
        if self.last_stamp - self.last_path >= .5 and self.poses:
            path = GroundTruthPathMessage()
            path.header = self.poses[-1].header
            path.poses = self.poses
            self.path_pub.publish(path)
            self.last_path = self.last_stamp

    def close(self):
        self.subscription = None
        self.node.destroy_publisher(self.pose_pub)
        self.node.destroy_publisher(self.path_pub)


validation_gt_hook = ValidationGroundTruth(live_gui.controller)
# Start at the latest measured pose, without replaying the pre-validation history.
if live_gui.controller.rows:
    validation_gt_hook.last_stamp = live_gui.controller.rows[-1]['time'] - .000001
print('Validation-only ground truth attached: map coordinates, measured rear axle, no TF or PF input.')
