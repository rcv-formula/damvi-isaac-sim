"""Restore the measured test trajectory for RViz review; no PF input or TF."""
import json
from pathlib import Path as GroundTruthSeedFile
from geometry_msgs.msg import PoseStamped
seed = GroundTruthSeedFile('/home/shin/Desktop/RC_Car_Isaac/vesc_sim/outputs/validation_gt_path_seed.json')
poses = []
for point in json.loads(seed.read_text()):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp.sec, pose.header.stamp.nanosec = divmod(point['stamp_ns'],1000000000)
    pose.pose.position.x,pose.pose.position.y,pose.pose.position.z = point['x'],point['y'],point['z']
    q=point['quaternion_xyzw']
    pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w=q
    poses.append(pose)
validation_gt_hook.poses = poses
validation_gt_hook.last_path = -1.
print(f'Restored {len(poses)} measured GT path points for review; localization unchanged.')
