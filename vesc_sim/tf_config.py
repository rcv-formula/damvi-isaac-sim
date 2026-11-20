"""One validated static frame tree for sensor mounting and ROS TF."""
import hashlib
from pathlib import Path

import numpy as np
import yaml


def multiply_quaternions(a, b):
    av, bv = np.asarray(a[:3]), np.asarray(b[:3])
    return np.r_[a[3]*bv + b[3]*av + np.cross(av, bv),
                 a[3]*b[3] - np.dot(av, bv)]


def rotate(quaternion, vector):
    q = np.asarray(quaternion)
    v = np.asarray(vector)
    return v + 2*np.cross(q[:3], np.cross(q[:3], v) + q[3]*v)


def rotation_from_rpy_degrees(rpy):
    r, p, y = np.radians(rpy)/2
    cr, cp, cy = np.cos([r, p, y])
    sr, sp, sy = np.sin([r, p, y])
    return np.array([sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy,
                     cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy])


class FrameTree:
    """Resolve child-frame coordinates into the configured base frame."""
    def __init__(self, path):
        self.path = Path(path)
        content = self.path.read_bytes()
        self.sha256 = hashlib.sha256(content).hexdigest()
        self.config = yaml.safe_load(content)
        self.base_frame = self.config['base_frame']
        self.publish_static_tf = bool(self.config.get('publish_static_tf', True))
        if not isinstance(self.base_frame, str) or not self.base_frame or self.base_frame.startswith('/'):
            raise ValueError('TF base_frame must be a nonempty relative frame ID')
        self.edges = {}
        self.cache = {self.base_frame: (np.zeros(3), np.array([0., 0., 0., 1.]))}
        for entry in self.config['transforms']:
            parent, child = entry['parent'], entry['child']
            if any(not isinstance(f, str) or not f or f.startswith('/') for f in (parent, child)):
                raise ValueError('TF frame IDs must be nonempty and have no leading slash')
            if child == self.base_frame or child in self.edges or child == parent:
                raise ValueError(f'Duplicate, root, or self-parented TF child: {child}')
            translation = np.asarray(entry['translation_m'], dtype=float)
            if translation.shape != (3,) or not np.isfinite(translation).all():
                raise ValueError(f'Invalid TF translation for {child}')
            if ('rotation_xyzw' in entry) == ('rotation_rpy_deg' in entry):
                raise ValueError(f'Specify one TF rotation representation for {child}')
            if 'rotation_xyzw' in entry:
                quaternion = np.asarray(entry['rotation_xyzw'], dtype=float)
            else:
                rpy = np.asarray(entry['rotation_rpy_deg'], dtype=float)
                if rpy.shape != (3,) or not np.isfinite(rpy).all():
                    raise ValueError(f'Invalid TF RPY for {child}')
                quaternion = rotation_from_rpy_degrees(rpy)
            if quaternion.shape != (4,) or not np.isfinite(quaternion).all() or np.linalg.norm(quaternion) < 1e-12:
                raise ValueError(f'Invalid TF quaternion for {child}')
            quaternion = quaternion / np.linalg.norm(quaternion)
            self.edges[child] = (parent, translation, quaternion)
        for child in self.edges:
            self._resolve(child, set())

    def _resolve(self, frame, visiting):
        if frame in self.cache:
            return self.cache[frame]
        if frame not in self.edges:
            raise ValueError(f'TF frame {frame!r} is not connected to {self.base_frame!r}')
        if frame in visiting:
            raise ValueError(f'TF cycle at {frame!r}')
        parent, translation, quaternion = self.edges[frame]
        parent_t, parent_q = self._resolve(parent, visiting | {frame})
        result = (parent_t + rotate(parent_q, translation),
                  multiply_quaternions(parent_q, quaternion))
        self.cache[frame] = result
        return result

    def resolve(self, frame_id):
        return tuple(v.copy() for v in self._resolve(frame_id, set()))

    def status(self):
        return {'config_path': str(self.path), 'config_sha256': self.sha256,
                'base_frame': self.base_frame, 'publish_static_tf': self.publish_static_tf,
                'transforms': [dict(parent=p, child=c, translation_m=t.tolist(), rotation_xyzw=q.tolist())
                               for c, (p, t, q) in self.edges.items()]}


class StaticFrames:
    """Publish the complete tree in one retained /tf_static message."""
    def __init__(self, node, tree):
        self.node = node
        self.publisher = None
        self.reload(tree)

    def reload(self, tree):
        publisher = None
        if tree.publish_static_tf:
            from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
            from geometry_msgs.msg import TransformStamped
            from tf2_msgs.msg import TFMessage
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
            publisher = self.node.create_publisher(TFMessage, '/tf_static', qos)
            transforms = []
            for child, (parent, translation, quaternion) in tree.edges.items():
                transform = TransformStamped()
                transform.header.frame_id = parent
                transform.child_frame_id = child
                transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z = map(float, translation)
                transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w = map(float, quaternion)
                transforms.append(transform)
            publisher.publish(TFMessage(transforms=transforms))
        self.close()
        self.tree = tree
        self.publisher = publisher

    def close(self):
        if self.publisher is not None:
            self.node.destroy_publisher(self.publisher)
            self.publisher = None
