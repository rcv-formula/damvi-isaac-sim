import tempfile
import unittest
from pathlib import Path

import numpy as np
import yaml

from tf_config import FrameTree


class FrameTreeTests(unittest.TestCase):
    def load(self, transforms):
        folder = tempfile.TemporaryDirectory()
        self.addCleanup(folder.cleanup)
        path = Path(folder.name)/'tf.yaml'
        path.write_text(yaml.safe_dump({'base_frame': 'base_link', 'transforms': transforms}))
        return FrameTree(path)

    def test_hierarchical_translation_and_rotation(self):
        tree = self.load([
            dict(parent='base_link', child='carrier', translation_m=[1, 0, 0], rotation_rpy_deg=[0, 0, 90]),
            dict(parent='carrier', child='sensor', translation_m=[2, 0, 0], rotation_rpy_deg=[0, 0, 90]),
        ])
        translation, rotation = tree.resolve('sensor')
        np.testing.assert_allclose(translation, [1, 2, 0], atol=1e-12)
        np.testing.assert_allclose(rotation, [0, 0, 1, 0], atol=1e-12)
        translation[:] = 100
        np.testing.assert_allclose(tree.resolve('sensor')[0], [1, 2, 0], atol=1e-12)

    def test_duplicate_cycle_and_disconnected_frames_rejected(self):
        edge = dict(parent='base_link', child='a', translation_m=[0, 0, 0], rotation_xyzw=[0, 0, 0, 1])
        for edges in ([edge, edge],
                      [dict(edge, parent='b'), dict(edge, parent='a', child='b')],
                      [dict(edge, parent='missing')]):
            with self.subTest(edges=edges), self.assertRaises(ValueError):
                self.load(edges)

    def test_invalid_rotation_rejected(self):
        for rotation in ([0, 0, 0, 0], [0, 0, float('nan'), 1], [1, 0, 0]):
            with self.subTest(rotation=rotation), self.assertRaises(ValueError):
                self.load([dict(parent='base_link', child='a', translation_m=[0, 0, 0], rotation_xyzw=rotation)])

    def test_user_selected_tf_matches_repository(self):
        tree = FrameTree(Path(__file__).resolve().parents[1]/'config/tf.yaml')
        np.testing.assert_allclose(tree.resolve('laser')[0], [.27, 0, .16])
        np.testing.assert_allclose(tree.resolve('imu')[0], [.27, 0, .06])
        np.testing.assert_allclose(tree.resolve('imu')[1], [0, 0, 0, 1])


if __name__ == '__main__':
    unittest.main()
