#!/usr/bin/env python3
import math
import os
import sys
import tempfile
import unittest
import yaml

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "nodes"))
from asset_bundle import atomic_yaml, compose, inverse, relative, sha256_file, validate_pose_dict


class AssetBundleTest(unittest.TestCase):
    def assertPoseAlmostEqual(self, left, right):
        for a, b in zip(left, right):
            self.assertAlmostEqual(a, b, places=8)

    def test_non_identity_cartographer_reference(self):
        # 本机实现是 G_new=G_reference(time0)*relative；验证 reference 非单位时仍回到目标 map pose。
        reference = (4.2, -1.3, 1.1)
        desired = (-0.4, 2.7, -2.2)
        request_relative_pose = relative(reference, desired)
        self.assertPoseAlmostEqual(compose(reference, request_relative_pose), desired)

    def test_anchor_reprojection_after_optimization(self):
        old_anchor = (1.0, 2.0, 0.3)
        captured_base = (1.4, 2.2, 0.5)
        rel = relative(old_anchor, captured_base)
        optimized_anchor = (1.2, 1.8, 0.4)
        projected = compose(optimized_anchor, rel)
        self.assertNotEqual(projected, captured_base)
        self.assertPoseAlmostEqual(relative(optimized_anchor, projected), rel)

    def test_atomic_yaml_and_hash(self):
        with tempfile.TemporaryDirectory() as directory:
            path = os.path.join(directory, "manifest.yaml")
            atomic_yaml(path, {"map_id": "m1", "complete": True})
            with open(path) as stream:
                self.assertEqual(yaml.safe_load(stream)["map_id"], "m1")
            self.assertEqual(len(sha256_file(path)), 64)
            self.assertFalse(any(name.startswith(".tmp-") for name in os.listdir(directory)))

    def test_bad_pose_rejected(self):
        with self.assertRaises((ValueError, KeyError)):
            validate_pose_dict({"x": float("nan"), "y": 0, "yaw": 0})
        with self.assertRaises((ValueError, KeyError)):
            validate_pose_dict({"x": 0, "y": 0})


if __name__ == "__main__":
    unittest.main()
