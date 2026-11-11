"""Torque configuration and saturation tests (no Drake dependency)."""
import tempfile
import unittest
from pathlib import Path

import numpy as np

from lqr import control_torque, load_torque_limit


class TorqueTests(unittest.TestCase):
    def test_default_limit_and_both_directions(self):
        limit = load_torque_limit()
        self.assertEqual(limit, 3.0)
        gain = np.array([[1., 0., 0., 0.]])
        for error, expected in [(10., -3.), (-10., 3.), (2., -2.), (3., -3.)]:
            with self.subTest(error=error):
                self.assertEqual(control_torque(gain, np.array([error, 0., 0., 0.]), limit), expected)
        self.assertEqual(control_torque(gain, np.array([10., 0., 0., 0.])), -10.)

    def test_custom_yaml_and_invalid_limits(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "custom.yaml"
            path.write_text("torque_limit_nm: 1.5\n", encoding="utf-8")
            limit = load_torque_limit(path)
            self.assertEqual(control_torque(np.ones((1, 4)), np.ones(4), limit), -1.5)
            for content in ("", "[]", "other: 3", "torque_limit_nm: -1",
                            "torque_limit_nm: 0", "torque_limit_nm: .inf",
                            "torque_limit_nm: .nan", "torque_limit_nm: true",
                            "torque_limit_nm: text"):
                path.write_text(content, encoding="utf-8")
                with self.subTest(content=content), self.assertRaises(ValueError):
                    load_torque_limit(path)


if __name__ == "__main__":
    unittest.main()
