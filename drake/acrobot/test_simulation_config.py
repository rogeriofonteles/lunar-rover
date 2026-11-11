import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np

from acrobot_test import main, simulation_options


class SimulationConfigTests(unittest.TestCase):
    def test_yaml_and_cli_precedence(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "settings.yaml"
            path.write_text("mode: finite\nclip_torque: false\nheadless: true\n"
                            "initial_state: [3, 0, 0, 0]\n", encoding="utf-8")
            options = simulation_options(path)
            self.assertEqual(options["mode"], "finite")
            self.assertFalse(options["clip_torque"])
            np.testing.assert_array_equal(options["initial_state"], [3, 0, 0, 0])
            with patch("acrobot_test.run_simulation") as run:
                main(["--config", str(path), "--mode", "infinite",
                      "--torque-clipping", "--no-headless",
                      "--initial-state", "3.1", "0.1", "0", "0"])
            resolved = simulation_options(**run.call_args.kwargs)
            self.assertEqual(resolved["mode"], "infinite")
            self.assertTrue(resolved["clip_torque"])
            self.assertFalse(resolved["headless"])
            np.testing.assert_array_equal(resolved["initial_state"], [3.1, .1, 0, 0])

    def test_ilqr_cli_overrides(self):
        with patch("acrobot_test.run_simulation") as run:
            main(["--mode", "ilqr_receding", "--ilqr-max-iterations", "7",
                  "--ilqr-regularization", "0.01", "--ilqr-state-damping", "0",
                  "--ilqr-control-damping", "2", "--ilqr-initialization", "collocation"])
        options = simulation_options(**run.call_args.kwargs)
        self.assertEqual(options["mode"], "ilqr_receding")
        self.assertEqual(options["ilqr_max_iterations"], 7)
        self.assertEqual(options["ilqr_regularization"], .01)
        self.assertEqual(options["ilqr_state_damping"], 0)
        self.assertEqual(options["ilqr_control_damping"], 2)
        self.assertEqual(options["ilqr_initialization"], "collocation")

    def test_cost_cli_overrides(self):
        with patch("acrobot_test.run_simulation") as run:
            main(["--R", "0.02", "--Q-diagonal", "20", "20", "2", "2"])
        options = simulation_options(**run.call_args.kwargs)
        self.assertEqual(options["R"], .02)
        np.testing.assert_array_equal(options["Q"], np.diag([20., 20., 2., 2.]))

    def test_invalid_settings(self):
        for kwargs in ({"initial_state": [0, 0]}, {"timestep": 0},
                       {"clip_torque": "false"}, {"horizon": 1.5}, {"ilqr_regularization": 0},
                       {"ilqr_max_iterations": 0}, {"ilqr_control_damping": -1},
                       {"R": 0}, {"Q": -np.eye(4)}, {"ilqr_initialization": "unknown"}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                simulation_options(**kwargs)
