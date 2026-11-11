"""Button reachability, configuration selection, and end-to-end movement."""
from contextlib import redirect_stdout
import io
import json
from pathlib import Path
import tempfile
import unittest

import mujoco
import numpy as np
from numpy.testing import assert_allclose

from arm.buttons import BUTTON_NAMES, button_target
from arm.model import ArmModel
from arm.simulate import load_config, run_simulation


class ButtonTests(unittest.TestCase):
    def test_targets_are_separated_in_space_and_height(self):
        positions = np.array([button_target(name)["position_m"] for name in BUTTON_NAMES])
        for i in range(len(positions)):
            for j in range(i):
                self.assertGreater(np.linalg.norm(positions[i]-positions[j]), .9)
        self.assertGreater(np.ptp(positions[:, 2]), .45)

    def test_saved_joints_match_button_faces_and_approach_direction(self):
        arm = ArmModel(torque_limit=load_config()["torque_limit"])
        for name in BUTTON_NAMES:
            with self.subTest(button=name):
                recorded = button_target(name)
                target = np.r_[recorded["joint_positions_rad"], np.zeros(6)]
                torque = arm.equilibrium_control(target)
                arm.data.qpos[:] = target[:6]
                arm.data.qvel[:] = 0
                mujoco.mj_forward(arm.model, arm.data)
                tip = arm.data.site("button_tip")
                button = arm.data.site(f"button_{name}_target")
                assert_allclose(button.xpos, recorded["position_m"], atol=1e-10)
                assert_allclose(tip.xpos, button.xpos, atol=1e-10)
                assert_allclose(tip.xmat.reshape(3, 3)[:, 2],
                                -button.xmat.reshape(3, 3)[:, 2], atol=1e-10)
                assert_allclose(arm.derivative(target, torque), 0, atol=1e-10)

    def test_button_selection_preserves_initial_state_and_validates_input(self):
        default = load_config()
        for name in BUTTON_NAMES:
            selected = load_config(button=name)
            self.assertEqual(selected["initial_state"], default["initial_state"])
            self.assertEqual(selected["target_state"],
                             button_target(name)["joint_positions_rad"] + [0.]*6)
        with self.assertRaises(ValueError):
            load_config(button="blue")
        with self.assertRaises(ValueError):
            load_config(button="red", target_state=default["target_state"])
        with tempfile.TemporaryDirectory() as directory:
            config = Path(directory) / "button.yaml"
            config.write_text("button: red\n")
            self.assertEqual(load_config(config)["button"], "red")
            self.assertEqual(load_config(config, button="green")["target_state"],
                             load_config(button="green")["target_state"])

    def test_all_buttons_reached_from_identical_start_and_exported(self):
        initial = load_config()["initial_state"]
        with tempfile.TemporaryDirectory() as directory:
            for name in BUTTON_NAMES:
                with self.subTest(button=name), redirect_stdout(io.StringIO()):
                    output = Path(directory) / name
                    xs, us, info = run_simulation(button=name, headless=True,
                                                  wait=False, output=output)
                    assert_allclose(xs[0], initial, atol=0, rtol=0)
                    self.assertLess(info["final_error"], 1e-3)
                    self.assertLess(info["final_tip_error_m"], 1e-4)
                    self.assertEqual(info["button"], name)
                    self.assertTrue(np.all(np.abs(us) <= load_config()["torque_limit"]))
                    with np.load(output.with_suffix(".npz")) as saved:
                        assert_allclose(saved["target"], load_config(button=name)["target_state"])
                    metadata = json.loads(output.with_suffix(".json").read_text())
                    self.assertEqual(metadata["button"], name)
                    self.assertEqual(metadata["initial_state"], initial)


if __name__ == "__main__":
    unittest.main()
