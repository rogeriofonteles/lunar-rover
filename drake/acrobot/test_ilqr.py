import unittest
import contextlib
import io
from unittest.mock import patch
import numpy as np
from pydrake.examples import AcrobotPlant
from pydrake.systems.analysis import Simulator
from ilqr import iterative_lqr, nonlinear_step, RecedingHorizonILQR
from lqr import _riccati_step, wrapped_state_error
from acrobot_test import run_simulation
from trajectory_seed import collocation_seed


class ILQRTests(unittest.TestCase):
    def test_linear_case_matches_riccati(self):
        A = np.eye(4); A[0, 2] = .1; A[1, 3] = .1
        B = np.array([[0.], [.02], [.1], [.2]])
        Q = np.diag([10., 10., 1., 1.]); horizon = 20
        value = Q.copy(); expected = []
        for _ in range(horizon):
            gain, value = _riccati_step(A, B, Q, 1., value)
            expected.append(gain)
        expected.reverse()
        x = np.array([.1, -.1, .2, -.2]); controls = []
        for gain in expected:
            u = (-gain @ x).item(); controls.append(u); x = A @ x + B[:, 0]*u
        linear_step = lambda x, u: A @ x + B[:, 0]*u
        linear_step.jacobian = lambda x, u: (A, B)
        with patch('ilqr.nonlinear_step', return_value=linear_step):
            result = iterative_lqr(None, [.1, -.1, .2, -.2], horizon=horizon,
                                   equilibrium_state=np.zeros(4), terminal_cost=Q, regularization=1e-9,
                                   tolerance=1e-10)
        self.assertTrue(result.converged, result.status)
        np.testing.assert_allclose(result.controls, controls, atol=1e-6)
        np.testing.assert_allclose(result.gains, expected, atol=1e-6)

    def test_nonlinear_improvement_and_bounds(self):
        result = iterative_lqr(AcrobotPlant(), [3.13, .01, 0, 0], horizon=100,
                               torque_limit=3., max_iterations=30, initial_controls=np.zeros(100))
        self.assertTrue(result.converged, result.status)
        self.assertLess(result.costs[-1], result.costs[0]*.1)
        self.assertTrue(np.all(np.diff(result.costs) < 0))
        self.assertLessEqual(np.max(np.abs(result.controls)), 3.)
        self.assertTrue(np.all(np.isfinite(result.gains)))

    def test_iteration_limit_reported(self):
        result = iterative_lqr(AcrobotPlant(), [3., .1, 0, 0], horizon=50,
                               max_iterations=1, tolerance=1e-12)
        self.assertFalse(result.converged)
        self.assertEqual(result.status, 'maximum iterations reached')

    def test_three_second_stabilization(self):
        # Regression: zero initialization converged to a falling trajectory.
        for clipping in (False, True):
            with self.subTest(clipping=clipping), contextlib.redirect_stdout(io.StringIO()):
                state = run_simulation(
                    mode="ilqr", initial_state=[3.1316, .01, 0, 0],
                    timestep=.01, horizon=300, steps=300,
                    clip_torque=clipping, torque_limit_nm=3., headless=True, wait=False,
                )
            error = wrapped_state_error(state, [np.pi, 0, 0, 0])
            self.assertLess(np.linalg.norm(error), .005)

    def test_receding_uses_measurement_and_shifted_warm_start(self):
        controller = RecedingHorizonILQR(AcrobotPlant(), horizon=20, timestep=.02,
                                          torque_limit=3., max_iterations=10)
        first = controller.command([3.13, .01, 0, 0])
        previous = controller.last_result
        measured = previous.states[1].copy()
        measured[1] += .02  # Disturb the plant away from the predicted state.
        with patch("ilqr.iterative_lqr", wraps=iterative_lqr) as solve:
            second = controller.command(measured)
        np.testing.assert_array_equal(solve.call_args.args[1], measured)
        np.testing.assert_array_equal(solve.call_args.kwargs["initial_controls"],
                                      np.r_[previous.controls[1:], previous.controls[-1]])
        np.testing.assert_array_equal(controller.last_result.states[0], measured)
        self.assertEqual(controller.solve_count, 2)
        self.assertLessEqual(abs(first), 3.)
        self.assertLessEqual(abs(second), 3.)
        self.assertTrue(np.isfinite(second))

    def test_receding_simulation_runs_beyond_horizon(self):
        # A short moving horizon must continue balancing beyond its endpoint.
        with patch("ilqr.iterative_lqr", wraps=iterative_lqr) as solve:
            with contextlib.redirect_stdout(io.StringIO()):
                state = run_simulation(
                    mode="ilqr_receding", initial_state=[3.13, .01, 0, 0],
                    timestep=.05, horizon=20, steps=60, ilqr_max_iterations=10,
                    clip_torque=True, torque_limit_nm=3., headless=True, wait=False,
                )
            self.assertEqual(solve.call_count, 60)
        self.assertLess(np.linalg.norm(wrapped_state_error(state, [np.pi, 0, 0, 0])), .01)

    def test_large_offset_collocation_seed_reaches_upright(self):
        plant = AcrobotPlant(); target = np.array([np.pi, 0, 0, 0])
        initial = np.array([2.5, .6, 0, 0]); dynamics = nonlinear_step(plant, .01)
        local = iterative_lqr(plant, target, horizon=1, initialization="lqr")
        controls = collocation_seed(
            plant, initial, target, .01, 300, np.diag([10., 10., 1., 1.]),
            1., local.terminal_cost, np.inf, dynamics,
        )
        state = initial.copy()
        for control in controls:
            state = dynamics(state, control)
        self.assertTrue(np.all(np.isfinite(controls)))
        self.assertLess(np.linalg.norm(wrapped_state_error(state, target)), .01)

    def test_auto_seed_failure_is_reported(self):
        with patch("trajectory_seed.collocation_seed", side_effect=ValueError("infeasible")):
            with self.assertWarnsRegex(RuntimeWarning, "Collocation seed unavailable"):
                result = iterative_lqr(AcrobotPlant(), [3., .3, 0, 0], horizon=20,
                                       max_iterations=1)
        self.assertIn(result.initialization, ("zero", "lqr"))

    def test_rk4_matches_drake_held_torque(self):
        plant = AcrobotPlant(); x = np.array([3., .1, .2, -.1]); u = .5
        predicted = nonlinear_step(plant, .1)(x, u)
        simulator = Simulator(plant)
        context = simulator.get_mutable_context(); context.SetContinuousState(x)
        plant.get_input_port(0).FixValue(context, [u])
        simulator.get_mutable_integrator().set_target_accuracy(1e-9)
        simulator.AdvanceTo(.1)
        np.testing.assert_allclose(predicted, context.get_continuous_state_vector().CopyToVector(), atol=1e-6)
