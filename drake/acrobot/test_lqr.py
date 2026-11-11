"""Run with: python -m unittest discover -s drake/acrobot -p 'test_*.py'."""
import unittest

import numpy as np
from pydrake.examples import AcrobotPlant
from pydrake.systems.controllers import DiscreteTimeLinearQuadraticRegulator

from lqr import wrapped_state_error, _linear_model, controller_gains, finite_horizon_lqr, infinite_horizon_lqr


class LqrTests(unittest.TestCase):
    def setUp(self):
        self.plant = AcrobotPlant()

    def test_wrapped_error_is_periodic_and_preserves_velocity(self):
        equilibrium = np.array([np.pi, 0., 0., 0.])
        state = np.array([np.pi + .2, -.3, 4., -5.])
        shifted = state + [6*np.pi, -8*np.pi, 0., 0.]
        batch = wrapped_state_error([state, shifted], equilibrium)
        np.testing.assert_allclose(batch, [[.2, -.3, 4., -5.]] * 2, atol=1e-14)
        self.assertEqual(wrapped_state_error([2*np.pi, np.pi, 0., 0.], equilibrium)[0], -np.pi)
        self.assertTrue(np.all(batch[:, :2] >= -np.pi))
        self.assertTrue(np.all(batch[:, :2] < np.pi))

    def test_infinite_matches_drake(self):
        A, B, Q = _linear_model(self.plant, 0.1, None, 1., None)
        expected, _ = DiscreteTimeLinearQuadraticRegulator(A, B, Q, np.eye(1))
        np.testing.assert_allclose(infinite_horizon_lqr(self.plant), expected, atol=1e-7)

    def test_finite_matches_full_trajectory_optimization(self):
        # Independently solve for all controls at once, then compare feedback rollout.
        N = 8
        A, B, Q = _linear_model(self.plant, 0.1, None, 1., None)
        x0 = np.array([.1, -.1, .02, -.03])
        F = np.vstack([np.linalg.matrix_power(A, k) for k in range(1, N + 1)])
        G = np.zeros((4 * N, N))
        for k in range(N):
            for j in range(k + 1):
                G[4*k:4*k+4, j:j+1] = np.linalg.matrix_power(A, k-j) @ B
        costs = np.kron(np.eye(N), Q)
        optimal = -np.linalg.solve(np.eye(N) + G.T @ costs @ G, G.T @ costs @ F @ x0)
        gains = finite_horizon_lqr(self.plant, horizon=N, return_sequence=True)
        x = x0.copy()
        controls = []
        for gain in gains:
            u = (-gain @ x).item()
            controls.append(u)
            x = A @ x + B[:, 0] * u
        np.testing.assert_allclose(controls, optimal, atol=1e-10)
        np.testing.assert_allclose(controller_gains(self.plant, 'receding', horizon=N)[0], gains[0])
        self.assertFalse(np.allclose(gains[0], gains[-1]))
        np.testing.assert_allclose(controller_gains(self.plant, 'finite', horizon=N), gains)

    def test_validation(self):
        for kwargs in ({'horizon': 0}, {'horizon': 1.5}, {'timestep': 0},
                       {'R': -1}, {'Q': -np.eye(4)}, {'terminal_cost': np.eye(3)}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                finite_horizon_lqr(self.plant, **kwargs)


if __name__ == '__main__':
    unittest.main()
