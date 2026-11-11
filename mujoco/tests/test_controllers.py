"""Numerical and nonlinear integration regression tests; run in Docker."""
from contextlib import redirect_stdout
import io
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np
from numpy.testing import assert_allclose

from arm.ilqr import iterative_lqr, RecedingHorizonILQR, box_policy
from arm.lqr import discretize, infinite_horizon_lqr, finite_horizon_lqr
from arm.model import ArmModel
from arm.mpc import BoxQPSolver, ModelPredictiveController
from arm.problem import ControlProblem
from arm.simulate import load_config, run_simulation


TARGET = np.array([0., .4, -.7, 0., .3, 0.] + [0.]*6)
INITIAL = TARGET + np.array([.3, .25, -.2, .2, .2, -.2] + [0.]*6)


class LinearControlTests(unittest.TestCase):
    def test_discretizations_against_analytic_double_integrator(self):
        A = np.array([[0., 1.], [0., 0.]])
        B = np.array([[0.], [1.]])
        for method, expected_B in (("euler", [[0.], [.1]]), ("zoh", [[.005], [.1]])):
            Ad, Bd = discretize(A, B, .1, method)
            assert_allclose(Ad, [[1., .1], [0., 1.]])
            assert_allclose(Bd, expected_B)

    def test_riccati_identity_and_finite_limit(self):
        A, B = discretize(np.array([[0., 1.], [0., 0.]]), np.array([[0.], [1.]]), .1)
        Q, R = np.eye(2), np.eye(1)
        K, P = infinite_horizon_lqr(A, B, Q, R)
        assert_allclose(P, Q + A.T @ P @ A - A.T @ P @ B @ K, atol=1e-10)
        Ks, Ps = finite_horizon_lqr(A, B, Q, R, 150)
        assert_allclose(Ks[0], K, atol=1e-9)
        self.assertLess(max(abs(np.linalg.eigvals(A-B@K))), 1)
        self.assertGreater(np.linalg.norm(Ks[0]-Ks[-1]), .1)
        assert_allclose(Ps[-1], Q)

    def test_invalid_parameters(self):
        for dt in (0, -1, np.nan):
            with self.assertRaises(ValueError):
                discretize(np.eye(2), np.ones((2, 1)), dt)
        with self.assertRaises(ValueError):
            finite_horizon_lqr(np.eye(2), np.ones((2, 1)), np.eye(2), [[-1]], 10)
        with self.assertRaises(ValueError):
            ArmModel(torque_limit=0)


class MuJoCoDynamicsTests(unittest.TestCase):
    def test_six_actuators_equilibrium_and_determinism(self):
        model = ArmModel()
        self.assertEqual((model.model.nq, model.model.nv, model.model.nu), (6, 6, 6))
        u = model.equilibrium_control(TARGET)
        self.assertGreater(np.linalg.norm(u), 1)
        assert_allclose(model.derivative(TARGET, u), 0, atol=1e-10)
        assert_allclose(model.step(TARGET, u), TARGET, atol=1e-10)
        first = model.step(INITIAL, u)
        model.step(np.zeros(12), np.zeros(6))
        assert_allclose(model.step(INITIAL, u), first, atol=0, rtol=0)

    def test_linear_prediction_near_equilibrium(self):
        model = ArmModel()
        p = ControlProblem(model, TARGET)
        dx = np.array([1, -2, 3, -1, 2, -3, 1, 1, 1, 1, 1, 1])*1e-5
        du = np.arange(6)*1e-5
        actual = model.step(TARGET+dx, p.u_ref+du)
        assert_allclose(actual, TARGET+p.A@dx+p.B@du, atol=3e-8)

    def test_target_must_be_stationary_and_feasible(self):
        moving = TARGET.copy()
        moving[6] = .1
        with self.assertRaises(ValueError):
            ArmModel().equilibrium_control(moving)
        with self.assertRaises(ValueError):
            ArmModel(torque_limit=1).equilibrium_control(TARGET)


class OptimizationTests(unittest.TestCase):
    def test_coupled_box_policy_is_not_componentwise_clipping(self):
        H = np.array([[2., 1.], [1., 2.]])
        g = np.array([-4., 0.])
        k, K = box_policy(H, g, np.eye(2), -np.ones(2), np.ones(2))
        assert_allclose(k, [1., -.5], atol=1e-7)
        assert_allclose(K[0], 0)
        assert_allclose(K[1], [0., .5])

    def test_all_mpc_solvers_agree_with_active_constraints(self):
        rng = np.random.default_rng(14)
        C = rng.normal(size=(12, 12))
        H = C.T@C + np.eye(12)*.5
        g = rng.normal(size=12)*10
        answers = []
        for method in ("dual_ascent", "sqp", "slsqp"):
            result = BoxQPSolver(H, -np.ones(12), np.ones(12), method, 3000, 2e-6).solve(g)
            self.assertTrue(result.converged, (method, result.status, result.residual))
            self.assertLessEqual(np.max(np.abs(result.x)), 1)
            self.assertTrue(np.any(np.isclose(np.abs(result.x), 1)))
            answers.append(result.x)
        for answer in answers[1:]:
            assert_allclose(answer, answers[0], atol=1e-5)

    def test_sqp_releases_wrong_bound_without_calling_scipy_optimizer(self):
        # Clipping the unconstrained solution incorrectly fixes x2=-1.
        H, g = np.array([[2., 1.], [1., 2.]]), np.array([-4., 0.])
        with patch("arm.mpc.minimize", side_effect=AssertionError("SQP must not call SLSQP")):
            limited = BoxQPSolver(H, [-1., -1.], [1., 1.], "sqp", max_iterations=1).solve(g)
            self.assertFalse(limited.converged)
            self.assertTrue(np.all(np.abs(limited.x) <= 1))
            result = BoxQPSolver(H, [-1., -1.], [1., 1.], "sqp", max_iterations=2).solve(g)
        self.assertTrue(result.converged, result.status)
        assert_allclose(result.x, [1., -.5], atol=1e-12)

    def test_sqp_matches_exhaustive_box_face_minimization(self):
        from itertools import product
        rng = np.random.default_rng(42)
        for _ in range(12):
            C = rng.normal(size=(4, 4))
            H = C.T@C + .15*np.eye(4)
            g = rng.normal(size=4)*8
            lower = rng.uniform(-3, -.1, 4)
            upper = rng.uniform(.1, 2, 4)
            candidates = []
            for face in product((-1, 0, 1), repeat=4):
                face = np.array(face)
                free = face == 0
                x = np.where(face == -1, lower, np.where(face == 1, upper, 0.))
                if free.any():
                    x[free] = np.linalg.solve(H[np.ix_(free, free)],
                                             -g[free]-H[np.ix_(free, ~free)]@x[~free])
                if np.all(x >= lower) and np.all(x <= upper):
                    candidates.append((.5*x@H@x+g@x, x.copy()))
            expected = min(candidates, key=lambda item: item[0])[1]
            solver = BoxQPSolver(H, lower, upper, "sqp", tolerance=1e-9)
            result = solver.solve(g, initial=rng.uniform(lower, upper))
            self.assertTrue(result.converged, (result.status, result.residual))
            assert_allclose(result.x, expected, atol=1e-8)
            self.assertTrue(np.all(result.x >= lower) and np.all(result.x <= upper))

    def test_iteration_limit_reports_feasible_nonconvergence(self):
        rng = np.random.default_rng(7)
        C = rng.normal(size=(15, 15))
        result = BoxQPSolver(C.T@C+np.eye(15)*.1, -np.ones(15), np.ones(15),
                             max_iterations=1, tolerance=1e-12).solve(rng.normal(size=15)*30)
        self.assertFalse(result.converged)
        self.assertTrue(np.isfinite(result.x).all())
        self.assertLessEqual(np.max(np.abs(result.x)), 1)

    def test_mpc_matches_unconstrained_lqr_and_model_predictions(self):
        p = ControlProblem(ArmModel(), TARGET, horizon=12)
        state = TARGET + (INITIAL-TARGET)*.05
        for method in ("dual_ascent", "sqp", "slsqp"):
            mpc = ModelPredictiveController(p, method)
            u = mpc.command(state)
            assert_allclose(u, p.u_ref-p.K@(state-TARGET), atol=1e-5)
            result = mpc.last_result
            self.assertTrue(result.converged, result.status)
            for k in range(p.horizon):
                assert_allclose(result.states[k+1]-p.target,
                                p.A@(result.states[k]-p.target)+p.B@(result.controls[k]-p.u_ref), atol=1e-12)
            mpc.command(p.model.step(state, u))
            self.assertEqual(mpc.solve_count, 2)

    def test_ilqr_decreases_cost_and_respects_bounds(self):
        p = ControlProblem(ArmModel(torque_limit=[35, 35, 20, 4, 4, 3]), TARGET, horizon=40)
        seed = np.tile(p.u_ref, (p.horizon, 1))
        result = iterative_lqr(p, INITIAL, seed, max_iterations=12)
        self.assertTrue(result.converged, result.status)
        self.assertLess(result.costs[-1], .8*result.costs[0])
        self.assertTrue(np.all(np.diff(result.costs) < 0))
        self.assertTrue(np.all(np.abs(result.controls) <= p.model.limit+1e-12))
        self.assertTrue(np.any(np.isclose(np.abs(result.controls), p.model.limit)))
        assert_allclose(p.rollout(INITIAL, result.controls), result.states, atol=1e-12)
        self.assertLess(np.linalg.norm(result.states[-1]-TARGET), np.linalg.norm(INITIAL-TARGET))

    def test_mpc_solvers_agree_for_saturated_arm(self):
        p = ControlProblem(ArmModel(torque_limit=[35, 35, 20, 4, 4, 3]), TARGET, horizon=20)
        plans = []
        for method in ("dual_ascent", "sqp", "slsqp"):
            controller = ModelPredictiveController(p, method, tolerance=2e-6)
            controller.command(INITIAL)
            result = controller.last_result
            self.assertTrue(result.converged, (method, result.status, result.residual))
            self.assertTrue(np.any(np.isclose(np.abs(result.controls), p.model.limit)))
            plans.append(result.controls)
        for plan in plans[1:]:
            assert_allclose(plan, plans[0], atol=2e-5)

    def test_receding_ilqr_shifts_plan_and_uses_measurement(self):
        p = ControlProblem(ArmModel(), TARGET, horizon=8)
        controller = RecedingHorizonILQR(p, max_iterations=2)
        controller.command(INITIAL)
        old = controller.last_result.controls.copy()
        measured = p.model.step(INITIAL, old[0])
        measured[0] += .02
        with patch("arm.ilqr.iterative_lqr", wraps=iterative_lqr) as solve:
            controller.command(measured)
        assert_allclose(solve.call_args.kwargs["initial_controls"], np.vstack((old[1:], old[-1:])))
        assert_allclose(controller.last_result.states[0], measured)
        self.assertEqual(controller.solve_count, 2)


class SimulationTests(unittest.TestCase):
    def test_all_linear_modes_stabilize_nonlinear_arm(self):
        for method in ("euler", "zoh"):
            for mode in ("infinite", "finite", "receding", "mpc"):
                with self.subTest(method=method, mode=mode), redirect_stdout(io.StringIO()):
                    xs, us, info = run_simulation(mode=mode, discretization=method, horizon=30,
                                                  steps=50, headless=True, wait=False)
                    self.assertLess(info["final_error"], .65*info["initial_error"])
                    self.assertEqual(len(us), 30 if mode == "finite" else 50)
                    self.assertTrue(np.isfinite(xs).all())
                    self.assertEqual(info["nonconverged_solves"], 0)

    def test_finite_ilqr_stops_at_horizon_and_receding_continues(self):
        for mode, expected in (("ilqr", 8), ("ilqr_receding", 10)):
            with redirect_stdout(io.StringIO()):
                xs, us, info = run_simulation(mode=mode, horizon=8, steps=10,
                                              ilqr_max_iterations=2, headless=True, wait=False)
            self.assertEqual(len(us), expected)
            self.assertEqual(len(xs), expected+1)

    def test_config_overrides_validation_and_trajectory_export(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/"config.yaml"
            path.write_text("mode: finite\nsteps: 4\nheadless: true\n")
            self.assertEqual(load_config(path, steps=3)["steps"], 3)
            output = Path(directory)/"run.npz"
            with redirect_stdout(io.StringIO()):
                run_simulation(path, steps=3, wait=False, output=output)
            with np.load(output) as saved:
                self.assertEqual(saved["states"].shape, (4, 12))
                self.assertEqual(saved["controls"].shape, (3, 6))
            self.assertTrue(output.with_suffix(".json").exists())
            for text in ("horizon: 0", "headless: 'false'", "unknown: 1", "mode: bad", "solver: bad"):
                path.write_text(text)
                with self.assertRaises(ValueError):
                    load_config(path)


if __name__ == "__main__":
    unittest.main()
