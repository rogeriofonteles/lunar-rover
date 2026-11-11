"""Linear MPC with Dual-Ascent, exact-Hessian SQP, and SciPy SLSQP solvers.

All solvers minimize the SAME condensed finite-horizon objective. Dynamics
are the fixed equilibrium linearization; nonlinear MPC is ilqr_receding.
"""
from dataclasses import dataclass

import numpy as np
from scipy.linalg import block_diag, cho_factor, cho_solve
from scipy.optimize import minimize

from .model import positive, positive_int, vector

MPC_SOLVERS = ("dual_ascent", "sqp", "slsqp")


@dataclass
class QPResult:
    x: np.ndarray
    converged: bool
    iterations: int
    status: str
    residual: float
    multipliers: np.ndarray | None = None


def projected_residual(H, g, x, lower, upper):
    """Infinity norm of the box projected-gradient KKT residual."""
    return float(np.max(np.abs(x - np.clip(x - (H @ x + g), lower, upper))))


class BoxQPSolver:
    def __init__(self, H, lower, upper, solver="dual_ascent", max_iterations=1000, tolerance=1e-6):
        if solver not in MPC_SOLVERS:
            raise ValueError(f"solver must be one of {MPC_SOLVERS}")
        self.H = np.asarray(H, dtype=float)
        if self.H.ndim != 2 or self.H.shape[0] != self.H.shape[1] or not np.isfinite(self.H).all():
            raise ValueError("H must be finite and square")
        if not np.allclose(self.H, self.H.T, atol=1e-10, rtol=0):
            raise ValueError("H must be symmetric")
        self.factor = cho_factor(self.H)
        self.n = len(self.H)
        self.lower = vector(lower, self.n, "lower")
        self.upper = vector(upper, self.n, "upper")
        if (self.lower >= self.upper).any():
            raise ValueError("Each lower bound must be below its upper bound")
        self.solver = solver
        self.max_iterations = positive_int(max_iterations, "max_iterations")
        self.tolerance = positive(tolerance, "tolerance")
        self.inverse = cho_solve(self.factor, np.eye(self.n)) if solver == "dual_ascent" else None
        self.scale = 1 / np.sqrt(np.diag(self.H))
        self.scaled_H = self.scale[:, None] * self.H * self.scale[None, :]

    def _solve_sqp(self, g, initial):
        """Exact-Hessian SQP specialized to this strictly convex box QP.

        The SQP subproblem is min grad'f p + .5 p'Hp, l-x <= p <= u-x.
        Since f is already quadratic and bounds are linear, this is the
        original problem in step coordinates: no nonlinear outer loop is
        needed. Solve it with a primal active set, independently of SLSQP.

        Working bounds fix p_i=0. Solve H_FF p_F=-grad_F on free variables,
        advance to the first blocking bound (or the full minimizer), and
        release a bound with an invalid multiplier at a stationary face.
        The feasible step is an exact line minimization on [0, alpha_bound].
        Iterations count working-set updates/steps, not nonlinear SQP passes.
        """
        H, lower, upper = self.H, self.lower, self.upper
        x = initial.copy()
        active = np.where(x == lower, -1, np.where(x == upper, 1, 0))
        status = "maximum iterations reached; feasible SQP iterate"
        for iteration in range(1, self.max_iterations + 1):
            grad = H @ x + g
            residual = projected_residual(H, g, x, lower, upper)
            if residual <= self.tolerance:
                return QPResult(x, True, iteration, "KKT tolerance", residual)
            free = active == 0
            if not free.any() or np.max(np.abs(grad[free])) <= self.tolerance:
                # grad_lower=lambda_lower >= 0; grad_upper=-lambda_upper <= 0.
                violation = np.where(active == -1, -grad,
                                     np.where(active == 1, grad, -np.inf))
                release = int(np.argmax(violation))
                if violation[release] <= self.tolerance:
                    status = "SQP numerical stationarity limit; KKT tolerance not achieved"
                    break
                active[release] = 0
                continue
            p = np.zeros(self.n)
            factor = cho_factor(self.scaled_H[np.ix_(free, free)])
            p[free] = self.scale[free] * cho_solve(factor, -self.scale[free]*grad[free])
            ratios = np.full(self.n, np.inf)
            up, down = p > 0, p < 0
            ratios[up] = (upper[up]-x[up]) / p[up]
            ratios[down] = (lower[down]-x[down]) / p[down]
            blocking = int(np.argmin(ratios))
            alpha = min(1., float(ratios[blocking]))
            trial = np.clip(x + alpha*p, lower, upper)
            if ratios[blocking] <= 1.:
                active[blocking] = 1 if p[blocking] > 0 else -1
                trial[blocking] = upper[blocking] if p[blocking] > 0 else lower[blocking]
            elif np.array_equal(trial, x):
                status = "SQP numerical step limit; KKT tolerance not achieved"
                break
            x = trial
        # A final permitted step may reach the solution: do not falsely report
        # nonconvergence just because there is no iteration left to recheck it.
        residual = projected_residual(H, g, x, lower, upper)
        converged = residual <= self.tolerance
        return QPResult(x, converged, iteration, "KKT tolerance" if converged else status, residual)

    def solve(self, gradient, initial=None, multipliers=None):
        g = vector(gradient, self.n, "gradient")
        H, lower, upper = self.H, self.lower, self.upper
        cold_start = np.clip(cho_solve(self.factor, -g), lower, upper)
        if initial is None:
            initial = cold_start
        initial = np.clip(vector(initial, self.n, "initial"), lower, upper)
        # A shifted plan can be worse than the current unconstrained solution,
        # especially once constraints become inactive. Compare feasible seeds.
        objective = lambda x: .5*x @ H @ x + g @ x
        if objective(cold_start) < objective(initial):
            initial = cold_start
        if self.solver == "sqp":
            return self._solve_sqp(g, initial)
        if self.solver == "slsqp":
            # Diagonal variable scaling gives SLSQP unit Hessian diagonals.
            scale = self.scale
            result = minimize(lambda y: .5 * (scale*y) @ H @ (scale*y) + g @ (scale*y),
                              initial / scale, jac=lambda y: scale * (H @ (scale*y) + g),
                              bounds=list(zip(lower / scale, upper / scale)), method="SLSQP",
                              options={"maxiter": self.max_iterations, "ftol": max(1e-14, self.tolerance**2)})
            x = np.clip(result.x * scale, lower, upper)
            if not np.isfinite(x).all():
                raise FloatingPointError("SLSQP returned nonfinite controls")
            residual = projected_residual(H, g, x, lower, upper)
            converged = residual <= self.tolerance
            status = "KKT tolerance" if converged else str(result.message) + "; KKT tolerance not achieved"
            return QPResult(x, converged, int(result.nit), status, residual)

        # L(x,lambda) = .5 x'Hx + g'x + lambda'(Gx-h), G=[I;-I].
        # x(lambda) = -H^-1(g + lambda_upper - lambda_lower).
        # Exact projected coordinate ascent on the concave dual updates each
        # multiplier by violation / (G H^-1 G')_ii. No primal clipping occurs
        # inside these updates. Clipping is only a feasible return safeguard.
        dual = np.zeros((2, self.n)) if multipliers is None else np.asarray(multipliers, dtype=float).copy()
        if dual.shape != (2, self.n) or not np.isfinite(dual).all() or (dual < 0).any():
            raise ValueError("multipliers must be nonnegative with shape (2, n)")
        x = cho_solve(self.factor, -(g + dual[0] - dual[1]))
        best_x = initial.copy()
        best_cost = .5 * best_x @ H @ best_x + g @ best_x
        for iteration in range(1, self.max_iterations + 1):
            for side, sign, bound in ((0, 1., upper), (1, -1., -lower)):
                for i in range(self.n):
                    updated = max(0., dual[side, i] + (sign*x[i] - bound[i]) / self.inverse[i, i])
                    change = updated - dual[side, i]
                    if change:
                        x -= sign * change * self.inverse[:, i]
                        dual[side, i] = updated
            feasible = np.clip(x, lower, upper)
            residual = projected_residual(H, g, feasible, lower, upper)
            if residual <= self.tolerance:
                return QPResult(feasible, True, iteration, "KKT tolerance", residual, dual)
            cost = .5 * feasible @ H @ feasible + g @ feasible
            if cost < best_cost:
                best_x, best_cost = feasible.copy(), cost
        return QPResult(best_x, False, iteration, "maximum iterations reached; best feasible iterate",
                        projected_residual(H, g, best_x, lower, upper), dual)


@dataclass
class MPCResult:
    states: np.ndarray  # LINEAR model prediction, not a nonlinear rollout
    controls: np.ndarray
    converged: bool
    iterations: int
    status: str
    residual: float
    objective: float


class ModelPredictiveController:
    def __init__(self, problem, solver="dual_ascent", max_iterations=1000, tolerance=1e-6):
        self.problem = p = problem
        N, n, m = p.horizon, 12, 6
        self.S = np.zeros((N*n, N*m))
        self.T = np.empty((N*n, n))
        power = np.eye(n)
        for k in range(N):
            power = p.A @ power
            self.T[k*n:(k+1)*n] = power
            for j in range(k + 1):
                self.S[k*n:(k+1)*n, j*m:(j+1)*m] = np.linalg.matrix_power(p.A, k-j) @ p.B
        Qbar = block_diag(*([p.Q] * (N - 1) + [p.Qf]))
        self.H = self.S.T @ Qbar @ self.S + np.kron(np.eye(N), p.R)
        self.H = (self.H + self.H.T) / 2
        self.F = self.S.T @ Qbar @ self.T
        lower = np.tile(-p.model.limit - p.u_ref, N)
        upper = np.tile(p.model.limit - p.u_ref, N)
        self.optimizer = BoxQPSolver(self.H, lower, upper, solver, max_iterations, tolerance)
        self.last_result = None
        self._dual = None
        self.solve_count = self.nonconverged_count = 0

    def command(self, measured_state):
        p = self.problem
        x = vector(measured_state, 12, "measured_state")
        error = x - p.target
        initial, dual = None, None
        if self.last_result is not None:
            old = self.last_result.controls - p.u_ref
            initial = np.concatenate((old[1:], old[-1:])).ravel()
        if self._dual is not None:
            old = self._dual.reshape(2, p.horizon, 6)
            dual = np.concatenate((old[:, 1:], old[:, -1:]), axis=1).reshape(2, -1)
        result = self.optimizer.solve(self.F @ error, initial, dual)
        controls = result.x.reshape(p.horizon, 6) + p.u_ref
        predicted = (self.T @ error + self.S @ result.x).reshape(p.horizon, 12) + p.target
        states = np.vstack((x, predicted))
        self.last_result = MPCResult(states, controls, result.converged, result.iterations,
                                     result.status, result.residual, p.cost(states, controls))
        self._dual = result.multipliers
        self.solve_count += 1
        self.nonconverged_count += int(not result.converged)
        return controls[0].copy()
