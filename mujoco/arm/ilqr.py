"""Multi-input, box-constrained iLQR with nonlinear MuJoCo rollouts."""
from dataclasses import dataclass

import numpy as np
from scipy.optimize import minimize

from .model import positive, positive_int, vector


@dataclass
class ILQRResult:
    states: np.ndarray
    controls: np.ndarray
    gains: np.ndarray  # u = u_nominal - K (x - x_nominal)
    costs: list
    converged: bool
    iterations: int
    status: str


def box_policy(H, g, cross, lower, upper):
    """Solve the coupled six-input box QP and differentiate its free variables.

    Componentwise clipping of an unconstrained Newton step is incorrect when
    H has off-diagonal terms. Active controls have zero local feedback.
    """
    np.linalg.cholesky(H)
    unconstrained = np.linalg.solve(H, -g)
    if np.all(unconstrained >= lower) and np.all(unconstrained <= upper):
        return unconstrained, np.linalg.solve(H, cross)
    result = minimize(lambda k: .5 * k @ H @ k + g @ k,
                      np.clip(unconstrained, lower, upper),
                      jac=lambda k: H @ k + g, bounds=list(zip(lower, upper)),
                      method="L-BFGS-B", options={"ftol": 1e-14, "gtol": 1e-9, "maxiter": 200})
    k = np.clip(result.x, lower, upper)
    gradient = H @ k + g
    residual = np.max(np.abs(k - np.clip(k - gradient, lower, upper)))
    if not np.isfinite(k).all() or residual > 1e-5:
        raise np.linalg.LinAlgError("Box QP did not reach stationarity")
    active = ((k <= lower + 1e-7) & (gradient >= 0)) | ((k >= upper - 1e-7) & (gradient <= 0))
    free = ~active
    K = np.zeros_like(cross)
    if free.any():
        K[free] = np.linalg.solve(H[np.ix_(free, free)], cross[free])
    return k, K


def iterative_lqr(problem, initial_state, initial_controls=None, max_iterations=30,
                  tolerance=1e-6, regularization=1e-5,
                  state_damping=1., control_damping=1.):
    max_iterations = positive_int(max_iterations, "max_iterations")
    tolerance = positive(tolerance, "tolerance")
    regularization = positive(regularization, "regularization")
    damping_weights = np.array([state_damping, control_damping], dtype=float)
    if not np.isfinite(damping_weights).all() or (damping_weights < 0).any() or not damping_weights.any():
        raise ValueError("Damping weights must be nonnegative, finite, and not both zero")
    p = problem
    x0 = vector(initial_state, 12, "initial_state")
    if initial_controls is None:
        seeds = [p.reference_controls.copy()]
        try:
            seeds.append(p.seed(x0))
        except FloatingPointError:
            pass  # A failed feedback seed must not discard another finite seed.
    else:
        seeds = [np.asarray(initial_controls, dtype=float)]
    candidates = []
    for seed in seeds:
        try:
            us = p.clip(seed)
            xs = p.rollout(x0, us)
            candidates.append((p.cost(xs, us), xs, us))
        except FloatingPointError:
            continue
    if not candidates:
        raise ValueError("No finite initial rollout")
    cost, states, controls = min(candidates, key=lambda item: item[0])
    costs = [cost]
    damping = regularization
    converged, status = False, "maximum iterations reached"

    def backward(xs, us, reg):
        Vx = p.Qf @ (xs[-1] - p.reference_states[-1])
        Vxx = p.Qf.copy()
        ks = np.empty_like(us)
        Ks = np.empty((p.horizon, 6, 12))
        for t in range(p.horizon - 1, -1, -1):
            A, B = p.model.linearize(xs[t], us[t], discrete=True)
            qx = p.Q @ (xs[t] - p.reference_states[t]) + A.T @ Vx
            qu = p.R @ (us[t] - p.reference_controls[t]) + B.T @ Vx
            qxx = p.Q + A.T @ Vxx @ A
            quu = p.R + B.T @ Vxx @ B
            qux = B.T @ Vxx @ A
            damped = Vxx + reg * state_damping * np.eye(12)
            H = p.R + B.T @ damped @ B + reg * control_damping * np.eye(6)
            cross = B.T @ damped @ A
            k, K = box_policy(H, qu, cross, -p.model.limit - us[t], p.model.limit - us[t])
            ks[t], Ks[t] = k, K
            Vx = qx - K.T @ qu + qux.T @ k - K.T @ quu @ k
            Vxx = qxx - K.T @ qux - qux.T @ K + K.T @ quu @ K
            Vxx = (Vxx + Vxx.T) / 2
        return ks, Ks

    for iteration in range(1, max_iterations + 1):
        try:
            ks, gains = backward(states, controls, damping)
        except (np.linalg.LinAlgError, FloatingPointError):
            damping *= 10
            if damping > 1e10:
                status = "backward-pass regularization limit"
                break
            continue
        if np.max(np.abs(ks)) <= tolerance and damping <= regularization:
            converged, status = True, "stationary control update"
            break
        accepted = False
        for alpha in (1., .5, .25, .125, .0625, .03125, .015625):
            trial_x = np.empty_like(states)
            trial_u = np.empty_like(controls)
            trial_x[0] = x0
            try:
                for t in range(p.horizon):
                    trial_u[t] = p.clip(controls[t] + alpha * ks[t] - gains[t] @ (trial_x[t] - states[t]))
                    trial_x[t + 1] = p.model.step(trial_x[t], trial_u[t])
                trial_cost = p.cost(trial_x, trial_u)
            except FloatingPointError:
                continue
            if trial_cost < cost:
                improvement = cost - trial_cost
                cost, states, controls = trial_cost, trial_x, trial_u
                costs.append(cost)
                accepted = True
                if improvement <= tolerance * max(1., abs(cost)) and damping <= regularization:
                    converged, status = True, "relative cost tolerance"
                damping = max(regularization, damping / 10)
                break
        if converged:
            break
        if not accepted:
            damping *= 10
            if damping > 1e10:
                status = "line search failed"
                break
    try:
        _, gains = backward(states, controls, damping)
    except (np.linalg.LinAlgError, FloatingPointError):
        gains = np.zeros((p.horizon, 6, 12))
        converged, status = False, "final backward pass failed; feedforward plan only"
    return ILQRResult(states, controls, gains, costs, converged, iteration, status)


class RecedingHorizonILQR:
    def __init__(self, problem, **options):
        self.problem, self.options = problem, options
        self.last_result = None
        self.solve_count = self.nonconverged_count = 0

    def command(self, state):
        seed = None
        if self.last_result is not None:
            us = self.last_result.controls
            seed = np.concatenate((us[1:], us[-1:]))
        result = iterative_lqr(self.problem, state, initial_controls=seed, **self.options)
        self.last_result = result
        self.solve_count += 1
        self.nonconverged_count += int(not result.converged)
        return result.controls[0].copy()
