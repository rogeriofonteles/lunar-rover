"""Finite-horizon nonlinear iLQR with regularized backward passes."""
from dataclasses import dataclass
import math
import warnings
import numpy as np

if __package__:
    from .lqr import _positive_integer, _riccati_step, _state_cost, _torque_limit, wrapped_state_error
else:
    from lqr import _positive_integer, _riccati_step, _state_cost, _torque_limit, wrapped_state_error


@dataclass
class ILQRResult:
    states: np.ndarray
    controls: np.ndarray
    gains: np.ndarray  # Negative-feedback convention: u = u_nominal - K dx.
    costs: list
    converged: bool
    iterations: int
    status: str
    terminal_cost: np.ndarray
    initialization: str = "provided"
    initial_controls: np.ndarray | None = None


def nonlinear_step(plant, timestep):
    """Return an RK4 discrete model with internal steps no longer than 10 ms."""
    context = plant.CreateDefaultContext()
    port = plant.get_input_port(0)
    count = max(1, math.ceil(timestep / .01))
    dt = timestep / count

    def derivative(x, u):
        context.SetContinuousState(x)
        port.FixValue(context, [float(u)])
        return plant.EvalTimeDerivatives(context).CopyToVector()

    def step(x, u):
        x = np.asarray(x, dtype=float).copy()
        with np.errstate(over="raise", invalid="raise"):
            for _ in range(count):
                a = derivative(x, u)
                b = derivative(x + dt * a / 2, u)
                c = derivative(x + dt * b / 2, u)
                d = derivative(x + dt * c, u)
                x += dt * (a + 2*b + 2*c + d) / 6
        if not np.all(np.isfinite(x)):
            raise FloatingPointError("Non-finite nonlinear rollout")
        return x
    # Exact derivatives of this RK4 discretization, shared by all nonlinear solvers.
    # Create the AutoDiff plant lazily: plain rollouts continue to use doubles.
    ad_cache = []
    def jacobian(x, u):
        from pydrake.autodiffutils import InitializeAutoDiff, ExtractGradient
        if not ad_cache:
            ad_plant = plant.ToAutoDiffXd()
            ad_cache.extend([ad_plant, ad_plant.CreateDefaultContext(), ad_plant.get_input_port(0)])
        ad_plant, ad_context, ad_port = ad_cache
        z = InitializeAutoDiff(np.r_[x, u]).ravel()
        state, torque = z[:4], z[4]
        def f(q):
            ad_context.SetContinuousState(q); ad_port.FixValue(ad_context, [torque])
            return ad_plant.EvalTimeDerivatives(ad_context).CopyToVector()
        for _ in range(count):
            a=f(state); b=f(state+dt*a/2); c=f(state+dt*b/2); d=f(state+dt*c)
            state=state+dt*(a+2*b+2*c+d)/6
        J=ExtractGradient(state)
        if not np.isfinite(J).all():raise FloatingPointError('Non-finite RK4 Jacobian')
        return J[:,:4],J[:,4:5]
    step.jacobian = jacobian
    return step


def iterative_lqr(plant, initial_state, timestep=.01, horizon=100, Q=None, R=1.,
                  terminal_cost=None, equilibrium_state=None, torque_limit=None,
                  max_iterations=50, tolerance=1e-6, regularization=1e-5,
                  state_damping=1., control_damping=1., initial_controls=None, initialization="auto",
                  initialize_only=False):
    """Optimize a nonlinear finite plan, returning its trajectory and feedback.

    Damping scales lambda I in the value-state Hessian and control Q_uu
    Hessian, respectively; it is numerical damping, not physical friction.
    Scalar torque bounds are enforced in both the backward pass and rollout.
    The default terminal cost is the local DARE value matrix (Q fallback
    when a stabilizing local solution is unavailable). Initial controls use
    the lowest-cost zero/local-LQR rollout, plus a collocation candidate in
    auto mode for angular errors above 0.25 rad. Explicit controls bypass seeds.
    costs records only accepted costs. Nonconvergence is reported explicitly.
    """
    if initialization not in ("auto", "lqr", "collocation"):
        raise ValueError("initialization must be auto, lqr, or collocation")
    horizon = _positive_integer(horizon, "horizon")
    max_iterations = _positive_integer(max_iterations, "max_iterations")
    for name, value in (("timestep", timestep), ("R", R), ("tolerance", tolerance),
                        ("regularization", regularization)):
        if not np.isfinite(value) or value <= 0:
            raise ValueError(f"{name} must be positive and finite")
    for value in (state_damping, control_damping):
        if not np.isfinite(value) or value < 0:
            raise ValueError("damping must be nonnegative and finite")
    if state_damping + control_damping == 0:
        raise ValueError("at least one damping coefficient must be positive")
    Q = _state_cost(Q, "Q")
    Qf = Q if terminal_cost is None else _state_cost(terminal_cost, "terminal_cost")
    target = np.asarray([math.pi, 0, 0, 0] if equilibrium_state is None else equilibrium_state, dtype=float)
    x0 = np.asarray(initial_state, dtype=float)
    for x in (target, x0):
        if x.shape != (4,) or not np.all(np.isfinite(x)):
            raise ValueError("states must contain four finite values")
    bound = np.inf if torque_limit is None else _torque_limit(torque_limit)
    controls = np.zeros(horizon) if initial_controls is None else np.asarray(initial_controls, dtype=float).copy()
    if controls.shape != (horizon,) or not np.all(np.isfinite(controls)):
        raise ValueError("initial_controls must have shape (horizon,) and be finite")
    controls = np.clip(controls, -bound, bound)
    dynamics = nonlinear_step(plant, timestep)

    def rollout(us, nominal=None, feedforward=None, gains=None, alpha=1.):
        xs = np.empty((horizon+1, 4)); xs[0] = x0
        applied = us.copy()
        cost = 0.
        for t in range(horizon):
            if nominal is not None:
                dx = wrapped_state_error(xs[t], nominal[t])
                applied[t] += alpha * feedforward[t] - (gains[t] @ dx).item()
            applied[t] = np.clip(applied[t], -bound, bound)
            e = wrapped_state_error(xs[t], target)
            with np.errstate(over="raise", invalid="raise"):
                cost += .5 * (e @ Q @ e + R * applied[t]**2)
            if not np.isfinite(cost):
                raise FloatingPointError("Non-finite rollout cost")
            xs[t+1] = dynamics(xs[t], applied[t])
        e = wrapped_state_error(xs[-1], target)
        with np.errstate(over="raise", invalid="raise"):
            cost = float(cost + .5 * e @ Qf @ e)
        if not np.isfinite(cost):
            raise FloatingPointError("Non-finite terminal cost")
        return xs, applied, cost

    def backward(xs, us, damping):
        # Jacobians of the actual discrete nonlinear rollout, not Euler dynamics.
        As = []; Bs = []
        for x, u in zip(xs[:-1], us):
            A,B = dynamics.jacobian(x,u)
            As.append(A); Bs.append(B)
        Vx = Qf @ wrapped_state_error(xs[-1], target); Vxx = Qf.copy()
        ks = np.zeros(horizon); gains = np.zeros((horizon, 1, 4))
        for t in range(horizon-1, -1, -1):
            A, B = As[t], Bs[t]
            qx = Q @ wrapped_state_error(xs[t], target) + A.T @ Vx
            qu = R*us[t] + (B.T @ Vx).item()
            qxx = Q + A.T @ Vxx @ A
            quu = R + (B.T @ Vxx @ B).item()
            qux = B.T @ Vxx @ A
            damped = Vxx + damping*state_damping*np.eye(4)
            huu = R + (B.T @ damped @ B).item() + damping*control_damping
            hux = B.T @ damped @ A
            if not np.isfinite(huu) or huu <= 0:
                raise np.linalg.LinAlgError("Control Hessian is not positive definite")
            raw = -qu/huu
            k = np.clip(raw, -bound-us[t], bound-us[t])
            K = hux/huu if k == raw else np.zeros((1, 4))
            ks[t], gains[t] = k, K
            # Propagate the original quadratic model under the damped policy.
            L = -K
            Vx = qx + L.ravel()*qu + qux.ravel()*k + L.ravel()*quu*k
            Vxx = qxx + L.T @ qux + qux.T @ L + quu*(L.T @ L)
            Vxx = (Vxx + Vxx.T)/2
        return ks, gains

    # Zero torque can fall far from upright before optimization even starts.
    # Compare it with a stabilizing local feedback rollout, using a DARE gain
    # for the same RK4 dynamics. Never replace an explicitly supplied seed.
    seeds = [("zero" if initial_controls is None else "provided", controls)]
    if initial_controls is None or terminal_cost is None:
        try:
            A,B = dynamics.jacobian(target,0.)
            value = Q.copy()
            for _ in range(10000):
                gain, updated = _riccati_step(A, B, Q, R, value)
                if np.max(np.abs(updated-value)) <= 1e-10*max(1., np.max(np.abs(value))):
                    break
                value = updated
            else:
                raise np.linalg.LinAlgError("Seed Riccati iteration did not converge")
            if np.max(np.abs(np.linalg.eigvals(A-B@gain))) >= 1:
                raise np.linalg.LinAlgError("Seed feedback is not stabilizing")
            if terminal_cost is None:
                # Approximate the cost of balancing after the finite endpoint.
                Qf = updated
            if initial_controls is None:
                seed = np.empty(horizon); x = x0.copy()
                for t in range(horizon):
                    seed[t] = np.clip(-(gain @ wrapped_state_error(x, target)).item(), -bound, bound)
                    x = dynamics(x, seed[t])
                seeds.append(("lqr", seed))
        except (np.linalg.LinAlgError, FloatingPointError, OverflowError):
            pass  # A local stabilizing seed may not exist for a general target.
    if initial_controls is None and (initialization == "collocation" or
            (initialization == "auto" and np.max(np.abs(wrapped_state_error(x0, target)[:2])) > .25)):
        try:
            if __package__:
                from .trajectory_seed import collocation_seed
            else:
                from trajectory_seed import collocation_seed
            seed = collocation_seed(plant, x0, target, timestep, horizon, Q, R, Qf, bound, dynamics)
            seeds.append(("collocation", seed))
        except (RuntimeError, ValueError, FloatingPointError, np.linalg.LinAlgError) as error:
            if initialization == "collocation":
                raise ValueError(f"Cannot initialize iLQR: {error}") from error
            warnings.warn(f"Collocation seed unavailable; using local seeds: {error}", RuntimeWarning)
    candidates = []
    for label, seed in seeds:
        try:
            candidate = rollout(seed)
            if np.isfinite(candidate[2]):
                candidates.append((label, candidate))
        except (FloatingPointError, OverflowError):
            continue
    if not candidates:
        raise ValueError("No finite initial rollout; supply initial_controls or reduce timestep/horizon")
    seed_label, (states, controls, cost) = min(candidates, key=lambda candidate: candidate[1][2])
    costs = [cost]; damping = regularization
    converged = False; status = "maximum iterations reached"
    gains = np.zeros((horizon, 1, 4))
    shared_seed=controls.copy()
    if initialize_only:
        return ILQRResult(states,controls,gains,costs,False,0,'initialization only',Qf.copy(),seed_label,shared_seed)
    for iteration in range(1, max_iterations+1):
        try:
            ks, candidate_gains = backward(states, controls, damping)
        except (np.linalg.LinAlgError, FloatingPointError):
            damping *= 10
            if damping > 1e12:
                status = "regularization limit reached"; break
            continue
        if np.max(np.abs(ks)) < tolerance and damping <= regularization:
            converged = True; status = "stationary control update"; gains = candidate_gains; break
        accepted = False
        for alpha in (1., .5, .25, .125, .0625, .03125, .015625):
            try:
                xs, us, candidate_cost = rollout(controls, states, ks, candidate_gains, alpha)
            except (FloatingPointError, OverflowError):
                continue
            if np.isfinite(candidate_cost) and candidate_cost < cost:
                improvement = cost-candidate_cost
                states, controls, cost = xs, us, candidate_cost
                costs.append(cost); gains = candidate_gains
                accepted = True
                if improvement <= tolerance * max(1., abs(cost)):
                    converged = True; status = "relative cost tolerance"
                damping = max(regularization, damping/10)
                break
        if converged:
            break
        if not accepted:
            damping *= 10
            if damping > 1e12:
                status = "line search failed"; break
    # Feedback must correspond to the final accepted trajectory.
    try:
        _, gains = backward(states, controls, damping)
    except (np.linalg.LinAlgError, FloatingPointError):
        status = "final backward pass failed"; converged = False
        gains = np.zeros_like(gains)
    return ILQRResult(states, controls, gains, costs, converged, iteration, status, Qf.copy(), seed_label,shared_seed)


class RecedingHorizonILQR:
    """Replan from each measured state and apply only the first control.

    The next solve shifts the accepted control sequence by one sample and
    repeats its last torque at the tail. A nonconverged solve still provides
    its best finite accepted plan; callers can inspect last_result. A solve
    with no finite rollout raises rather than applying stale controls.
    """

    def __init__(self, plant, **solver_options):
        if "initial_controls" in solver_options:
            raise ValueError("Receding controller manages its own initial_controls")
        self.plant = plant
        self.solver_options = dict(solver_options)
        self.last_result = None
        self.solve_count = 0
        self.nonconverged_count = 0
        self._terminal_cost = solver_options.get("terminal_cost")

    def command(self, measured_state):
        seed = None
        if self.last_result is not None:
            us = self.last_result.controls
            seed = np.concatenate((us[1:], us[-1:]))
        options = dict(self.solver_options)
        options["terminal_cost"] = self._terminal_cost
        result = iterative_lqr(self.plant, measured_state,
                               initial_controls=seed, **options)
        self.last_result = result
        self._terminal_cost = result.terminal_cost
        self.solve_count += 1
        self.nonconverged_count += int(not result.converged)
        return float(result.controls[0])
