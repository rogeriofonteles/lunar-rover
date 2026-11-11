"""Dynamically feasible initialization for difficult local iLQR problems."""
import math
import numpy as np
from pydrake.planning import DirectCollocation
from pydrake.solvers import Solve
from pydrake.trajectories import PiecewisePolynomial

if __package__:
    from .lqr import _riccati_step, wrapped_state_error
else:
    from lqr import _riccati_step, wrapped_state_error


def collocation_seed(plant, initial_state, target, timestep, horizon,
                     Q, R, terminal_cost, torque_limit, dynamics):
    """Find an endpoint-constrained trajectory, then track it on the RK4 grid.

    Collocation inputs are continuous between knots; the returned inputs are
    sample-and-hold torques, corrected by time-varying trajectory feedback.
    """
    if horizon < 2:
        raise ValueError("Collocation initialization requires horizon >= 2")
    duration = timestep * horizon
    samples = min(horizon + 1, 101, max(3, math.ceil(duration / .05) + 1))
    knot_dt = duration / (samples - 1)
    context = plant.CreateDefaultContext()
    problem = DirectCollocation(plant, context, samples, knot_dt, knot_dt)
    problem.AddEqualTimeIntervalsConstraints()
    program = problem.prog()
    # Choose the closest periodic representative of the upright target.
    endpoint = initial_state + wrapped_state_error(target, initial_state)
    program.AddBoundingBoxConstraint(initial_state, initial_state, problem.initial_state())
    program.AddBoundingBoxConstraint(endpoint, endpoint, problem.final_state())
    state = problem.state(); control = problem.input()
    if np.isfinite(torque_limit):
        problem.AddConstraintToAllKnotPoints(control[0] <= torque_limit)
        problem.AddConstraintToAllKnotPoints(control[0] >= -torque_limit)
    error = [state[i] - float(endpoint[i]) for i in range(4)]
    state_cost = sum(float(Q[i, j]) * error[i] * error[j]
                     for i in range(4) for j in range(4))
    # This is a feasibility seed, not the final objective. Keep the NLP
    # input curvature well scaled when iLQR uses a small torque penalty.
    problem.AddRunningCost(.5 * (state_cost + max(1., R) * control[0]**2))
    problem.SetInitialTrajectory(
        PiecewisePolynomial(),
        PiecewisePolynomial.FirstOrderHold([0., duration], np.column_stack((initial_state, endpoint))),
    )
    solution = Solve(program)
    if not solution.is_success():
        raise ValueError(f"Collocation initialization failed: {solution.get_solution_result()}")
    states = problem.ReconstructStateTrajectory(solution)
    inputs = problem.ReconstructInputTrajectory(solution)
    nominal = np.array([states.value(t*timestep).ravel() for t in range(horizon)])
    feedforward = np.array([inputs.value((t+.5)*timestep).item() for t in range(horizon)])
    value = terminal_cost.copy(); gains = np.empty((horizon, 1, 4)); eps = 1e-5
    for t in range(horizon-1, -1, -1):
        x, u = nominal[t], feedforward[t]
        if hasattr(dynamics,'jacobian'):A,B=dynamics.jacobian(x,u)
        else:
            A = np.column_stack([(dynamics(x+eps*v, u)-dynamics(x-eps*v, u))/(2*eps)
                                 for v in np.eye(4)])
            B = ((dynamics(x, u+eps)-dynamics(x, u-eps))/(2*eps)).reshape(4, 1)
        gains[t], value = _riccati_step(A, B, Q, R, value)
    controls = np.empty(horizon); x = initial_state.copy()
    for t in range(horizon):
        u = feedforward[t] - (gains[t] @ wrapped_state_error(x, nominal[t])).item()
        controls[t] = np.clip(u, -torque_limit, torque_limit)
        x = dynamics(x, controls[t])
    return controls
