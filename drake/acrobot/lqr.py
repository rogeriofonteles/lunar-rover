"""Discrete LQR design for the Acrobot near its upright equilibrium."""

import math
import operator
from pathlib import Path
import numpy as np


def wrapped_state_error(state, equilibrium_state):
    """Return state deviations with angular errors in [-pi, pi).

    Supports a single state or batches (..., 4); velocities are unchanged.
    Plant coordinates may accumulate revolutions; feedback uses periodic errors.
    """
    error = np.asarray(state, dtype=float) - np.asarray(equilibrium_state, dtype=float)
    error[..., :2] = (error[..., :2] + np.pi) % (2 * np.pi) - np.pi
    return error


def load_torque_limit(config_path=None):
    """Read the positive torque magnitude limit (N m) from YAML."""
    import yaml

    path = Path(__file__).with_name("lqr.yaml") if config_path is None else Path(config_path)
    with path.open(encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    if not isinstance(config, dict) or "torque_limit_nm" not in config:
        raise ValueError("LQR YAML must contain torque_limit_nm")
    return _torque_limit(config["torque_limit_nm"])


def _torque_limit(value):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError("torque_limit_nm must be positive and finite")
    if not np.isfinite(value) or value <= 0:
        raise ValueError("torque_limit_nm must be positive and finite")
    return float(value)


def control_torque(gain, state_error, torque_limit=None):
    """Compute u = -K x_error, optionally saturating to +/- torque_limit N m.

    Pass the result of load_torque_limit() to enable clipping, or None for
    unconstrained feedback. Saturation does not change the LQR gain design.
    """
    torque = float(np.asarray(-gain @ state_error).item())
    if torque_limit is not None:
        limit = _torque_limit(torque_limit)
        torque = float(np.clip(torque, -limit, limit))
    return torque


def _positive_integer(value, name):
    try:
        value = operator.index(value)
    except TypeError:
        raise ValueError(f"{name} must be a positive integer") from None
    if value < 1:
        raise ValueError(f"{name} must be a positive integer")
    return value


def _state_cost(value, name):
    value = np.diag([10., 10., 1., 1.]) if value is None else np.asarray(value, dtype=float)
    if value.shape != (4, 4) or not np.all(np.isfinite(value)):
        raise ValueError(f"{name} must be a finite 4x4 matrix")
    if not np.allclose(value, value.T) or np.linalg.eigvalsh(value).min() < -1e-10:
        raise ValueError(f"{name} must be symmetric positive semidefinite")
    return value


def _linear_model(plant, timestep, Q, R, equilibrium_state):
    if not np.isfinite(timestep) or timestep <= 0:
        raise ValueError("timestep must be positive and finite")
    if not np.isfinite(R) or R <= 0:
        raise ValueError("R must be positive and finite")
    Q = _state_cost(Q, "Q")
    equilibrium_state = np.asarray(
        [math.pi, 0., 0., 0.] if equilibrium_state is None else equilibrium_state,
        dtype=float,
    )
    if equilibrium_state.shape != (4,) or not np.all(np.isfinite(equilibrium_state)):
        raise ValueError("equilibrium_state must contain four finite values")
    context = plant.CreateDefaultContext()
    input_port = plant.get_input_port(0)
    input_port.FixValue(context, [0.])

    def derivatives(state, torque):
        context.SetContinuousState(state)
        input_port.FixValue(context, [torque])
        return plant.EvalTimeDerivatives(context).CopyToVector()

    # Central finite differences of x_dot = f(x, u) at the upright equilibrium.
    epsilon = 1e-6
    A_continuous = np.empty((4, 4))
    for i in range(4):
        basis_vector = np.zeros(4)  # 1 at index i, 0 everywhere else.
        basis_vector[i] = 1.0
        delta = epsilon * basis_vector
        A_continuous[:, i] = (
            derivatives(equilibrium_state + delta, 0.)
            - derivatives(equilibrium_state - delta, 0.)
        ) / (2. * epsilon)
    B_continuous = (
        derivatives(equilibrium_state, epsilon)
        - derivatives(equilibrium_state, -epsilon)
    ) / (2. * epsilon)

    A = np.eye(4) + timestep * A_continuous
    B = timestep * B_continuous.reshape(4, 1)

    return A, B, Q


def _riccati_step(A, B, Q, R, value_matrix):
    gain = np.linalg.solve(np.array([[R]]) + B.T @ value_matrix @ B,
                           B.T @ value_matrix @ A)
    value = Q + A.T @ value_matrix @ A - A.T @ value_matrix @ B @ gain
    return gain, 0.5 * (value + value.T)


def finite_horizon_lqr(plant, timestep=0.1, horizon=10, Q=None, R=1.0,
                       equilibrium_state=None, terminal_cost=None,
                       return_sequence=False):
    """Solve a finite plan with terminal cost Q_f (defaults to Q).

    The cost is sum(x[k].T Q x[k] + u[k].T R u[k]) + x[N].T Q_f x[N].
    Set return_sequence=True for gains K[0], ..., K[N-1], shape (N, 1, 4).
    The default returns K[0], shape (1, 4), for receding-horizon use and
    compatibility with the comparison scripts. Forward Euler discretization
    uses timestep seconds. States represent deviations from equilibrium.
    """
    horizon = _positive_integer(horizon, "horizon")
    A, B, Q = _linear_model(plant, timestep, Q, R, equilibrium_state)
    value = Q.copy() if terminal_cost is None else _state_cost(terminal_cost, "terminal_cost")
    gains = np.empty((horizon, 1, 4))
    for step in range(horizon - 1, -1, -1):
        gains[step], value = _riccati_step(A, B, Q, R, value)
    return gains if return_sequence else gains[0]


def infinite_horizon_lqr(plant, timestep=0.1, Q=None, R=1.0,
                         equilibrium_state=None, tolerance=1e-9,
                         max_iterations=10000):
    """Compute a constant discrete LQR gain by Riccati value iteration."""
    if not np.isfinite(tolerance) or tolerance <= 0:
        raise ValueError("tolerance must be positive and finite")
    max_iterations = _positive_integer(max_iterations, "max_iterations")
    A, B, Q = _linear_model(plant, timestep, Q, R, equilibrium_state)
    value = Q.copy()
    previous_gain = None
    for _ in range(max_iterations):
        gain, value = _riccati_step(A, B, Q, R, value)
        if previous_gain is not None and np.max(np.abs(gain - previous_gain)) < tolerance:
            return gain
        previous_gain = gain
    raise RuntimeError(f"Infinite-horizon LQR did not converge in {max_iterations} iterations")


def controller_gains(plant, mode="infinite", timestep=0.1, horizon=10,
                     Q=None, R=1.0, equilibrium_state=None, terminal_cost=None):
    """Return a gain schedule: constant modes have length 1.

    With fixed dynamics, costs and horizon, each receding-horizon solve has
    the same first gain, so it only needs to be computed once.
    """
    kwargs = dict(timestep=timestep, Q=Q, R=R, equilibrium_state=equilibrium_state)
    if mode == "infinite":
        return infinite_horizon_lqr(plant, **kwargs)[None, ...]
    if mode in ("finite", "receding"):
        gains = finite_horizon_lqr(plant, horizon=horizon, terminal_cost=terminal_cost,
                                   return_sequence=True, **kwargs)
        return gains if mode == "finite" else gains[:1]
    raise ValueError("mode must be infinite, finite, or receding")
