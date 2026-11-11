"""Shared equilibrium, objective, and nonlinear rollouts."""
import numpy as np

from .lqr import cost_matrix, discretize, infinite_horizon_lqr
from .model import positive_int, vector


class ControlProblem:
    def __init__(self, model, target, horizon=60, Q=None, R=None,
                 terminal_cost=None, discretization="zoh"):
        self.model = model
        self.target = vector(target, 12, "target_state")
        self.horizon = positive_int(horizon, "horizon")
        self.Q = cost_matrix([50.] * 6 + [2.] * 6 if Q is None else Q, 12, "Q")
        self.R = cost_matrix([.05] * 6 if R is None else R, 6, "R", definite=True)
        self.u_ref = model.equilibrium_control(self.target)
        Ac, Bc = model.linearize(self.target, self.u_ref)
        self.A, self.B = discretize(Ac, Bc, model.dt, discretization)
        self.K, P = infinite_horizon_lqr(self.A, self.B, self.Q, self.R)
        self.Qf = P if terminal_cost is None else cost_matrix(terminal_cost, 12, "terminal_cost")
        self.reference_states = np.tile(self.target, (self.horizon+1, 1))
        self.reference_controls = np.tile(self.u_ref, (self.horizon, 1))

    def clip(self, controls):
        return np.clip(controls, -self.model.limit, self.model.limit)

    def cost(self, states, controls):
        e = states - self.reference_states
        v = controls - self.reference_controls
        result = .5 * (np.einsum("ti,ij,tj->", e[:-1], self.Q, e[:-1])
                       + np.einsum("ti,ij,tj->", v, self.R, v)
                       + e[-1] @ self.Qf @ e[-1])
        if not np.isfinite(result):
            raise FloatingPointError("Nonfinite trajectory cost")
        return float(result)

    def rollout(self, initial_state, controls):
        controls = np.asarray(controls, dtype=float)
        if controls.shape != (self.horizon, 6) or not np.isfinite(controls).all():
            raise ValueError("controls must have shape (horizon, 6) and be finite")
        states = np.empty((self.horizon + 1, 12))
        states[0] = vector(initial_state, 12, "initial_state")
        for k, u in enumerate(controls):
            states[k + 1] = self.model.step(states[k], self.clip(u))
        return states

    def seed(self, initial_state):
        x = vector(initial_state, 12, "initial_state")
        controls = np.empty((self.horizon, 6))
        for k in range(self.horizon):
            controls[k] = self.clip(self.reference_controls[k] - self.K @ (x - self.reference_states[k]))
            x = self.model.step(x, controls[k])
        return controls
