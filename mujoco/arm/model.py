"""Deterministic, contact-free MuJoCo dynamics for a fully actuated arm."""
from pathlib import Path
import math

import mujoco
import numpy as np


def positive(value, name):
    if isinstance(value, bool) or not np.isscalar(value) or not np.isfinite(value) or value <= 0:
        raise ValueError(f"{name} must be positive and finite")
    return float(value)


def positive_int(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, np.integer)) or value < 1:
        raise ValueError(f"{name} must be a positive integer")
    return int(value)


def vector(value, size, name):
    result = np.asarray(value, dtype=float)
    if result.shape != (size,) or not np.isfinite(result).all():
        raise ValueError(f"{name} must contain {size} finite values")
    return result.copy()


def jacobian(function, x, epsilon=1e-5):
    columns = []
    for i in range(len(x)):
        delta = np.zeros_like(x)
        delta[i] = epsilon * max(1., abs(x[i]))
        columns.append((function(x + delta) - function(x - delta)) / (2 * delta[i]))
    return np.column_stack(columns)


class ArmModel:
    nx, nu = 12, 6

    def __init__(self, timestep=.02, physics_timestep=.002, torque_limit=80.):
        self.dt = positive(timestep, "timestep")
        self.substeps = max(1, math.ceil(self.dt / positive(physics_timestep, "physics_timestep")))
        self.model = mujoco.MjModel.from_xml_path(str(Path(__file__).parent / "models/arm6.xml"))
        self.model.opt.timestep = self.dt / self.substeps
        self.limit = np.broadcast_to(np.asarray(torque_limit, dtype=float), (6,)).copy()
        if not np.isfinite(self.limit).all() or (self.limit <= 0).any():
            raise ValueError("torque_limit must be a positive scalar or six positive values")
        self.model.actuator_ctrlrange[:] = np.column_stack((-self.limit, self.limit))
        self.data = mujoco.MjData(self.model)

    def _set(self, x, u):
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[:] = x[:6]
        self.data.qvel[:] = x[6:]
        self.data.ctrl[:] = u

    def derivative(self, x, u):
        self._set(x, u)
        mujoco.mj_forward(self.model, self.data)
        return np.concatenate((self.data.qvel, self.data.qacc))

    def step(self, x, u, external_torque=None):
        self._set(x, u)
        if external_torque is not None:
            self.data.qfrc_applied[:] = vector(external_torque, 6, "external_torque")
        for _ in range(self.substeps):
            mujoco.mj_step(self.model, self.data)
        result = np.concatenate((self.data.qpos, self.data.qvel))
        if not np.isfinite(result).all() or any(w.number for w in self.data.warning):
            raise FloatingPointError("MuJoCo rollout failed; reduce the step or control magnitude")
        return result

    def linearize(self, x, u, discrete=False):
        # At a torque bound use the smooth unconstrained motor dynamics for
        # derivatives; optimizer box constraints separately enforce saturation.
        limited = self.model.actuator_ctrllimited.copy()
        self.model.actuator_ctrllimited[:] = False
        try:
            f = self.step if discrete else self.derivative
            return jacobian(lambda z: f(z, u), x), jacobian(lambda v: f(x, v), u)
        finally:
            self.model.actuator_ctrllimited[:] = limited

    def equilibrium_control(self, target):
        target = vector(target, 12, "target_state")
        if np.max(np.abs(target[6:])) > 1e-12:
            raise ValueError("A fixed target must have zero joint velocities")
        self._set(target, np.zeros(6))
        mujoco.mj_forward(self.model, self.data)
        torque = self.data.qfrc_bias.copy() - self.data.qfrc_passive
        if np.any(np.abs(torque) >= self.limit):
            raise ValueError("Target gravity compensation requires torque at or beyond the limit")
        if np.linalg.norm(self.derivative(target, torque), ord=np.inf) > 1e-7:
            raise ValueError("Target is not an equilibrium of this arm")
        return torque
