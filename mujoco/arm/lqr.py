"""Discrete finite/infinite-horizon LQR; Q and R are per-sample costs."""
import numpy as np
from scipy.linalg import expm, solve_discrete_are

from .model import positive, positive_int


def cost_matrix(value, size, name, definite=False):
    matrix = np.asarray(value, dtype=float)
    if matrix.ndim == 1:
        matrix = np.diag(matrix)
    if matrix.shape != (size, size) or not np.isfinite(matrix).all():
        raise ValueError(f"{name} must be a finite {size}x{size} matrix or diagonal")
    if not np.allclose(matrix, matrix.T, atol=1e-12, rtol=0):
        raise ValueError(f"{name} must be symmetric")
    eigenvalues = np.linalg.eigvalsh(matrix)
    if eigenvalues.min() < -1e-10 or (definite and eigenvalues.min() <= 0):
        raise ValueError(f"{name} must be positive {'definite' if definite else 'semidefinite'}")
    return matrix.copy()


def discretize(A, B, timestep, method="zoh"):
    dt = positive(timestep, "timestep")
    A, B = np.asarray(A, dtype=float), np.asarray(B, dtype=float)
    n, m = B.shape
    if A.shape != (n, n) or not np.isfinite(A).all() or not np.isfinite(B).all():
        raise ValueError("A and B must be compatible finite matrices")
    if method == "euler":
        return np.eye(n) + dt * A, dt * B
    if method != "zoh":
        raise ValueError("discretization must be euler or zoh")
    augmented = np.zeros((n + m, n + m))
    augmented[:n, :n], augmented[:n, n:] = A, B
    transition = expm(dt * augmented)
    return transition[:n, :n], transition[:n, n:]


def infinite_horizon_lqr(A, B, Q, R):
    Q = cost_matrix(Q, A.shape[0], "Q")
    R = cost_matrix(R, B.shape[1], "R", definite=True)
    P = solve_discrete_are(A, B, Q, R)
    K = np.linalg.solve(R + B.T @ P @ B, B.T @ P @ A)
    if np.max(np.abs(np.linalg.eigvals(A - B @ K))) >= 1:
        raise ValueError("The discrete LQR solution is not stabilizing")
    return K, P


def finite_horizon_lqr(A, B, Q, R, horizon, terminal_cost=None):
    horizon = positive_int(horizon, "horizon")
    Q = cost_matrix(Q, A.shape[0], "Q")
    R = cost_matrix(R, B.shape[1], "R", definite=True)
    P = cost_matrix(Q if terminal_cost is None else terminal_cost, A.shape[0], "terminal_cost")
    gains = np.empty((horizon, B.shape[1], A.shape[0]))
    values = np.empty((horizon + 1, A.shape[0], A.shape[0]))
    values[-1] = P
    for k in range(horizon - 1, -1, -1):
        K = np.linalg.solve(R + B.T @ P @ B, B.T @ P @ A)
        P = Q + A.T @ P @ (A - B @ K)
        P = (P + P.T) / 2
        gains[k], values[k] = K, P
    return gains, values
