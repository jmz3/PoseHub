import numpy as np
from scipy.spatial.transform import Rotation as R


def hat(omega):
    """Return the hat operator of a 3-vector."""
    return np.array(
        [
            [0, -omega[2], omega[1]],
            [omega[2], 0, -omega[0]],
            [-omega[1], omega[0], 0],
        ]
    )


def se3_exp(x):
    """
    Exponential map for SE(3).
    x: 6-vector, with x[0:3] = rotation (angle-axis) and x[3:6] = translation.
    Returns a 4x4 transformation matrix.
    """
    omega = x[0:3]
    v = x[3:6]
    theta = np.linalg.norm(omega)
    if theta < 1e-8:
        R_mat = np.eye(3)
        V = np.eye(3)
    else:
        R_mat = R.from_rotvec(omega).as_matrix()
        omega_hat = hat(omega)
        V = (
            np.eye(3)
            + (1 - np.cos(theta)) / (theta**2) * omega_hat
            + (theta - np.sin(theta)) / (theta**3) * (omega_hat @ omega_hat)
        )
    T = np.eye(4)
    T[:3, :3] = R_mat
    T[:3, 3] = V @ v
    return T


def se3_log(T):
    """
    Logarithm map for SE(3).
    Returns a 6-vector: first 3 for rotation, last 3 for translation.
    """
    R_mat = T[:3, :3]
    t = T[:3, 3]
    cos_theta = (np.trace(R_mat) - 1) / 2
    cos_theta = np.clip(cos_theta, -1.0, 1.0)
    theta = np.arccos(cos_theta)
    if abs(theta) < 1e-8:
        omega = np.zeros(3)
        V_inv = np.eye(3)
    else:
        omega = (
            theta
            / (2 * np.sin(theta))
            * np.array(
                [
                    R_mat[2, 1] - R_mat[1, 2],
                    R_mat[0, 2] - R_mat[2, 0],
                    R_mat[1, 0] - R_mat[0, 1],
                ]
            )
        )
        omega_hat = hat(omega)
        V_inv = (
            np.eye(3)
            - 0.5 * omega_hat
            + (1 / (theta**2) - (1 + np.cos(theta)) / (2 * theta * np.sin(theta)))
            * (omega_hat @ omega_hat)
        )
    v = V_inv @ t
    return np.concatenate((omega, v))
