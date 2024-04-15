#!usr/bin/env python3

import numpy as np
from scipy.spatial.transform import Rotation as Rot
from filterpy.kalman import UnscentedKalmanFilter as UKF, MerweScaledSigmaPoints


class UnscentedKalmanFilter:
    def __init__(self, n, m, q, r, x0, p0):
        """
        Initialize the UKF class

        Params:
        -------
        n: int
            The number of states
        m: int
            The number of measurements
        q: float
            The process noise
        r: float
            The measurement noise
        x0: np.array
            The initial state
        p0: np.array
            The initial covariance matrix

        """

    def fx(self, x, dt):
        """
        The state transition function,
        The position and orientation are updated based on the same velocity of the previous state


        Params:
        -------
        x: np.array
            The state vector, x = [ p(t)    : 1x3
                                    q(t)    : 1x4
                                    p(t-1)  : 1x3
                                    q(t-1)  : 1x4   ]
        dt: float
            The time step

        Returns:
        --------
        np.array
            The updated state vector
        """
        x_transit = np.zeros_like(x)
        pt = x[:3]
        qt = x[3:7]
        pt_prev = x[7:10]
        qt_prev = x[10:14]

        # update the position
        pt_next = 2 * pt - pt_prev
        qt_next = qt + (qt - qt_prev) / np.linalg.norm(qt - qt_prev)

        # normalize the quaternion
        # if qt_next is too small, then set it to qt
        qt_next = (
            qt_next / np.linalg.norm(qt_next) if np.linalg.norm(qt_next) != 0 else qt
        )

        return np.concatenate([pt_next, qt_next, pt, qt])
