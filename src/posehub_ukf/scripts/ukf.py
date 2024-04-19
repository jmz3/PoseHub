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

        # Set the state vector
        pt, qt, pt_prev, qt_prev = x[:3], x[3:7], x[7:10], x[10:14]

        # update the position based on the previous velocity
        pt_next = 2 * pt - pt_prev

        dqt = qt - qt_prev

        if np.linalg.norm(dqt) != 0:
            # normalize the quaternion difference
            dqt = dqt / np.linalg.norm(dqt)
            # update the orientation based on the previous angular velocity
            qt_next = Rot.from_quat(dqt).apply(qt)

        else:
            qt_next = qt

        return np.concatenate([pt_next, qt_next, pt, qt])

    def hx(self, x):
        """
        The measurement function, the measurement is the position and orientation of the target object
        relative to other objects

        Params:
        -------
        x: np.array
            The state vector

        Returns:
        --------
        np.array
            The measurement vector
        """

        return x[:7]
