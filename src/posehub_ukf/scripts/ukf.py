#!usr/bin/env python3

import numpy as np
from scipy.spatial.transform import Rotation as Rot


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
