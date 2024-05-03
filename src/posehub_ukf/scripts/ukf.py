#!usr/bin/env python3

import numpy as np
from scipy.spatial.transform import Rotation as Rot
from filterpy.kalman import UnscentedKalmanFilter as UKF, MerweScaledSigmaPoints


class UnscentedKalmanFilter:
    def __init__(self, num_sensors: int, init_pose: np.array):
        """
        Initialize the UKF for one relative pose,
        The pose is represented by the position and orientation


        Params:
        -------
        num_sensors: int,
            The number of sensors in the system

        init_pose: np.array
            The initial pose of the target object,
            init_state = [ p(1) : 1x3
                           q(1) : 1x4 ]
            To ensure the fused pose donot drift largely from the actual pose,
            initialize the pose using one of the sensor's measurement

        """
        self.num_sensors = num_sensors
        self.points = MerweScaledSigmaPoints(14, alpha=0.1, beta=2.0, kappa=-1)
        self.ukf = UKF(
            dim_x=14,
            dim_z=7 * num_sensors,
            dt=0.01,
            fx=self.fx_relative,
            hx=self.observation_relative,
            points=self.points,
        )

        # Set the initial state
        # Check if the initial pose is valid
        if init_pose.shape != (7,) or not isinstance(init_pose, np.ndarray):
            raise ValueError("The initial pose should be a 1x7 numpy array")

        self.ukf.x = np.concatenate([init_pose, np.zeros(7)])

    def fx_relative(self, x, dt):
        """
        The state transition function,
        The position and orientation are updated based on the same velocity of the previous state

        this state transition function is used to predict the next state of the target object
        which is not so important because the measurement function is used to update the state

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
            # update the orientation based on the previous angular velocity
            qt_next = (Rot.from_quat(qt) * Rot.from_quat(dqt)).as_quat()

        else:
            qt_next = qt

        return np.concatenate([pt_next, qt_next, pt, qt])

    def observation_relative(self, x):
        """
        The measurement function, the measurement is the position and orientation of the target object
        relative to other objects

        Given any state vector, we can only measure the position and orientation at the current time

        Params:
        -------
        x: np.array
            The state vector

        Returns:
        --------
        np.array
            The measurement vector
        """
        observation = np.tile(x[:7], self.num_sensors)

        return observation

    def predict(self, measurement: np.array):
        """
        Predict the next state of the target object
        """
        self.ukf.predict()
        self.ukf.update(measurement)
