# MIT License
#
# Copyright (c) [2024] [Jiaming Zhang]
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import numpy as np
from scipy.spatial.transform import Rotation as Rot
from ukf import UnscentedKalmanFilter as UKF


def main():
    # Define initial state
    x0 = np.array([0, 0, 0, 1, 0, 0, 0])  # Initial position and orientation

    # Define state covariance
    P = np.eye(6) * 0.1  # Initial estimate of state covariance

    # Define process noise covariance
    Q = np.eye(6) * 0.1  # Process noise

    # Define measurement noise covariance
    R = np.eye(6) * 0.1  # Observation noise (adjust based on sensor accuracy)

    # Initialize UKF
    ukf = UKF(num_sensors=6, init_pose=x0)

    # Example of how to use the UKF
    # for measurement in measurements: # measurements is a list of sensor readings
    #     ukf.predict()
    #     ukf.update(measurement)

    print(ukf.observation_relative(x0))


if __name__ == "__main__":
    main()
