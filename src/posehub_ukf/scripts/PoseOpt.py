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
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt


def main():
    # Define initial state
    x0 = np.array([0, 0, 0, 0, 0, 0, 1])  # Initial position and orientation

    # Define state covariance
    P = np.eye(14) * 0.1  # Initial estimate of state covariance

    # Define process noise covariance
    Q = np.eye(14) * 0.1  # Process noise

    # Define measurement noise covariance
    R_sole = np.eye(7) * 0.1  # Measurement noise for the sole sensor
    R_sole[:3, :3] = np.eye(3) * 10  # Position noise
    R_sole[3:, 3:] = np.eye(4) * 0.05  # Orientation noise

    R = np.block(
        [
            [R_sole, np.zeros((7, 5 * 7))],
            [np.zeros((7, 7)), R_sole, np.zeros((7, 4 * 7))],
            [np.zeros((7, 2 * 7)), R_sole, np.zeros((7, 3 * 7))],
            [np.zeros((7, 3 * 7)), R_sole, np.zeros((7, 2 * 7))],
            [np.zeros((7, 4 * 7)), R_sole, np.zeros((7, 7))],
            [np.zeros((7, 5 * 7)), R_sole],
        ],
    )

    # repeat the measurement noise for the other sensors and stack them

    # Initialize UKF
    ukf = UKF(num_sensors=6, init_pose=x0)

    ukf.ukf.P = P
    ukf.ukf.Q = Q
    ukf.ukf.R = R

    # Example of how to use the UKF
    # for measurement in measurements: # measurements is a list of sensor readings
    #     ukf.predict()
    #     ukf.update(measurement)

    states = dict(gt=dict(), fuse=dict())
    states["gt"]["x"] = []
    states["gt"]["y"] = []
    states["gt"]["z"] = []
    states["gt"]["qx"] = []
    states["gt"]["qy"] = []
    states["gt"]["qz"] = []
    states["gt"]["qw"] = []

    states["fuse"]["x"] = []
    states["fuse"]["y"] = []
    states["fuse"]["z"] = []
    states["fuse"]["qx"] = []
    states["fuse"]["qy"] = []
    states["fuse"]["qz"] = []
    states["fuse"]["qw"] = []

    p = np.array([0, 0, 0], dtype=np.float64)  # Initial position
    angle_deg = np.array([0, 0, 0], dtype=np.float64)  # Initial orientation in degrees

    observations = np.zeros(7 * 6, dtype=np.float64)  # Initial observations

    # Simulate some states
    for i in range(100):
        # Simulate the actual state of the object
        p += np.array([1.0, 2.0, 2.5])  # Add noise to the position
        angle_deg += np.array([0.08, 0.05, 0.10])
        q = Rot.from_euler(
            "xyz",
            angle_deg,
        ).as_quat()  # Add noise to the orientation

        states["gt"]["x"].append(p[0])
        states["gt"]["y"].append(p[1])
        states["gt"]["z"].append(p[2])
        states["gt"]["qx"].append(q[0])
        states["gt"]["qy"].append(q[1])
        states["gt"]["qz"].append(q[2])
        states["gt"]["qw"].append(q[3])

        # # generate observations
        for j in range(6):
            # Simulate the sensor readings
            p_temp = p + np.random.multivariate_normal(np.zeros(3), R[:3, :3])
            q_temp = Rot.from_quat(
                q
                + np.random.multivariate_normal(
                    np.zeros(4), np.diag([0.1, 0.1, 0.1, 0.1])
                )
            ).as_quat()

            observations[j * 7 : j * 7 + 7] = np.concatenate([p_temp, q_temp])

        ukf.predict(observations)
        states["fuse"]["x"].append(ukf.ukf.x[0])
        states["fuse"]["y"].append(ukf.ukf.x[1])
        states["fuse"]["z"].append(ukf.ukf.x[2])

        states["fuse"]["qx"].append(ukf.ukf.x[3])
        states["fuse"]["qy"].append(ukf.ukf.x[4])
        states["fuse"]["qz"].append(ukf.ukf.x[5])
        states["fuse"]["qw"].append(ukf.ukf.x[6])

    # Plot the 3D trajectory of the ground truth and the fused state
    fig = plt.figure()
    ax1 = fig.add_subplot(121, projection="3d")
    ax1.plot(
        states["gt"]["x"],
        states["gt"]["y"],
        states["gt"]["z"],
        label="Ground Truth",
    )
    ax1.plot(
        states["fuse"]["x"],
        states["fuse"]["y"],
        states["fuse"]["z"],
        label="Fused State",
        linestyle="--",
    )

    ax2 = fig.add_subplot(122, projection="3d")
    ax2.plot(
        states["gt"]["qx"],
        states["gt"]["qy"],
        states["gt"]["qz"],
        label="Ground Truth",
    )
    ax2.plot(
        states["fuse"]["qx"],
        states["fuse"]["qy"],
        states["fuse"]["qz"],
        label="Fused State",
        linestyle="--",
    )

    # print(states["fuse"]["roll"])
    plt.legend()
    plt.show()


if __name__ == "__main__":
    main()
