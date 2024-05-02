#!usr/env python3

import numpy as np
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped

from matplotlib import pyplot as plt
import tf2_ros


class VizTrajectory:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node("viz_trajectory")

        self.gt_state = dict()
        self.gt_state["x"] = []
        self.gt_state["y"] = []
        self.gt_state["z"] = []
        self.gt_state["roll"] = []
        self.gt_state["pitch"] = []
        self.gt_state["yaw"] = []
        self.gt_state["covariance"] = np.zeros((6, 6))

        self.real_state = dict()
        self.real_state["x"] = []
        self.real_state["y"] = []
        self.real_state["z"] = []
        self.real_state["roll"] = []
        self.real_state["pitch"] = []
        self.real_state["yaw"] = []

        # Subscribe to the "/HoloLens/object_1_noisy" topic
        rospy.Subscriber(
            "/HoloLens/object_1_noisy",
            PoseWithCovarianceStamped,
            self.object_1_callback,
        )

        # Subscribe to the "/HoloLens/object_2_noisy" topic
        rospy.Subscriber(
            "/HoloLens/object_2_noisy",
            PoseWithCovarianceStamped,
            self.object_2_callback,
        )

        self.rate = rospy.Rate(50)

        # Set up the plot
        self.fig, self.ax = plt.subplots()
        self.ax.set_xlabel("Time")
        self.ax.set_ylabel("Position")
        plt.ion()

    def object_1_callback(self, msg: PoseWithCovarianceStamped):
        # Extract the position data from the message
        self.gt_state["x"].append(msg.pose.pose.position.x)
        self.gt_state["y"].append(msg.pose.pose.position.y)
        self.gt_state["z"].append(msg.pose.pose.position.z)
        self.gt_state["roll"].append(msg.pose.pose.orientation.x)
        self.gt_state["pitch"].append(msg.pose.pose.orientation.y)
        self.gt_state["yaw"].append(msg.pose.pose.orientation.z)
        self.gt_state["covariance"] = np.array(msg.pose.covariance).reshape(6, 6)

        self.apply_noise()
        # Plot the position of object 1
        # print("Object 1: x={}, y={}".format(x, y))

    def object_2_callback(self, msg: PoseWithCovarianceStamped):
        # Extract the position data from the message
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Plot the position of object 2
        # print("Object 2: x={}, y={}".format(x, y))

    def apply_noise(self):
        # apply the noise defined in the covariance matrix to the state

        curr_gt_state = np.array(
            [
                self.gt_state["x"][-1],
                self.gt_state["y"][-1],
                self.gt_state["z"][-1],
                self.gt_state["roll"][-1],
                self.gt_state["pitch"][-1],
                self.gt_state["yaw"][-1],
            ]
        )
        curr_noisy_state = np.random.multivariate_normal(
            curr_gt_state,
            self.gt_state["covariance"],
        )

        self.real_state["x"].append(curr_noisy_state[0] - 0.4)
        self.real_state["y"].append(curr_noisy_state[1])
        self.real_state["z"].append(curr_noisy_state[2])
        self.real_state["roll"].append(curr_noisy_state[3])
        self.real_state["pitch"].append(curr_noisy_state[4])
        self.real_state["yaw"].append(curr_noisy_state[5])

    def run(self):
        # Show the plot
        while not rospy.is_shutdown():
            self.ax.clear()

            # Plot the trajectory of object 1 using x y z separately
            self.ax.plot(
                self.gt_state["x"],
                label="ground truth x",
                color="r",
                linestyle="-",
            )
            self.ax.plot(
                self.real_state["x"], label="real x", color="r", linestyle="--"
            )
            # self.ax.plot(self.state["y"], label="y", color="g")
            # self.ax.plot(self.state["z"], label="z", color="b")

            plt.pause(0.01)
            plt.show()
            # Start the ROS spin loop
            self.rate.sleep()


if __name__ == "__main__":
    v = VizTrajectory()
    v.run()
