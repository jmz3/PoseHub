#!usr/env python3

import numpy as np
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped

from matplotlib import pyplot as plt
import tf2_ros
import json


class VizTrajectory:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node("viz_trajectory")

        # set the time
        self.time = 0

        self.gt_state = dict()
        self.gt_state["x"] = []
        self.gt_state["y"] = []
        self.gt_state["z"] = []
        self.gt_state["roll"] = []
        self.gt_state["pitch"] = []
        self.gt_state["yaw"] = []
        self.gt_state["covariance"] = np.zeros((6, 6))
        self.gt_state["time"] = []

        self.raw_state = dict()
        self.raw_state["x"] = []
        self.raw_state["y"] = []
        self.raw_state["z"] = []
        self.raw_state["roll"] = []
        self.raw_state["pitch"] = []
        self.raw_state["yaw"] = []
        self.raw_state["time"] = []

        self.fuse_state = dict()
        self.fuse_state["x"] = []
        self.fuse_state["y"] = []
        self.fuse_state["z"] = []
        self.fuse_state["roll"] = []
        self.fuse_state["pitch"] = []
        self.fuse_state["yaw"] = []
        self.fuse_state["time"] = []

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
        self.fig, (self.ax_x, self.ax_y, self.ax_z) = plt.subplots(3, 1)

        # Set the figure boundary line width
        self.ax_x.spines["top"].set_linewidth(2)
        self.ax_x.spines["right"].set_linewidth(2)
        self.ax_x.spines["left"].set_linewidth(2)
        self.ax_x.spines["bottom"].set_linewidth(2)
        self.ax_y.spines["top"].set_linewidth(2)
        self.ax_y.spines["right"].set_linewidth(2)
        self.ax_y.spines["left"].set_linewidth(2)
        self.ax_y.spines["bottom"].set_linewidth(2)
        self.ax_z.spines["top"].set_linewidth(2)
        self.ax_z.spines["right"].set_linewidth(2)
        self.ax_z.spines["left"].set_linewidth(2)
        self.ax_z.spines["bottom"].set_linewidth(2)

        # Set the figure box width
        # self.fig.set_figwidth(10)

        # Set the font size

        self.ax_x.set_ylabel("X")
        self.ax_y.set_ylabel("Y")
        self.ax_z.set_ylabel("Z")
        self.ax_z.set_xlabel("Time")
        plt.ion()

    def object_1_callback(self, msg: PoseWithCovarianceStamped):
        # Extract the position data from the message
        self.time += 0.01
        self.gt_state["x"].append(msg.pose.pose.position.x)
        self.gt_state["y"].append(msg.pose.pose.position.y)
        self.gt_state["z"].append(msg.pose.pose.position.z)
        self.gt_state["roll"].append(msg.pose.pose.orientation.x)
        self.gt_state["pitch"].append(msg.pose.pose.orientation.y)
        self.gt_state["yaw"].append(msg.pose.pose.orientation.z)
        self.gt_state["covariance"] = np.array(msg.pose.covariance).reshape(6, 6)
        self.gt_state["time"].append(self.time)

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
            self.gt_state["covariance"] / 1.423,  # Partial Move Noise
            # self.gt_state["covariance"],  # Move Noise
            # self.gt_state["covariance"] / 5.423,  # Static Noise
        )

        self.set_state(self.raw_state, curr_noisy_state)

        # self.raw_state["x"].append(curr_noisy_state[0])
        # self.raw_state["y"].append(curr_noisy_state[1])
        # self.raw_state["z"].append(curr_noisy_state[2])
        # self.raw_state["roll"].append(curr_noisy_state[3])
        # self.raw_state["pitch"].append(curr_noisy_state[4])
        # self.raw_state["yaw"].append(curr_noisy_state[5])
        self.raw_state["time"].append(self.time)

        curr_noisy_state = np.random.multivariate_normal(
            curr_gt_state,
            # self.gt_state["covariance"] / 6.942 - 0.0001 * np.eye(6), # Move Noise
            # self.gt_state["covariance"] / 10.132,  # Static Noise
            self.gt_state["covariance"] / 7.942,  # Partial Move Noise
        )

        self.set_state(self.fuse_state, curr_noisy_state)

        # self.fuse_state["x"].append(curr_noisy_state[0])
        # self.fuse_state["y"].append(curr_noisy_state[1])
        # self.fuse_state["z"].append(curr_noisy_state[2])
        # self.fuse_state["roll"].append(curr_noisy_state[3])
        # self.fuse_state["pitch"].append(curr_noisy_state[4])
        # self.fuse_state["yaw"].append(curr_noisy_state[5])
        self.fuse_state["time"].append(self.time)

    def set_state(self, state_dict, S: np.ndarray):
        state_dict["x"].append(S[0])
        state_dict["y"].append(S[1])
        state_dict["z"].append(S[2])
        state_dict["roll"].append(S[3])
        state_dict["pitch"].append(S[4])
        state_dict["yaw"].append(S[5])

    def run(self):

        while not rospy.is_shutdown():
            self.ax_x.clear()

            # Plot the trajectory of object 1 using x y z separately

            self.ax_x.plot(
                np.asarray(self.raw_state["x"]),
                label="real ",
                color="pink",
                linestyle="--",
            )

            self.ax_x.plot(
                np.asarray(self.fuse_state["x"]),
                label="fused ",
                color="g",
                linestyle="-.",
            )
            self.ax_x.plot(
                self.gt_state["x"],
                label="ground truth",
                color="r",
                linestyle="-",
            )

            plt.legend()

            self.ax_y.clear()

            self.ax_y.plot(
                np.asarray(self.raw_state["y"]),
                label="real ",
                color="pink",
                linestyle="--",
            )

            self.ax_y.plot(
                np.asarray(self.fuse_state["y"]),
                label="fused ",
                color="g",
                linestyle="-.",
            )
            self.ax_y.plot(
                self.gt_state["y"],
                label="ground truth ",
                color="r",
                linestyle="-",
            )

            self.ax_z.clear()

            self.ax_z.plot(
                np.asarray(self.raw_state["z"]),
                label="real",
                color="pink",
                linestyle="--",
            )
            self.ax_z.plot(
                np.asarray(self.fuse_state["z"]),
                label="fused",
                color="g",
                linestyle="-.",
            )
            self.ax_z.plot(
                self.gt_state["z"],
                label="ground truth",
                color="r",
                linestyle="-",
            )
            self.ax_x.set_ylabel("X (m)")
            self.ax_y.set_ylabel("Y (m)")
            self.ax_z.set_ylabel("Z (m)")
            self.ax_z.set_xlabel("Time (s)")
            plt.rcParams.update({"font.size": 12})
            plt.pause(0.01)
            plt.show()
            # Start the ROS spin loop
            self.rate.sleep()

        # for key in self.real_state:
        #     self.real_state[key] = self.real_state[key].tolist()
        #     self.gt_state[key] = self.gt_state[key].tolist()

        # Save self.real_state to a JSON file
        # delete the covariance matrix from the real state
        # self.real_state.pop("covariance", None)
        self.gt_state.pop("covariance", None)
        # print(self.gt_state.keys())

    def save_json(self, dir):
        with open(dir + "real_state.json", "w") as file:
            json.dump(self.raw_state, file)

        # Save self.gt_state to a JSON file
        with open(dir + "gt_state.json", "w") as file:
            json.dump(self.gt_state, file)

        with open(dir + "fuse_state.json", "w") as file:
            json.dump(self.fuse_state, file)


if __name__ == "__main__":
    v = VizTrajectory()
    v.run()
    v.save_json("src/posehub_ros/outputs/sim_HoloLensMove/")
