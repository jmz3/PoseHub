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


if __name__ == "__main__":
    v = VizTrajectory()
    v.run()
