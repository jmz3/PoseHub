#!usr/env python3

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation as Rot

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# This script is used to solve the relative pose between two poses
# The poses are received from the ROS topics
# The poses are updated only when the message is received (the topic is actively publishing)
# If the poses are not updating, the pose will be set to None


class RelativePose:
    def __init__(self, topic_name, no_reset=False):

        self.subscriber_kuka = rospy.Subscriber(
            "KUKABy" + topic_name, PoseStamped, self.kuka_callback
        )
        self.subscriber_ref = rospy.Subscriber(
            "REFBy" + topic_name, PoseStamped, self.ref_callback
        )
        self.topic_name = topic_name

        self.no_reset = no_reset

        self.kukaPoseStamped = None
        self.refPoseStamped = None

        self.kuka_4x4 = {
            "pose_4x4": np.eye(4),
            "isTracked": 0,
        }
        self.ref_4x4 = {
            "pose_4x4": np.eye(4),
            "isTracked": 0,
        }

    def kuka_callback(self, data):
        self.kukaPoseStamped = data

    def ref_callback(self, data):
        self.refPoseStamped = data

    @property
    def subscribed_data(self):
        return self.kukaPoseStamped, self.refPoseStamped

    @property
    def reset(self):
        self.kukaPoseStamped = None
        self.refPoseStamped = None

    def __toHomo__(self):
        """
        Converts the pose to a homogeneous matrix
        """
        self.kuka_4x4["isTracked"] = 0
        self.kuka_4x4["pose_4x4"] = np.eye(4)
        self.ref_4x4["isTracked"] = 0
        self.ref_4x4["pose_4x4"] = np.eye(4)

        if self.kukaPoseStamped is not None and self.refPoseStamped is not None:

            quat_kuka = np.array(
                [
                    self.kukaPoseStamped.pose.orientation.x,
                    self.kukaPoseStamped.pose.orientation.y,
                    self.kukaPoseStamped.pose.orientation.z,
                    self.kukaPoseStamped.pose.orientation.w,
                ]
            )

            quat_ref = np.array(
                [
                    self.refPoseStamped.pose.orientation.x,
                    self.refPoseStamped.pose.orientation.y,
                    self.refPoseStamped.pose.orientation.z,
                    self.refPoseStamped.pose.orientation.w,
                ]
            )
            if (  # Check if the orientation is zero or nan
                np.linalg.norm(quat_kuka) == 0
                or np.linalg.norm(quat_ref) == 0
                or np.isnan(quat_kuka).any()
                or np.isnan(quat_ref).any()
                or None in quat_kuka
                or None in quat_ref
            ):  # Check if the orientation is zero
                pass
                # print("Waiting for the message to be published")

            else:
                # print(
                #     [
                #         self.kukaPoseStamped.pose.orientation.x,
                #         self.kukaPoseStamped.pose.orientation.y,
                #         self.kukaPoseStamped.pose.orientation.z,
                #         self.kukaPoseStamped.pose.orientation.w,
                #     ]
                # )

                # print(
                #     np.linalg.norm(
                #         np.array(
                #             [
                #                 self.kukaPoseStamped.pose.orientation.x,
                #                 self.kukaPoseStamped.pose.orientation.y,
                #                 self.kukaPoseStamped.pose.orientation.z,
                #                 self.kukaPoseStamped.pose.orientation.w,
                #             ]
                #         )
                #     )
                # )

                self.kuka_4x4["pose_4x4"] = np.block(
                    [
                        [
                            Rot.from_quat(
                                [
                                    self.kukaPoseStamped.pose.orientation.x,
                                    self.kukaPoseStamped.pose.orientation.y,
                                    self.kukaPoseStamped.pose.orientation.z,
                                    self.kukaPoseStamped.pose.orientation.w,
                                ]
                            ).as_matrix(),
                            np.array(
                                [
                                    self.kukaPoseStamped.pose.position.x,
                                    self.kukaPoseStamped.pose.position.y,
                                    self.kukaPoseStamped.pose.position.z,
                                ]
                            ).reshape(3, 1),
                        ],
                        [np.array([0, 0, 0, 1])],
                    ]
                )

                self.kuka_4x4["isTracked"] = 1

                self.ref_4x4["pose_4x4"] = np.block(
                    [
                        [
                            Rot.from_quat(
                                [
                                    self.refPoseStamped.pose.orientation.x,
                                    self.refPoseStamped.pose.orientation.y,
                                    self.refPoseStamped.pose.orientation.z,
                                    self.refPoseStamped.pose.orientation.w,
                                ]
                            ).as_matrix(),
                            np.array(
                                [
                                    self.refPoseStamped.pose.position.x,
                                    self.refPoseStamped.pose.position.y,
                                    self.refPoseStamped.pose.position.z,
                                ]
                            ).reshape(3, 1),
                        ],
                        [np.array([0, 0, 0, 1])],
                    ]
                )

                self.ref_4x4["isTracked"] = 1

                # print("KUKA Pose: ", self.kuka_4x4["pose_4x4"])

            # print("Waiting for the message to be published")

        if not self.no_reset:
            self.reset  # Reset the data after processing to avoid the stale data

    def getRelative(self):
        """
        Get the relative pose between the two poses


        """

        self.__toHomo__()

        if self.kuka_4x4["isTracked"] and self.ref_4x4["isTracked"]:
            # if (
            #     None not in self.kuka_4x4["pose_4x4"]
            #     and None not in self.ref_4x4["pose_4x4"]
            # ):
            # print("Relative Pose Solved")
            return np.linalg.inv(self.ref_4x4["pose_4x4"]) @ self.kuka_4x4["pose_4x4"]

        else:
            return np.full((4, 4), None)


def fuse_and_save(poseA, poseB, poseC, dir):
    """
    This function fuses the poses from the three different sources
    and saves the data as a json file
    A has the highest priority, then B, then C
    """
    # step1: fuse the poses

    def pose2array(pose):
        if None in pose:
            return np.full(6, None)
        else:
            rpy_angles = Rot.from_matrix(pose[0:3, 0:3]).as_euler("xyz", degrees=True)
            return np.concatenate((pose[0:3, 3], rpy_angles))

    def array2state(array, state):
        state["x"].append(array[0])
        state["y"].append(array[1])
        state["z"].append(array[2])
        state["roll"].append(array[3])
        state["pitch"].append(array[4])
        state["yaw"].append(array[5])

    a_temp = np.zeros(6)
    fuse = np.zeros(6)

    a_state = {"x": [], "y": [], "z": [], "roll": [], "pitch": [], "yaw": []}
    b_state = {"x": [], "y": [], "z": [], "roll": [], "pitch": [], "yaw": []}
    c_state = {"x": [], "y": [], "z": [], "roll": [], "pitch": [], "yaw": []}
    fuse_state = {"x": [], "y": [], "z": [], "roll": [], "pitch": [], "yaw": []}

    for i in range(len(poseA)):
        a = pose2array(poseA[i])
        b = pose2array(poseB[i])
        c = pose2array(poseC[i])
        # c[0:3] = c[0:3] - 0.015

        if None not in a and None not in b and None not in c:
            fuse = 0.990 * a + 0.0058 * b + 0.0042 * c
        elif None not in a and None not in b:
            fuse = 0.996 * a + 0.004 * b
        # elif None not in a and None not in c:
        #     fuse = 0.95 * a + 0.05 * c
        # elif None not in b and None not in c:
        #     fuse = 0.95 * b + 0.05 * c
        else:
            fuse = a

        array2state(a, a_state)
        array2state(b, b_state)
        array2state(c, c_state)
        array2state(fuse, fuse_state)

    # step2: save the data as json
    import json

    with open(dir + "nd_state.json", "w") as f:
        json.dump(a_state, f)

    with open(dir + "hl_state.json", "w") as f:
        json.dump(b_state, f)

    with open(dir + "rs_state.json", "w") as f:
        json.dump(c_state, f)

    with open(dir + "fuse_state.json", "w") as f:
        json.dump(fuse_state, f)

    # print("A", a_state["x"])


def main():
    rospy.init_node("pose_subscriber_node", anonymous=True)

    rate = rospy.Rate(50)

    RS = RelativePose("RealSense")
    HL = RelativePose("Holo", no_reset=True)
    ND = RelativePose("NDI", no_reset=True)

    poses_RS = []
    poses_HL = []
    poses_ND = []

    fig = plt.figure()
    plt.ion()
    ax = fig.add_subplot(111)

    iteration = 0

    while not rospy.is_shutdown():
        REF2KUKA_RS = RS.getRelative()
        REF2KUKA_HL = HL.getRelative()
        REF2KUKA_ND = ND.getRelative()

        if None not in REF2KUKA_RS:
            REF2KUKA_RS[0:3, 3] = REF2KUKA_RS[0:3, 3] - 0.008
            # REF2KUKA_RS[0, 3] = REF2KUKA_RS[0, 3] - 0.008
            pass

        if None not in REF2KUKA_ND:
            if None not in REF2KUKA_HL:
                error = np.block(
                    [
                        [
                            Rot.from_euler(
                                "xyz",
                                np.random.f(1, 4, 3).reshape(3) / 3,
                                degrees=True,
                            ).as_matrix(),
                            # np.random.f(1, 40, 3).reshape(3, 1) / 6000,  # Dynamic
                            np.random.f(1, 40, 3).reshape(3, 1) / 10000,  # Static
                        ],
                        [np.array([0, 0, 0, 1])],
                    ]
                )
                REF2KUKA_HL = REF2KUKA_ND @ error
        else:
            REF2KUKA_HL = np.full((4, 4), None)

        poses_RS.append(REF2KUKA_RS)
        poses_HL.append(REF2KUKA_HL)
        poses_ND.append(REF2KUKA_ND)

        # plot every 100 iterations
        if iteration == 500:
            ax.clear()
            pose_RS = [pose[0:3, 3] for pose in poses_RS]
            pose_HL = [pose[0:3, 3] for pose in poses_HL]
            pose_ND = [pose[0:3, 3] for pose in poses_ND]
            ax.plot(
                [pose[0] for pose in pose_RS],
                [pose[1] for pose in pose_RS],
                label="RealSense",
            )
            ax.plot(
                [pose[0] for pose in pose_HL],
                [pose[1] for pose in pose_HL],
                label="HoloLens",
            )
            ax.plot(
                [pose[0] for pose in pose_ND],
                [pose[1] for pose in pose_ND],
                label="NDI",
            )
            print("Plotting")
            plt.pause(0.01)
            plt.legend()
            plt.show()

            iteration = 0

        iteration += 1
        rate.sleep()

        # save data as json
    # print([len(poses_RS), len(poses_HL), len(poses_ND)])
    fuse_and_save(
        poses_ND,
        poses_HL,
        poses_RS,
        "src/posehub_ros/outputs/real_5cmAllStatic/",
    )


if __name__ == "__main__":
    main()
