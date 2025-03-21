import numpy as np
import rosbag
import os
import threading
import rospy
from geometry_msgs.msg import PoseStamped, Pose
import plotly.graph_objects as go
from scipy.spatial.transform import Rotation


def npy2bag(
    session_dir,
    output_bag_dir,
    modes,
    num_of_trackers=3,
    num_of_markers=2,
    vis=False,
    bag=False,
    pub=False,
):
    if modes == "STTAR_FUSE":
        print("STTAR_FUSE_format")
        # tobeExtracted = ["NDI2mov.npy","HOL2mov.npy"]
        NotImplementedError
    if modes == "SCENEGRAPH":
        if pub:
            threads = []
        object_align = {"NDI": [1, 2], "Holo": [0, 2], "Real": [0, 1]}

        bag = rosbag.Bag(output_bag_dir, "w") if bag else None

        print("SCENEGRAPH_format...")
        tobeExtracted = ["SceneByHolo.npy", "SceneByNDI.npy", "SceneByRealSense.npy"]
        topic_names = ["KUKA", "REF"]
        for npy_file in tobeExtracted:
            poses, timestamps = npy2poses(
                os.path.join(session_dir, npy_file), npy_file[7:10], num_of_markers
            )
            print(npy_file + " with", poses.shape)
            for i in range(num_of_markers):
                topic_name = "/" + topic_names[i] + npy_file.split(".")[0][5:]
                print(topic_name)
                if vis:
                    x = poses[:, i, 0]
                    y = poses[:, i, 1]
                    z = poses[:, i, 2]
                    fig = go.Figure(data=go.Scatter3d(x=x, y=y, z=z, mode="markers"))
                    fig.update_layout(
                        scene=dict(xaxis_title="X", yaxis_title="Y", zaxis_title="Z")
                    )
                    fig.show()
                if bag:

                    for pose, timestamp in zip(poses, timestamps):
                        # print(pose[i]) if None in pose[i] else None
                        # print(timestamp[0])
                        pose_msg = PoseStamped()
                        pose_msg.header.stamp = rospy.Time.from_sec(timestamp[0])

                        pose_msg.pose.position.x = pose[i, 0]
                        pose_msg.pose.position.y = pose[i, 1]
                        pose_msg.pose.position.z = pose[i, 2]
                        pose_msg.pose.orientation.x = pose[i, 3]
                        pose_msg.pose.orientation.y = pose[i, 4]
                        pose_msg.pose.orientation.z = pose[i, 5]
                        pose_msg.pose.orientation.w = pose[i, 6]
                        bag.write(
                            topic=topic_name,
                            msg=pose_msg,
                            t=rospy.Time.from_sec(timestamp[0]),
                        )
                if pub:
                    poses_single_marker = poses[i, :]
                    print("Pose shape", poses_single_marker.shape)
                    thread = threading.Thread(
                        target=publish_pose(
                            None, topic_name, poses_single_marker, timestamps
                        )
                    )
                    thread.start()
                    threads.append(thread)
                    # NotImplementedError
        if pub:
            for t in threads:
                t.join()
        if bag:
            bag.close()

    print("Done!")


def npy2poses(npy_dir, npy_type, num_of_markers):
    print(npy_type)
    if npy_type == "NDI":
        poses = np.load(npy_dir)
        print(npy_dir + " with", poses.shape)
        assert poses.shape[1] == 3 * 16 + 1  # FIXED NOW
        # poses = poses[:,:num_of_markers*16+1]
        poseswithouttime = poses[:, 17:].reshape(
            -1, num_of_markers, 16
        )  # FIXME:17 is due to MINGXU's sequence
        # Convert to 7*num_of_markers
        poseswithouttime = np.apply_along_axis(homo2quat, axis=2, arr=poseswithouttime)
        timestamps = poses[:, 0].reshape(-1, 1)

        # align time stamps to force it start from 1
        timestamps = timestamps - timestamps[0] + 1
        # poses = np.concatenate((poses[:,0].reshape(-1,1),poseswithouttime),axis=2)
    elif npy_type == "Hol":
        poses = np.load(npy_dir)
        print(npy_dir + " with", poses.shape)
        assert poses.shape[1] == 3 * 7 + 1  # FIXED NOW
        # poses = poses[:,:num_of_markers*7+1]
        poseswithouttime = np.vstack([poses[:, 1:8], poses[:, 15:22]]).reshape(
            -1, num_of_markers, 7
        )
        timestamps = poses[:, 0].reshape(-1, 1)

        # align time stamps to force it start from 0
        timestamps = timestamps - timestamps[0] + 1

    else:  # Realsense
        poses = np.load(npy_dir)
        print(npy_dir + " with", poses.shape)
        assert poses.shape[1] == 3 * 8 + 1  # FIXED NOW
        poses = poses[:, : num_of_markers * 8 + 1]
        print(poses.shape)
        poseswithouttime = poses[:, 1:].reshape(-1, num_of_markers, 8)
        timestamps = poses[:, 0].reshape(-1, 1)
        # align time stamps to force it start from 0
        timestamps = timestamps - timestamps[0] + 1

    return poseswithouttime, timestamps


def homo2quat(matrix):
    matrix = matrix.reshape(4, 4)
    translation = matrix[:3, 3] / 1000
    rotation = Rotation.from_matrix(matrix[:3, :3])
    quaternion = rotation.as_quat()
    result = np.concatenate((translation, quaternion))
    return result


def publish_pose(thread_name, topic_name, poses, timestamps):

    thread_name = "temp_node" if thread_name is None else thread_name

    rospy.init_node(thread_name, anonymous=True)
    pub = rospy.Publisher(topic_name, PoseStamped, queue_size=10)
    rate = rospy.Rate(60)
    i = 0
    while not rospy.is_shutdown():
        pose_msg = PoseStamped()
        pose_msg.header.stamp = timestamps[i]
        # Set the pose values here
        pose_msg.pose.position.x = poses[i, 0]
        pose_msg.pose.position.y = poses[i, 1]
        pose_msg.pose.position.z = poses[i, 2]
        pose_msg.pose.orientation.x = poses[i, 3]
        pose_msg.pose.orientation.y = poses[i, 4]
        pose_msg.pose.orientation.z = poses[i, 5]
        pose_msg.pose.orientation.w = poses[i, 6]
        i += 1
        rospy.loginfo(f"{thread_name}: Publishing pose")
        pub.publish(pose_msg)
        rate.sleep()


if __name__ == "__main__":

    # npy2bag(
    #     "ExpData/SceneGraph-Mar-10-2024/0cmAllStatic",
    #     "src/posehub_ros/outputs/bag/0cmAllStatic.bag",
    #     "SCENEGRAPH",
    #     3,
    #     2,
    #     vis=0,
    #     bag=1,
    #     pub=0,
    # )
    npy2bag(
        "ExpData/SceneGraph-Mar-10-2024/5cmHoloLensMoveMarkerMove",
        "src/posehub_ros/outputs/bag/5cmHoloLensMoveMarkerMove.bag",
        "SCENEGRAPH",
        3,
        2,
        vis=0,
        bag=1,
        pub=0,
    )
