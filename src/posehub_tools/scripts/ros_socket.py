#!/usr/bin/env python3
"""
This node listens to the existing tf2 tree (with all frames defined relative to "world")
and queries the relative transforms from a specified sensor frame to one or more object frames.
It then publishes the results via a ZeroMQ PUB socket.

ROS parameters (with default values) are used to configure the node:
  ~sensor_frame   (default: "camera")
      - The reference frame (sensor) from which relative transforms are computed.
  ~object_frames  (default: ["object_1", "object_2"])
      - A list of object frames for which to compute the transforms.
  ~zmq_port       (default: 5555)
      - The port number on which the ZeroMQ publisher will bind.

Usage Example:
  rosrun posehub_tools ros_socket.py _sensor_frame:=Camera _object_frames:="[object_1, object_2, ref_1]" _zmq_port:=5555
  rosrun posehub_tools ros_socket.py _sensor_frame:=HoloLens _object_frames:="[object_1, object_2, ref_1]" _zmq_port:=5557
"""

import rospy
import tf2_ros
import zmq
import time
import numpy as np
from scipy.spatial.transform import Rotation as Rot
from geometry_msgs.msg import TransformStamped


def query_and_send(tfBuffer, publisher, sensor_frame, object_frames):
    """
    Queries the tfBuffer for the transform from sensor_frame to each frame in object_frames,
    formats the results, and sends them via the provided ZeroMQ socket.
    """
    try:
        result = "Relative transforms from '{}':\n".format(sensor_frame)
        for obj in object_frames:
            # Query the transform from sensor_frame to the current object frame
            trans = tfBuffer.lookup_transform(
                sensor_frame, obj, rospy.Time(0), rospy.Duration(1.0)
            )
            # Build the 4x4 transformation matrix.
            pose_mtx = np.eye(4, dtype=np.float32)
            pose_mtx[0, 3] = trans.transform.translation.x
            pose_mtx[1, 3] = trans.transform.translation.y
            pose_mtx[2, 3] = trans.transform.translation.z
            quat = [
                trans.transform.rotation.x,
                trans.transform.rotation.y,
                trans.transform.rotation.z,
                trans.transform.rotation.w,
            ]
            pose_mtx[:3, :3] = Rot.from_quat(quat).as_matrix()

            # Convert the matrix into a comma-separated string: "x,y,z,qx,qy,qz,qw,1"
            trans = pose_mtx[:3, 3]
            quat = Rot.from_matrix(pose_mtx[:3, :3]).as_quat()
            msg_values = [
                trans[0],
                trans[1],
                trans[2],
                quat[0],
                quat[1],
                quat[2],
                quat[3],
                1,
            ]
            msg_str = ",".join(str(val) for val in msg_values)
            # Publish the message on the given topic.
            publisher.send_multipart([obj.encode("utf-8"), msg_str.encode("utf-8")])
    except Exception as e:
        rospy.logwarn("TF lookup failed: %s", e)


def main():
    rospy.init_node("tf_to_zmq_modular", anonymous=True)

    # Retrieve parameters from the ROS parameter server
    sensor_frame = rospy.get_param("~sensor_frame", "Camera")
    object_frames = rospy.get_param("~object_frames", ["object_1", "object_2", "ref_1"])
    zmq_port = rospy.get_param("~zmq_port", 5555)

    rospy.loginfo(
        "Starting node with sensor_frame: %s, object_frames: %s, zmq_port: %d",
        sensor_frame,
        object_frames,
        zmq_port,
    )

    # Set up a tf2 buffer and listener to tap into the existing tf2 tree
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    # Set up a ZeroMQ PUB socket
    context = zmq.Context()
    publisher = context.socket(zmq.PUB)
    publisher.bind(f"tcp://*:{zmq_port}")
    # rospy.loginfo("ZeroMQ publisher bound to %s", bind_address)
    time.sleep(1)

    # Create a ROS timer that queries and sends transforms at 50 Hz
    rospy.Timer(
        rospy.Duration(0.02),
        lambda event: query_and_send(tfBuffer, publisher, sensor_frame, object_frames),
    )

    rospy.spin()


if __name__ == "__main__":
    main()
