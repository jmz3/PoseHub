from pose_graph import PoseGraph
from comm import *
from typing import Type, Dict, List, Optional
from comm.ZMQManager import ZMQManager
import argparse
import threading
import time
import numpy as np
from scipy.spatial.transform import Rotation as Rot


def receive_poses(args: argparse.Namespace, zmq_manager: ZMQManager):
    """
    Receive poses from the sensors

    Args:
    ----------
        args: argparse.Namespace, arguments
        zmq_manager: ZMQManager, the ZMQManager object

    Return:
    ----------
        tool_info: dict, {topic name: [transformation matrix: np.ndarray(np.float32, (4, 4)), isActive: bool]}
    """
    received_dict = zmq_manager.sub_poses
    pose_mtx = np.identity(4)
    tool_info = {}
    for topic in args.sub_topic:
        topic = topic.decode("utf-8")
        if topic in received_dict:  # check if the topic is in the received_dict
            if len(received_dict[topic].split(",")) < 7:
                # means no pose received
                print("waiting for poses")
                return {}
            else:
                twist = np.array([float(x) for x in received_dict[topic].split(",")])
                pose_mtx[:3, :3] = Rot.from_quat(twist[3:7]).as_matrix()
                pose_mtx[:3, 3] = np.array([twist[0], twist[1], -twist[2]])
                tool_info[topic] = [
                    pose_mtx,
                    twist[7] == 1,
                ]  # twist[7] == 1 means isActive?

        else:
            print("Subscribing to the topic: ", topic, " but no message received")
            return {}
    return tool_info


def send_poses(topic, zmq_manager, transform_mtx: np.ndarray):
    """
    Send poses to the sensors
    """

    # decode the matrix into twist
    # check the size of the matrix
    if transform_mtx.shape != (4, 4):
        print("The size of the transformation matrix is not 4x4")
        return

    pub_message_on_topic = ""
    position = transform_mtx[:3, 3]
    quaternion = Rot.from_matrix(transform_mtx[:3, :3]).as_quat()

    # encode the transformation matrix into a string
    # the string is in the order of [x, y, z, w, x, y, z, isActive], separated by commas
    pub_message_on_topic += f"{position[0]},{position[1]},{-position[2]},"
    pub_message_on_topic += f"{quaternion[0]},{quaternion[1]},{quaternion[2]},{quaternion[3]},"  # the quaternion is in the order of x,y,z,w
    pub_message_on_topic += f"1"  # isActive since we are sending the calculated poses

    zmq_manager.pub_messages[topic] = pub_message_on_topic


def initialize_ZMQManager(
    sub_ip, sub_port, pub_port, sub_topic, pub_topic, sensor_name
):
    """
    initialize a ZMQManager
    """
    zmq_manager = ZMQManager(
        sub_ip, sub_port, pub_port, sub_topic, pub_topic, sensor_name
    )
    zmq_manager.connected = True
    pub_thread = threading.Thread(target=zmq_manager.publisher_thread)
    sub_thread = threading.Thread(target=zmq_manager.subscriber_thread)
    sub_thread.start()
    pub_thread.start()
    return zmq_manager, sub_thread, pub_thread


def terminate_ZMQManager(zmq_manager, sub_thread, pub_thread):
    """
    terminate a ZMQManager
    """
    zmq_manager.connected = False
    time.sleep(0.1)
    print("Main thread interrupted, cleaning up...")
    sub_thread.join()
    pub_thread.join()


def main(args):
    """
    Main function
    """
    # initialize the pose graph
    # pose_graph = PoseGraph()

    # initialize communication managers
    zmq_manager_1, sub_thread_1, pub_thread_1 = initialize_ZMQManager(
        args.sub_ip,
        args.sub_port,
        args.pub_port,
        args.sub_topic,
        args.pub_topic,
        args.sensor_name,
    )
    # zmq_manager_2, sub_thread_2, pub_thread_2 = initialize_ZMQManager(args.sub_ip, args.sub_port, args.pub_port, args.sub_topic, args.pub_topic)
    i = 0
    try:
        while True:
            # Running the main loop

            # test receiving poses
            # print('tool 1: ', zmq_manager_1.sub_poses['tool_1'])
            poseinfo = receive_poses(args, zmq_manager_1)
            print(poseinfo)
            # print('tool 2: ', zmq_manager_1.sub_poses['tool_2'])
            # print('tool 3: ', zmq_manager_1.sub_poses['tool_3'])

            # test sending messages
            zmq_manager_1.pub_messages["topic4"] = f"topic4 test message {i}"
            zmq_manager_1.pub_messages["topic5"] = f"topic5 test message {i}"
            zmq_manager_1.pub_messages["topic6"] = f"topic6 test message {i}"
            i += 1e-6

    except KeyboardInterrupt:
        terminate_ZMQManager(zmq_manager_1, sub_thread_1, pub_thread_1)
        # terminate_ZMQManager(zmq_manager_2, sub_thread_2, pub_thread_2)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Python script for running the AR tool tracking",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--sub_ip", default="192.168.1.23", type=str, help="subscriber ip address"
    )  # 10.203.183.32
    parser.add_argument(
        "--sub_port", default="5588", type=str, help="port number for subscriber"
    )
    parser.add_argument(
        "--pub_port", default="5589", type=str, help="port number for publisher"
    )
    parser.add_argument(
        "--sub_topic",
        default=[b"tool_1", b"tool_2", b"tool_3"],
        type=str,
        help="subscriber topics",
    )
    parser.add_argument(
        "--pub_topic",
        default=[b"topic4", b"topic5", b"topic6"],
        type=str,
        help="publisher topic",
    )
    parser.add_argument("--sensor_name", default="h1", type=str, help="sensor name")

    args = parser.parse_args()

    main(args)
