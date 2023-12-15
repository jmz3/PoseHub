# from pose_graph import PoseGraph
from typing import Type, Dict, List, Optional
from comm.ZMQManager import ZMQManager
from pose_graph import PoseGraph
import argparse
import threading
import time
import numpy as np
from scipy.spatial.transform import Rotation as Rot


def main(args):
    """
    Main function
    """
    global memoryBuffer
    memoryBuffer = []
    # initialize the pose graph
    pose_graph = PoseGraph()

    tool_1_id = "tool_1"
    tool_2_id = "tool_2"
    tool_3_id = "tool_3"

    args_1 = argparse.Namespace(
        sub_ip="192.168.1.23",
        sub_port="5588",
        pub_port="5589",
        sub_topic=[tool_1_id, tool_2_id, tool_3_id],
        pub_topic=[tool_1_id, tool_2_id, tool_3_id],
        sensor_name="h1",
    )  # args for h1 sensor

    args_2 = argparse.Namespace(
        sub_ip="192.168.1.23",
        sub_port="5589",
        pub_port="5580",
        sub_topic=[tool_1_id, tool_2_id, tool_3_id],
        pub_topic=[b"topic4", b"topic5", b"topic6"],
        sensor_name="h2",
    )  # args for h2 sensor

    zmq_manager_1, sub_thread_1, pub_thread_1 = ZMQManager.initialize(
        args_1.sub_ip,
        args_1.sub_port,
        args_1.pub_port,
        args_1.sub_topic,
        args_1.pub_topic,
        args_1.sensor_name,
    )
    zmq_manager_2, sub_thread_2, pub_thread_2 = ZMQManager.initialize(
        args_2.sub_ip,
        args_2.sub_port,
        args_2.pub_port,
        args_2.sub_topic,
        args_2.pub_topic,
        args_2.sensor_name,
    )

    # zmq_manager_2, sub_thread_2, pub_thread_2 = initialize_ZMQManager(args.sub_ip, args.sub_port, args.pub_port, args.sub_topic, args.pub_topic)
    i = 0
    try:
        while True:
            # Running the main loop

            # receive messages
            poseinfo_sensor1 = zmq_manager_1.receive_poses(args_1, zmq_manager_1)
            if len(poseinfo_sensor1) != 0:
                print("poseinfo: ", poseinfo_sensor1)

            else:
                print("poseinfo_sensor1 is empty")

            poseinfo_sensor2 = zmq_manager_2.receive_poses(args_2, zmq_manager_2)
            if len(poseinfo_sensor2) != 0:
                print("poseinfo: ", poseinfo_sensor2)

            else:
                print("poseinfo_sensor2 is empty")

            # send messages
            for topic in args_1.pub_topic:
                # transfer the topic from bytes to string
                pose = pose_graph.get_transform(
                    "h1", topic.decode("utf-8"), solver_method="BFS"
                )
                zmq_manager_1.send_poses(topic, zmq_manager_1, pose)

            i += 1e-6

    except KeyboardInterrupt:
        # terminate_ZMQManager(zmq_manager_1, sub_thread_1, pub_thread_1)
        zmq_manager_1.terminate()
        zmq_manager_2.terminate()
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
        default=["tool_1"],
        type=str,
        help="subscriber topics",
    )
    parser.add_argument(
        "--pub_topic",
        default=["topic4", "topic5", "topic6"],
        type=str,
        help="publisher topic",
    )
    parser.add_argument("--sensor_name", default="h1", type=str, help="sensor name")

    args = parser.parse_args()

    main()
