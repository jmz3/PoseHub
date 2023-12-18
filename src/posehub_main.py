# from pose_graph import PoseGraph
from typing import Type, Dict, List, Optional
from comm.ZMQManager import ZMQManager
from pose_graph import PoseGraph
import argparse
import threading
import time
import numpy as np
from scipy.spatial.transform import Rotation as Rot
import matplotlib.pyplot as plt


def main(args):
    """
    Main function
    """
    global memoryBuffer
    memoryBuffer = []
    # initialize the pose graph
    pose_graph = PoseGraph()

    figure = plt.figure()
    plt.ion()
    plt.show()

    ax = figure.add_subplot(projection="3d")

    tool_1_id = args.sub_topic[0]
    tool_2_id = args.sub_topic[1]
    tool_3_id = args.sub_topic[2]

    args_1 = argparse.Namespace(
        sub_ip=args.sub_ip_1,
        sub_port="5588",
        pub_port="5589",
        sub_topic=[tool_1_id, tool_2_id, tool_3_id],
        pub_topic=[tool_1_id, tool_2_id, tool_3_id],
        sensor_name="h1",
    )  # args for h1 sensor

    args_2 = argparse.Namespace(
        sub_ip=args.sub_ip_1,
        sub_port="5588",
        pub_port="5580",
        sub_topic=[tool_1_id, tool_2_id, tool_3_id],
        pub_topic=[tool_1_id, tool_2_id, tool_3_id],
        sensor_name="h2",
    )  # args for h2 sensor

    zmq_manager_1 = ZMQManager(
        sub_ip=args_1.sub_ip,
        sub_port=args_1.sub_port,
        pub_port=args_1.pub_port,
        sub_topic=args_1.sub_topic,
        pub_topic=args_1.pub_topic,
        sensor_name=args_1.sensor_name,
    )

    zmq_manager_2 = ZMQManager(
        sub_ip=args_2.sub_ip,
        sub_port=args_2.sub_port,
        pub_port=args_2.pub_port,
        sub_topic=args_2.sub_topic,
        pub_topic=args_2.pub_topic,
        sensor_name=args_2.sensor_name,
    )

    zmq_manager_1.initialize()
    zmq_manager_2.initialize()

    pose_graph.add_sensor("h1")
    pose_graph.add_sensor("h2")

    try:
        while True:
            # Running the main loop
            start_time = time.time()
            # receive messages
            poseinfo_sensor1 = zmq_manager_1.receive_poses()
            poseinfo_sensor2 = zmq_manager_2.receive_poses()
            if len(poseinfo_sensor1) != 0:
                pose_graph.update_graph("h1", poseinfo_sensor1)
            else:
                print("poseinfo_sensor1 is empty")

            # if len(poseinfo_sensor2) != 0:
            #     print("poseinfo: ", poseinfo_sensor2)
            # else:
            #     print("poseinfo_sensor2 is empty")

            # send messages
            # for topic in args_1.pub_topic:
            #     # transfer the topic from bytes to string
            #     pose = pose_graph.get_transform("h1", topic, solver_method="BFS")
            #     if pose:
            #         zmq_manager_1.send_poses(topic, pose)

            # # for topic in args_2.pub_topic:
            # #     # transfer the topic from bytes to string
            # #     pose = pose_graph.get_transform("h2", topic, solver_method="BFS")
            # #     if pose is not None:
            # #         zmq_manager_2.send_poses(topic, pose)

            # visualize the poses
            start_time = time.time()
            pose_graph.viz_graph(
                ax=ax, world_frame_id="h1", axis_limit=1.0, frame_type="sensor"
            )

            plt.pause(0.001)
            plt.draw()
            print("Time for visualization: ", time.time() - start_time)

            # # test update poses
            # try:
            #     tool1_pose = poseinfo_sensor1["tool_1"][0]
            #     zmq_manager_1.send_poses("tool_2", tool1_pose)
            # except:
            #     pass

    except KeyboardInterrupt:
        zmq_manager_1.terminate()
        zmq_manager_2.terminate()
        plt.ioff()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Python script for running the AR tool tracking",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--sub_ip_1",
        default="10.203.59.134",
        type=str,
        help="subscriber ip address sensor 1",
    )  # 10.203.183.32
    parser.add_argument(
        "--sub_ip_2",
        default="10.203.72.192",
        type=str,
        help="subscriber ip address sensor 2",
    )
    parser.add_argument(
        "--sub_topic",
        default=["tool_1", "tool_2", "tool_3"],
        type=str,
        help="subscriber topics",
    )
    parser.add_argument(
        "--pub_topic",
        default=["tool_1", "tool_2", "tool_3"],
        type=str,
        help="publisher topic",
    )
    args = parser.parse_args()

    # args_1 = argparse.Namespace(
    # sub_ip=args.sub_ip,
    # sub_port="5588",
    # pub_port="5589",
    # sub_topic=["tool_1", "tool_2", "tool_3"],
    # pub_topic=["tool_1", "tool_2", "tool_3"],
    # sensor_name="h1"
    # ) # args for h1 sensor

    # args_2 = argparse.Namespace(
    #     sub_ip="10.203.150.51",
    #     sub_port="5589",
    #     pub_port="5580",
    #     sub_topic=["tool_1", "tool_2", "tool_3"],
    #     pub_topic=["tool_1", "tool_2", "tool_3"],
    #     sensor_name="h2"
    #     )

    main(args)
