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
    ax = figure.add_subplot(projection="3d")

    tool_1_id = "tool_1"
    tool_2_id = "tool_2"
    tool_3_id = "tool_3"

    args_1 = argparse.Namespace(
        sub_ip=args.sub_ip,
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
    # zmq_manager_2.initialize()

    i = 0
    try:
        while True:
            # Running the main loop

            # receive messages
            poseinfo_sensor1 = zmq_manager_1.receive_poses()
            if len(poseinfo_sensor1) != 0:
                pose_graph.update_graph("h1",poseinfo_sensor1)
                print("graph nodes: ", pose_graph.nodes)
                pass

            else:
                print("poseinfo_sensor1 is empty")

            # poseinfo_sensor2 = zmq_manager_2.receive_poses()
            # if len(poseinfo_sensor2) != 0:
            #     print("poseinfo: ", poseinfo_sensor2)

            # else:
            #     print("poseinfo_sensor2 is empty")

            # send messages
            for topic in args_1.pub_topic:
                # transfer the topic from bytes to string
                pose = pose_graph.get_transform("h1", topic, solver_method="BFS")
                if pose:
                    zmq_manager_1.send_poses(topic, pose)

            i += 1e-6
            
            # visualize the poses
            pose_graph.viz_graph(ax=ax, world_frame_id="h1", axis_limit=1.0)
            
            plt.pause(0.001)
            plt.show()
            # test update poses
            try:
                tool1_pose = poseinfo_sensor1['tool_1'][0]
                tool2_pose = tool1_pose.copy()
                tool2_quat = Rot.from_matrix(tool2_pose[:3, :3]).as_quat().reshape(1, -1)
                tool2_trans = tool2_pose[:3, 3].reshape(1, -1)
                tool2_pose = np.hstack([tool2_trans, tool2_quat])
                tool2_pose[:,:3] += np.array([-0.03, 0.0, 0.0])
                tool2_pose[:,2] *= -1
                tool2_pose_str = ",".join(str(num) for num in tool2_pose.flatten())
                # print(tool1_pose)
                zmq_manager_1.pub_messages["tool_2"] = tool2_pose_str+',0'
            except:
                pass
            
    except KeyboardInterrupt:
        # terminate_ZMQManager(zmq_manager_1, sub_thread_1, pub_thread_1)
        zmq_manager_1.terminate()
        # zmq_manager_2.terminate()
        # terminate_ZMQManager(zmq_manager_2, sub_thread_2, pub_thread_2)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Python script for running the AR tool tracking",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--sub_ip", default="10.203.59.134", type=str, help="subscriber ip address"
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
