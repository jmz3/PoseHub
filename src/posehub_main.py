# from pose_graph import PoseGraph
from comm import *
from typing import Type, Dict, List, Optional
from comm.ZMQManager import ZMQManager
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
    # pose_graph = PoseGraph()

    # # initialize communication managers
    zmq_manager_1 = ZMQManager(args.sub_ip, args.sub_port, args.pub_port, args.sub_topic, args.pub_topic, args.sensor_name)
    zmq_manager_1.initialize()
    # zmq_manager_2, sub_thread_2, pub_thread_2 = initialize_ZMQManager(args.sub_ip, args.sub_port, args.pub_port, args.sub_topic, args.pub_topic)
    i = 0
    try:
        while True:
            # Running the main loop

            # test receiving poses
            # print('tool 1: ', zmq_manager_1.sub_poses['tool_1'])
            poseinfo = zmq_manager_1.receive_poses()
            # print(poseinfo)
            # if len(poseinfo) != 0:
            #     print(poseinfo['tool_1'])
            # poseinfo should be a dictionary with key as object name and value is [pose, isActive]
            
            # else:
            #     print("no poseinfo received")
            # print('tool 2: ', zmq_manager_1.sub_poses['tool_2'])
            # print('tool 3: ', zmq_manager_1.sub_poses['tool_3'])

            # test sending messages
            # zmq_manager_1.pub_messages["tool_2"] = f"topic4 test message {i}"
            # zmq_manager_1.pub_messages["topic5"] = f"topic5 test message {i}"
            # zmq_manager_1.pub_messages["topic6"] = f"topic6 test message {i}"
            i += 1e-6
            
            # test update poses
            try:
                tool1_pose = poseinfo['tool_1'][0]
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

    args_1 = argparse.Namespace(
    sub_ip=args.sub_ip,
    sub_port="5588",
    pub_port="5589",
    sub_topic=["tool_1", "tool_2", "tool_3"],
    pub_topic=["tool_1", "tool_2", "tool_3"],
    sensor_name="h1"
    ) # args for h1 sensor

    args_2 = argparse.Namespace(    
        sub_ip="10.203.150.51",
        sub_port="5589",
        pub_port="5580",
        sub_topic=["tool_1", "tool_2", "tool_3"],
        pub_topic=["tool_1", "tool_2", "tool_3"],
        sensor_name="h2"
        )
    
    main(args_1)
