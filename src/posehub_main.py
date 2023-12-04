from .pose_graph import PoseGraph
from .comm import *
from typing import Type, Dict, List, Optional
from .ZMQManager import ZMQManager
import argparse

def receive_poses(port: str, msg):
    """
    Receive poses from the sensors
    """
    pass


def send_poses(ip: str, port: str, msg):
    """
    Send poses to the sensors
    """
    pass

    
def main(args):
    """
    Main function
    """
    # initialize the pose graph
    pose_graph = PoseGraph()

    # initialize the communication
    zmq_manager = ZMQManager(args.sub_ip, args.sub_port, args.pub_port, args.sub_topic, args.pub_topic)
    zmq_manager.run()
    
    # initialize the message
    # msg = "hello world"

    # receive poses from the sensors
    # receive_poses(port, msg)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Python script for running the AR tool tracking',
                                    formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("--sub_ip", default="192.168.1.23", type=str, help="subscriber ip address")
    parser.add_argument("--sub_port", default="5588", type=str, help="port number for subscriber")
    parser.add_argument("--pub_port", default="5589", type=str, help="port number for publisher")
    parser.add_argument("--sub_topic", default=[b"Tool 1", b"Tool 2", b"Tool 3"], type=str, help="subscriber topics")
    parser.add_argument("--pub_topic", default=[b"topic2",b"topic3",b"topic4"], type=str, help="publisher topic")
    args = parser.parse_args()
    
    main(args)
