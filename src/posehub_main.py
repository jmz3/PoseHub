from .pose_graph import PoseGraph
from .comm import *
from typing import Type, Dict, List, Optional


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


def main():
    """
    Main function
    """
    # initialize the pose graph
    pose_graph = PoseGraph()

    # initialize the communication
    port = 5000
    # ip =
    # initialize the message
    msg = "hello world"

    # receive poses from the sensors
    receive_poses(port, msg)


if __name__ == "__main__":
    main()
