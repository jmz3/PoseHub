import numpy as np
from typing import Type, Dict, List, Optional, Tuple


class PoseGraph:
    """
    PoseGraph class stores the transformation between all the sensors and the objects in the scene
    the graph is a directled represented as a list of nodes and a list of edges

    """

    def __init__(self):
        """
        Initialize the PoseGraph class, the graph is directed and

        Members:
            nodes: list, all the nodes in the graph, each node is a sensor or an object
            edges: dict, all the edges in the graph, each edge is a transformation between a sensor and an object
                         edges[parent_id] = [child_id, transformation, isActive]
            sensor_id: list[str] of all the sensor ids
            object_id: list[str] of all the object ids

        """
        self.nodes = []
        self.edges = dict([])
        self.sensor_id = None
        self.object_id = None

    def add_node(self, node_id):
        self.nodes.append(node_id)

    def add_edge(
        self,
        parent_id: str,
        child_id: str,
        tranformation: np.ndarray(np.float32, (4, 4)),
    ):
        """
        Add an edge between the parent node and the child node
        """
        self.edges[parent_id] = [child_id, tranformation, isActive]

    def add_sensor(
        self, sensor_id, poses: Dict[str, Tuple[bool, np.ndarray(np.float32, (4, 4))]]
    ):
        """
        Add a sensor as a node in the graph, and add the corresponding edge between the sensor and the objects in the graph

        Args:
        ----------
            sensor_id: str, id of the sensor
            poses: Dict[str, Tuple[bool, np.ndarray(np.float32, (4, 4))]], a dictionary of poses,
                                         the key is the object id, the value is a tuple of the pose
                                         and a bool indicating if the pose is active
        """

        # check if the sensor_id is already in the graph
        if sensor_id in self.sensor_id:
            print("The sensor id is already in the graph")
            return

        self.sensor_id.append(sensor_id)

        self.add_node(sensor_id)

        for pose in poses:
            self.add_edge(sensor_id, pose.object_id, pose.pose)

    def add_object(self, object_id):
        self.object_id.append(object_id)
        self.add_node(object_id)

    def update_graph(self, parent_id, child_id, transformation):
        """
        Update the transformation between the parent node and the child node
        """
        self.edges[parent_id][child_id][1] = transformation


class PoseDescriptor:
    def __init__(
        self,
        sensor_id: str,
        object_id: str,
        timestamp: float,
        pose: np.ndarray(np.float32, (4, 4)),
    ):
        """
        Initialize the PoseDescriptor class

        Members:
            sensor_id: str, id of the sensor
            timestamp: float, timestamp of the pose
            pose: np.ndarray(np.float32, (4, 4)), pose of the object in the sensor frame
        """

        self.sensor_id = sensor_id
        self.object_id = object_id
        self.timestamp = timestamp
        self.pose = pose


if __name__ == "__main__":
    sensor_id = ["cam0", "cam1", "imu0", "imu1"]

    pose_id = ["cam1", "cam2", "cam3", "cam4", "cam5"]

    for pose in pose_id:
        if pose in sensor_id:
            print("yes")
