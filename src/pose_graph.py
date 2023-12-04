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
        self.sensor_id = []
        self.object_id = []

    def _add_node(self, node_id):
        """
        Add a node to the graph, private method
        """

        self.nodes.append(node_id)

    def _add_edge(
        self,
        parent_id: str,
        child_id: str,
        tranformation: np.ndarray,
        isActive: bool = False,
    ):
        """
        Add an edge between the parent node and the child node, private method
        """
        self.edges[parent_id] = [child_id, tranformation, isActive]

    def add_sensor(self, sensor_id: str, poses: Dict[str, Tuple[np.ndarray, bool]]):
        """
        Add a sensor as a node in the graph, and add the corresponding edge between the sensor and the objects in the graph

        Args:
        ----------
            sensor_id: str, id of the sensor
            poses: Dict[str, Tuple[bool, np.ndarray(np.float32, (4, 4))]], a dictionary of poses,
                                         the key is the object id, the value is a tuple of the pose
                                         and a bool indicating if the pose is active,
                                         use tuple to make the pose immutable and for the efficiency of traversal
        """

        # check if the sensor_id is already in the graph
        if sensor_id in self.sensor_id:
            print("The sensor id is already in the graph")
            return

        self.sensor_id.append(sensor_id)

        self.add_node(sensor_id)

        for pose in poses.items():
            # check if the object_id is already in the graph,
            # if not, add it to the graph
            if pose[0] not in self.object_id:
                self.add_object(pose[0])

            self.add_edge(sensor_id, pose[0], pose[1][0], pose[1][1])

    def add_object(self, object_id):
        """
        Add an object as a node in the graph
        """
        if object_id in self.object_id:
            print("The object id is already in the graph")
            return

        self.object_id.append(object_id)
        self.add_node(object_id)

    def update_graph(self, parent_id, poses: Dict[str, Tuple[np.ndarray, bool]]):
        """
        Update the transformation between the parent node and the child node
        """
        if parent_id not in self.sensor_id:
            self.add_sensor(parent_id, poses)
            return
        else:
            # update the transformation between the parent node and the child node
            pass

        # TODO: need to consider the case when the object is not in the graph
        # TODO: need to consider the case when the object is no longer visible for the sensor

    def get_transformation(self, parent_id: str, child_id: str):
        """
        Get the transformation between the parent node and the child node
        """
        if parent_id not in self.sensor_id:
            print("The parent node is not in the graph")
            return

        if child_id not in self.object_id:
            print("The child node is not in the graph")
            return

        if child_id not in self.edges[parent_id]:
            # The edge between the parent node and the child node is not in the graph
            # TODO: graph search algorithm (DFS or BFS) to find a path between the parent node and the child node
            # TODO: if the path is not found, return None
            return

        return self.edges[parent_id][child_id]


class PoseDescriptor:
    def __init__(
        self,
        sensor_id: str,
        object_id: str,
        timestamp: float,
        pose: np.ndarray,
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

    poses = {
        "cam1": (1, "M"),
        "cam2": (2, "N"),
        "cam3": (3, "O"),
        "cam4": (4, "P"),
        "cam5": (5, "Q"),
    }

    for pose in poses.items():
        print(pose[1][1])
