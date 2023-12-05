import numpy as np
from typing import Type, Dict, List, Optional, Tuple


class PoseGraph:
    """
    PoseGraph class stores the transformation between all the sensors and the objects in the scene
    the graph is a directled represented as a list of nodes and a list of edges

    """

    def __init__(self):
        """
        Initialize the PoseGraph class, the graph is undirected and the edges are represented as an adjacency list

        Members:
            nodes: list, all the nodes in the graph, each node is a sensor or an object
                         nodes = [[property, id_0], [property, id_1], [property, id_2], ...]
                         e.g. nodes = [[sensor, "sensor_id"], [obejct, "object_id"], ["object", "object_id"]]
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

        The edges are represented as an adjacency list,
        therefore the edge is added to both the parent node and the child node

        Args:
        ----------
            parent_id: str, id of the parent node
            child_id: str, id of the child node
            tranformation: np.ndarray(np.float32, (4, 4)), transformation between the parent node and the child node
            isActive: bool, indicate if the transformation is active
        """
        self.edges[parent_id] = [child_id, tranformation, isActive]
        self.edges[child_id] = [parent_id, np.linalg.inv(tranformation), isActive]

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
            print(
                "The sensor "
                + sensor_id
                + " is already in the graph \nPlease use update_graph() to update the edges"
            )
            return

        self.sensor_id.append(sensor_id)

        self.add_node(sensor_id)

        for pose in poses.items():
            # check if the object_id is already in the graph,
            # if not, add it to the graph
            if pose[0] not in self.object_id:
                self.add_object(pose[0])

            # add the edge between the sensor and the object
            self.add_edge(sensor_id, pose[0], pose[1][0], pose[1][1])

    def add_object(self, object_id):
        """
        Add an object as a node in the graph
        """
        if object_id in self.object_id:
            print("The object" + object_id + " is already in the graph")
            return

        self.object_id.append(object_id)
        self.add_node(object_id)

    def update_graph(self, sensor_id, poses: Dict[str, Tuple[np.ndarray, bool]]):
        """
        Update the transformation between the parent node and the child node
        """
        if sensor_id not in self.sensor_id:
            self.add_sensor(sensor_id, poses)
            print("The parent node is not in the graph, add it to the graph ... ")
            return
        else:
            # update the transformation between the parent node and the child node
            # Only the edges are updated, the nodes stay the same
            object_transform = self.edges[
                sensor_id
            ]  # A list of [child_id, transformation, isActive]

            for objects in self.edges[sensor_id]:
                # check if the input poses contains the object
                if objects[0] not in poses.keys():
                    raise RuntimeWarning(
                        "The input poses does not contain the object that is already in the graph"
                    )
                objects[1] = poses[objects[0]][0]  # update the transformation
                objects[2] = poses[objects[0]][1]  # update the isActive flag

            pass

            # Consider the case when the object is not in the graph
            for object_id in poses.keys():
                if object_id not in self.object_id and poses[object_id][1] == True:
                    # add the object to the graph
                    self.add_object(object_id)
                    # create the edge between the sensor and the object
                    self.add_edge(sensor_id, object_id, poses[object_id][0])

        # TODO: need to consider the case when the object is no longer visible for the sensor
        # Is it necessary to remove the edge between the sensor and the object?
        # Is a flag enough to indicate if the transformation is active?

    def get_transform(self, parent_id: str, child_id: str):
        """
        Get the transformation between the parent node and the child node
        """
        if parent_id not in self.sensor_id:
            print(
                "The parent node is not in the graph,\nPlease check the input or add the node to the graph"
            )
            return

        if child_id not in self.object_id:
            print(
                "The child node is not in the graph,\nPlease check the input or add the node to the graph"
            )
            return

        if (
            child_id not in self.edges[parent_id]
            or self.edges[parent_id][child_id][2] == False
        ):
            # The edge between the parent node and the child node is not in the graph or is not active
            # TODO: graph search algorithm (DFS or BFS) to find a path between the parent node and the child node
            # TODO: if the path is not found, return None
            return

        elif (
            child_id in self.edges[parent_id]
            and self.edges[parent_id][child_id][2] == True
        ):
            transform = self.edges[parent_id][child_id][1]

        else:
            raise RuntimeError("Unexpected error when getting the transformation")

        return transform


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
    pass
