import numpy as np
from transform_solver import TransformSolver
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
                         e.g. edges = {parent_id: {child_id: [transformation, isActive]}, ...}
                              edges[parent_id] = {child_id, [transformation, isActive]}
            sensor_id: list[str] of all the sensor ids
            object_id: list[str] of all the object ids

        """
        self.nodes = []
        self.edges = dict([])
        self.sensor_id = []
        self.object_id = []
        self.transform_solver = TransformSolver(self.nodes, self.edges)

    def _add_node(self, node_id):
        """
        Add a node to the graph, private method
        The nodes are represented as a list
        """

        self.nodes.append(node_id)

    def _add_edge(
        self,
        parent_id: str,
        child_id: str,
        transformation: np.ndarray,
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
        if parent_id not in self.edges:
            self.edges[parent_id] = {}
        if child_id not in self.edges:
            self.edges[child_id] = {}

        self.edges[parent_id][child_id] = [transformation, isActive]
        self.edges[child_id][parent_id] = [np.linalg.inv(transformation), isActive]

    def add_sensor(self, sensor_id: str, poses: Dict[str, Tuple[np.ndarray, bool]]):
        """
        Add a sensor as a node in the graph, and add the corresponding edge between the sensor and the objects in the graph

        Args:
        ----------
            sensor_id: str, id of the sensor
            poses: Dict[str, List[np.ndarray(np.float32, (4, 4)), bool]], a dictionary of poses,
                                         the key is the object id, the value is a tuple of the pose
                                         and a bool indicating if the pose is active,
                                         use tuple to make the pose immutable and for the efficiency of traversal
        """
        sensor_id_prefix = (
            "sensor_" + sensor_id
        )  # add a prefix to the sensor id to distinguish it from the object id
        # check if the sensor_id is already in the graph
        if sensor_id_prefix in self.sensor_id:
            print(
                "The sensor "
                + sensor_id_prefix
                + " is already in the graph \nPlease use update_graph() to update the edges"
            )
            return

        self.sensor_id.append(sensor_id_prefix)

        self._add_node(sensor_id_prefix)

        for pose in poses.items():
            # check if the object_id is already in the graph,
            # if not, add it to the graph
            object_id_prefix = "object_" + pose[0]
            if object_id_prefix not in self.object_id:
                self.add_object(pose[0])

            # add the edge between the sensor and the object
            self._add_edge(
                sensor_id_prefix,
                object_id_prefix,
                transformation=pose[1][0],
                isActive=pose[1][1],
            )

    def add_object(self, object_id):
        """
        Add an object as a node in the graph
        """
        object_id_prefix = (
            "object_" + object_id
        )  # add a prefix to the object id to distinguish it from the sensor id
        if object_id_prefix in self.object_id:
            print("The object " + object_id + " is already in the graph")
            return

        self.object_id.append(object_id_prefix)
        self._add_node(object_id_prefix)

    def update_graph(self, sensor_id, poses: Dict[str, Tuple[np.ndarray, bool]]):
        """
        Update the transformation between the parent node and the child node
        """

        object_ids = list(poses.keys())
        sensor_id = "sensor_" + sensor_id

        if sensor_id not in self.sensor_id:
            print(sensor_id + " is not in the graph, add it to the graph ... ")
            self.add_sensor(sensor_id, poses)
            print("The parent node is not in the graph, add it to the graph ... ")
            return
        else:
            # update the transformation between the parent node and the child node
            # Only the edges are updated, the nodes stay the same

            for object_id in object_ids:
                object_id_prefix = "object_" + object_id
                try:
                    self.edges[sensor_id][object_id_prefix][0] = poses[object_id][0]
                    self.edges[sensor_id][object_id_prefix][1] = poses[object_id][1]
                except KeyError:
                    # Tackle the case when the object is not in the graph
                    # TODO: add the object to the graph
                    print("The object is not in the graph, add it to the graph ... ")

        # TODO: need to consider the case when the object is no longer visible for the sensor
        # Is it necessary to remove the edge between the sensor and the object?
        # Is a flag enough to indicate if the transformation is active?

    def get_transform(
        self, parent_id: str, child_id: str, solver_method: str = "SET"
    ) -> Optional[np.ndarray]:
        """
        Get the transformation between the parent node and the child node
        """
        parent_id = "sensor_" + parent_id
        child_id = "object_" + child_id

        if parent_id not in self.sensor_id:
            print(
                "The parent node is not in the graph,\nPlease check the input or add the node to the graph"
            )
            return None

        if child_id not in self.object_id:
            print(
                "The child node is not in the graph,\nPlease check the input or add the node to the graph"
            )
            return None

        if (
            child_id not in self.edges.get(parent_id, {})
            or self.edges[parent_id][child_id][1] == False
        ):
            # The edge between the parent node and the child node is not in the graph or is not active
            # Need to find a path between the parent node and the child node
            # TODO: if the path is not found, return None

            transform = np.eye(4, dtype=np.float32)

            self.transform_solver.update(self.nodes, self.edges)
            path = self.transform_solver.solve(
                parent_id, child_id, method=solver_method
            )

            if path is None:
                print("No path is found between the parent node and the child node")
                return None

            if len(path) == 1:
                print(
                    "The parent node and the child node are directly connected, please check the input"
                )
                return None

            else:
                print("The path between the parent node and the child node is: ", path)
                for idx in range(len(path) - 1):
                    # print(transform)

                    transform = transform @ self.edges[path[idx]][path[idx + 1]][0]

                # return the transformation based on the path

                return transform

        elif (
            child_id in self.edges.get(parent_id, {})
            and self.edges[parent_id][child_id][1] == True
        ):  # checking the flag, not properly implemented
            # The edge between the parent node and the child node is in the graph and is active
            # directly return the transformation
            print(
                "The edge between the parent node and the child node is active.\nDirectly return the transformation"
            )
            transform = self.edges[parent_id][child_id][0]
            return transform

        else:
            raise RuntimeError("Unexpected error when getting the transformation")

        return None


if __name__ == "__main__":
    pass
