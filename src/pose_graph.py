import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from transform_solver import TransformSolver
from typing import Type, Dict, List, Optional, Tuple
from itertools import cycle


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

    def add_sensor(
        self, sensor_id: str, poses: Optional[Dict[str, Tuple[np.ndarray, bool]]] = None
    ):
        """
        Add a sensor as a node in the graph. If poses are provided, add the corresponding edges between the sensor and the objects in the graph.

        Args:
        ----------
            sensor_id: str, id of the sensor, the sensor id should be unique
                            input the sensor id without the prefix "sensor_"
                            the prefix will be added in the function
            poses: Optional[Dict[str, Tuple[np.ndarray, bool]]], a dictionary of poses,
                            the key is the object id, the value is a tuple of the pose
                            and a bool indicating if the pose is active,
                            use tuple to make the pose immutable and for the efficiency of traversal
                            Default is None, which means no connected objects are specified.
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

        if poses:
            # for pose in poses.items():
            # check if the object_id is already in the graph,
            # if not, add it to the graph
            # object_id_prefix = "object_" + pose[0]
            # if object_id_prefix not in self.object_id:
            #     self.add_object(pose[0])

            # # add the edge between the sensor and the object
            # self._add_edge(
            #     sensor_id_prefix,
            #     object_id_prefix,
            #     transformation=pose[1][0],
            #     isActive=pose[1][1],
            # )
            for object_id, (transformation, isActive) in poses.items():
                object_id_prefix = "object_" + object_id
                if object_id_prefix not in self.object_id:
                    self.add_object(object_id)

                self._add_edge(
                    sensor_id_prefix,
                    object_id_prefix,
                    transformation=transformation,
                    isActive=isActive,
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

        Args:
        ----------
            sensor_id: str, id of the sensor
            poses: Dict[str, List[np.ndarray(np.float32, (4, 4)), bool]], a dictionary of poses,
                                         the key is the object id, the value is a tuple of the pose
                                         and a bool indicating if the pose is active,
                                         use tuple to make the pose immutable and for the efficiency of traversal

        """

        object_ids = list(poses.keys())
        sensor_id_prefix = "sensor_" + sensor_id

        if sensor_id_prefix not in self.sensor_id:
            # print(sensor_id_prefix + " is not in the graph, add it to the graph ... ")
            # self.add_sensor(sensor_id, poses)
            self.add_sensor(sensor_id, poses)
            return
        else:
            # update the transformation between the parent node and the child node
            # Only the edges are updated, the nodes stay the same

            for object_id in object_ids:
                object_id_prefix = "object_" + object_id
                try:
                    # update the tf between the sensor and the object
                    self.edges[sensor_id_prefix][object_id_prefix] = poses[object_id]

                    # update the tf from the object to the sensor, the tf is the inverse of the tf from the sensor to the object
                    self.edges[object_id_prefix][sensor_id_prefix][0] = np.linalg.inv(
                        poses[object_id][0]
                    )
                    self.edges[object_id_prefix][sensor_id_prefix][1] = poses[
                        object_id
                    ][1]
                except KeyError:
                    # Tackle the case when the object is not in the graph
                    # TODO: add the object to the graph
                    # print(
                    #     "The object",
                    #     object_id_prefix,
                    #     " is not in the graph, add it to the graph ... ",
                    # )
                    for object_id, (transformation, isActive) in poses.items():
                        object_id_prefix = "object_" + object_id
                        if object_id_prefix not in self.object_id:
                            self.add_object(object_id)

                        self._add_edge(
                            sensor_id_prefix,
                            object_id_prefix,
                            transformation=transformation,
                            isActive=isActive,
                        )

        # TODO: need to consider the case when the object is no longer visible for the sensor
        # Is it necessary to remove the edge between the sensor and the object?
        # Is a flag enough to indicate if the transformation is active?

    def get_transform(
        self, parent_id: str, child_id: str, solver_method: str = "SET"
    ) -> Optional[np.ndarray]:
        """
        Get the transformation between the parent node and the child node

        Args:
        ----------
            parent_id: str, id of the parent node
            child_id: str, id of the child node
            solver_method: str, method to solve the transformation between the parent node and the child node

        Return:
        ----------
            transform: np.ndarray(np.float32, (4, 4)), transformation between the parent node and the child node
        """
        parent_id = "sensor_" + parent_id
        child_id = "object_" + child_id

        if parent_id not in self.sensor_id:
            # print(
            #     "The parent node is not in the graph,\nPlease check the input or add the node to the graph"
            # )
            return None

        if child_id not in self.object_id:
            # print(
            #     "The child node is not in the graph,\nPlease check the input or add the node to the graph"
            # )
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
                # print("The path between the parent node and the child node is: ", path)
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

    def viz_graph(
        self,
        ax: plt.Axes,
        world_frame_id: str,
        axis_limit: float = 0.5,
        frame_type: [Optional[str]] = "object",
    ):
        """
        Visualize the graph

        Args:
        ----------
            ax: plt.Axes,   the axes to plot the graph
            world_frame: str,   the id of the world frame,
                                the world frame is the frame you choose to remain stationary in the plot
            axis_limit: float,  the limit of the axis, default is 0.5
            frame_type: str,    the type of the frame to plot, default is object, can be sensor or object

        """
        # define a 3d frame, the origin is at the center of the world frame
        # the x axis is red, the y axis is green, the z axis is blue
        # the length of each axis is axis_length

        # display frequency is up to 100Hz

        # clear the plot
        ax.cla()
        ax.set_xlabel("X", fontsize=12)
        ax.set_ylabel("Y", fontsize=12)
        ax.set_zlabel("Z", fontsize=12)
        ax.set_xlim(-axis_limit, axis_limit)
        ax.set_ylim(-axis_limit, axis_limit)
        ax.set_zlim(-axis_limit, axis_limit)
        ax.set_title("Pose Graph", fontsize=20)
        marker_styles = [
            "o",
            "s",
            "^",
            "v",
            "*",
            "+",
            "x",
            "D",
            "|",
            "_",
        ]  # 10 styles, can be extended if needed
        marker_colors = [
            "r",
            "g",
            "b",
            "c",
            "m",
            "y",
            "k",
            "w",
        ]  # 8 colors, can be extended if needed

        # construct the world frame
        world_frame_id = frame_type + "_" + world_frame_id

        if world_frame_id not in self.nodes:
            # print(
            #     "The world frame "
            #     + world_frame_id
            #     + " is not in the graph, please check the input"
            # )  # do not plot if the world frame is not in the graph
            return

        if len(self.edges) == 0 or len(self.nodes) == 1:  # if the graph has no edges:
            return  # do not plot if the graph has no other elements

        axis_length = 0.3

        # define the world frame
        x_axis = np.array(
            [[axis_length, 0, 0, 1], [0, 0, 0, 1]]
        )  # homogeneous coordinates of x
        y_axis = np.array(
            [[0, axis_length, 0, 1], [0, 0, 0, 1]]
        )  # homogeneous coordinates of y
        z_axis = np.array(
            [[0, 0, axis_length, 1], [0, 0, 0, 1]]
        )  # homogeneous coordinates of z

        # plot the world frame
        ax.plot(x_axis[:, 0], x_axis[:, 1], x_axis[:, 2], "r")
        ax.plot(y_axis[:, 0], y_axis[:, 1], y_axis[:, 2], "g")
        ax.plot(z_axis[:, 0], z_axis[:, 1], z_axis[:, 2], "b")
        ax.text(
            x_axis[0, 0],
            x_axis[0, 1],
            x_axis[0, 2],
            "World Frame",
            color="black",
            fontsize=12,
        )

        sCount = 0
        # apply transformation to the frame
        for sensor in self.sensor_id:
            sCount += 1

            world2sensor = (
                self.edges[world_frame_id][sensor][0]
                if frame_type == "object"
                else np.identity(4)
            )

            for object in self.edges[sensor]:
                if object == world_frame_id:
                    continue  # skip the world frame itself

                transform = self.edges[sensor][object][0]
                x_axis_temp = world2sensor @ transform @ x_axis.T
                y_axis_temp = world2sensor @ transform @ y_axis.T
                z_axis_temp = world2sensor @ transform @ z_axis.T

                x_axis_temp = x_axis_temp.T
                y_axis_temp = y_axis_temp.T
                z_axis_temp = z_axis_temp.T

                # plot the transformed frame
                ax.plot(x_axis_temp[:, 0], x_axis_temp[:, 1], x_axis_temp[:, 2], "r")
                ax.plot(y_axis_temp[:, 0], y_axis_temp[:, 1], y_axis_temp[:, 2], "g")
                ax.plot(z_axis_temp[:, 0], z_axis_temp[:, 1], z_axis_temp[:, 2], "b")

                # show the text of the object id next to the object frame origin
                ax.text(
                    x_axis_temp[1, 0],
                    x_axis_temp[1, 1],
                    x_axis_temp[1, 2] + axis_length + 0.05,
                    object,
                    color="black",
                    bbox=dict(facecolor="red", alpha=0.5),
                    fontsize=12,
                )
                ax.scatter(
                    x_axis_temp[0, 0] + 0.05 * sCount,
                    x_axis_temp[0, 1],
                    x_axis_temp[0, 2],
                    marker=marker_styles[sCount % len(marker_styles)],
                    color=marker_colors[sCount % len(marker_colors)],
                    s=50,
                    label=sensor,
                )

            # print(y_axis)
        ax.legend(loc="best", fontsize=12)
        # print(transform)


if __name__ == "__main__":
    pass
