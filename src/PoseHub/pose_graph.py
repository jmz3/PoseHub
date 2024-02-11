import numpy as np
import matplotlib.pyplot as plt

from transform_solver import TransformSolver
from typing import Type, Dict, List, Optional, Tuple
from enum import Enum


class FrameType(Enum):
    OBJECT = 1
    SENSOR = 2
    # Add more types as needed


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
            print("create parent node in edges: ", parent_id)
            self.edges[parent_id] = {}
        if child_id not in self.edges:
            print("create child node in edges: ", child_id)
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
            for object_id in poses.keys():
                object_id_prefix = "object_" + object_id
                if object_id_prefix not in self.object_id:
                    self.add_object(object_id)

                if len(poses[object_id]) == 0:
                    self._add_edge(
                        sensor_id_prefix,
                        object_id_prefix,
                        transformation=np.identity(4),
                        isActive=False,
                    )
                else:
                    self._add_edge(
                        sensor_id_prefix,
                        object_id_prefix,
                        transformation=poses[object_id][0],
                        isActive=poses[object_id][1],
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
            # self.add_sensor(sensor_id, poses)
            self.add_sensor(sensor_id, poses)
            return
        else:
            # update the transformation between the parent node and the child node
            # Only the edges are updated, the nodes stay the same

            for object_id in object_ids:
                object_id_prefix = "object_" + object_id

                if object_id_prefix not in self.object_id:
                    print(
                        "The object",
                        object_id_prefix,
                        " is not in the graph, add it to the graph ... ",
                    )
                    self.add_object(object_id)
                    self._add_edge(
                        sensor_id_prefix, object_id_prefix, np.identity(4), False
                    )

                elif len(poses[object_id]) == 0:
                    self.edges[sensor_id_prefix][object_id_prefix] = [
                        np.identity(4),
                        False,
                    ]
                    self.edges[object_id_prefix][sensor_id_prefix] = [
                        np.identity(4),
                        False,
                    ]
                # update the tf from the object to the sensor, the tf is the inverse of the tf from the sensor to the object
                else:
                    self.edges[sensor_id_prefix][object_id_prefix] = [
                        np.identity(4),
                        False,
                    ]
                    self.edges[object_id_prefix][sensor_id_prefix] = [
                        np.identity(4),
                        False,
                    ]
                    self.edges[sensor_id_prefix][object_id_prefix][0] = poses[
                        object_id
                    ][0]
                    self.edges[sensor_id_prefix][object_id_prefix][1] = poses[
                        object_id
                    ][1]

                    self.edges[object_id_prefix][sensor_id_prefix][0] = np.linalg.inv(
                        poses[object_id][0]
                    )
                    self.edges[object_id_prefix][sensor_id_prefix][1] = poses[
                        object_id
                    ][1]
                #     print("object id: ", object_id_prefix)
                #     print("sensor id: ", sensor_id_prefix)

                    # print("edges: ", self.edges[sensor_id_prefix][object_id_prefix])
                # print("edge id: ", self.edges[sensor_id_prefix])

                # Tackle the case when the object is not in the graph
                # TODO: add the object to the graph

                # if len(poses[object_id]) == 0:
                #     # update the tf between the sensor and the object

                #     self.add_object(object_id)
                #     print("object id: ", object_id_prefix)
                #     print("sensor id: ", sensor_id_prefix)
                #     print(
                #         "sensor in the graph? : ",
                #         sensor_id_prefix in self.sensor_id,
                #     )

                # else:
                #     self.add_object(object_id)
                #     print("object id: ", object_id_prefix)
                #     print("sensor id: ", sensor_id_prefix)
                #     print(
                #         "sensor in the graph? : ",
                #         sensor_id_prefix in self.sensor_id,
                #     )

                #     self._add_edge(
                #         sensor_id_prefix,
                #         object_id_prefix,
                #         transformation=poses[object_id][0],
                #         isActive=poses[object_id][1],
                #     )

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
        axis_length: float = 0.5,
        frame_type: [Optional[int]] = 1,
    ):
        """
        Visualize the graph in real time, the visualization only shows the frames
        The frames are represented as the x, y, z axes
        The world frame is the frame that remains stationary in the plot
        Scatter dots are used to represent which sensor can see which object

        Args:
        ----------
            ax: plt.Axes,   the axes to plot the graph
            world_frame: str,   the id of the world frame,
                                the world frame is the frame you choose to remain stationary in the plot
            axis_limit: float,  the limit of the axis, default is 0.5
            frame_type: int,    the type of the frame, 1 for object frame, 2 for sensor frame

        """
        # define a 3d frame, the origin is at the center of the world frame
        # the x axis is red, the y axis is green, the z axis is blue
        # the length of each axis is axis_length
        ax.cla()
        ax.set_xlabel("x")
        ax.set_ylabel("y")
        ax.set_zlabel("z")
        ax.set_xlim(-1, 1)
        ax.set_ylim(-1, 1)
        ax.set_zlim(-1, 1)
        ax.set_title("Pose Graph", fontsize=20)

        # display frequency is up to 100Hz

        # clear the plot

        # define the marker styles and colors
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

        # construct the world frame
        frame_type_enum = FrameType(frame_type)
        scatter = []

        if frame_type_enum == FrameType.OBJECT:
            # if the world frame is an object frame, the world frame is the object frame itself
            # plot the object frame and the whole graph, including the object frame and the sensor frames
            sCount = 0

            world_frame_id_prefix = "object" + "_" + world_frame_id
            if world_frame_id_prefix not in self.nodes:
                print(
                    "The world frame "
                    + world_frame_id_prefix
                    + " is not in the graph, please check the input"
                )
                return
            else:
                transform = np.identity(4)
                for sensor in self.sensor_id:
                    sCount += 1

                    if sensor not in self.edges[world_frame_id_prefix]:
                        continue  # skip the sensor if the edge is not in the graph
                    if sensor not in self.edges:
                        continue

                    world2sensor = self.edges[world_frame_id_prefix][sensor][0]

                    for object in self.edges[sensor]:
                        if object == world_frame_id_prefix:
                            continue  # skip the world frame itself

                        transform = self.edges[sensor][object][0]
                        x_axis_temp = world2sensor @ transform @ x_axis.T
                        y_axis_temp = world2sensor @ transform @ y_axis.T
                        z_axis_temp = world2sensor @ transform @ z_axis.T

                        x_axis_temp = x_axis_temp.T
                        y_axis_temp = y_axis_temp.T
                        z_axis_temp = z_axis_temp.T

                        # plot the transformed frame
                        ax.plot(
                            x_axis_temp[:, 0], x_axis_temp[:, 1], x_axis_temp[:, 2], "r"
                        )
                        ax.plot(
                            y_axis_temp[:, 0], y_axis_temp[:, 1], y_axis_temp[:, 2], "g"
                        )
                        ax.plot(
                            z_axis_temp[:, 0], z_axis_temp[:, 1], z_axis_temp[:, 2], "b"
                        )

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

                        scatter.append(None)  # create a placeholder for the scatter

                        scatter[sCount] = ax.scatter(
                            x_axis_temp[0, 0] + 0.05 * sCount,
                            x_axis_temp[0, 1],
                            x_axis_temp[0, 2],
                            marker=marker_styles[sCount % len(marker_styles)],
                            color=marker_colors[sCount % len(marker_colors)],
                            s=50,
                            label=sensor,
                        )

        elif frame_type_enum == FrameType.SENSOR:
            # if the world frame is a sensor frame,
            # only plot the frames of the objects that are visible for this sensor
            sCount = 0
            world_frame_id_prefix = "sensor" + "_" + world_frame_id
            if world_frame_id_prefix not in self.nodes:
                print(
                    "The world frame "
                    + world_frame_id_prefix
                    + " is not in the graph, please check the input"
                )
                return
            else:
                transform = np.identity(4)
                for object in self.object_id:
                    if object not in self.edges[world_frame_id_prefix]:
                        continue

                    world2object = self.edges[world_frame_id_prefix][object][0]
                    x_axis_temp = world2object @ x_axis.T
                    y_axis_temp = world2object @ y_axis.T
                    z_axis_temp = world2object @ z_axis.T

                    x_axis_temp = x_axis_temp.T
                    y_axis_temp = y_axis_temp.T
                    z_axis_temp = z_axis_temp.T

                    # plot the transformed frame
                    ax.plot(
                        x_axis_temp[:, 0], x_axis_temp[:, 1], x_axis_temp[:, 2], "r"
                    )
                    ax.plot(
                        y_axis_temp[:, 0], y_axis_temp[:, 1], y_axis_temp[:, 2], "g"
                    )
                    ax.plot(
                        z_axis_temp[:, 0], z_axis_temp[:, 1], z_axis_temp[:, 2], "b"
                    )

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

                    scatter.append(None)  # create a placeholder for the scatter

                    scatter[sCount] = ax.scatter(
                        x_axis_temp[0, 0] + 0.05 * sCount,
                        x_axis_temp[0, 1],
                        x_axis_temp[0, 2],
                        marker=marker_styles[sCount % len(marker_styles)],
                        color=marker_colors[sCount % len(marker_colors)],
                        s=50,
                        label=object,
                    )

        # apply transformation to the frame

        # print(y_axis)
        if scatter:
            ax.legend(
                loc="best", fontsize=12, fancybox=True
            )  # show legend, one legend for each sensor
        # print(transform)

    def viz_graph_update(
        self,
        frames: Dict[str, Dict[str, Dict[str, plt.Axes]]],
        frame_primitive: np.ndarray,
        world_frame_id: str,
        frame_type: [Optional[int]] = 1,
    ):
        """
        Visualize the graph in real time, the visualization only shows the frames
        The frames are represented as the x, y, z axes
        The world frame is the frame that remains stationary in the plot
        Scatter dots are used to represent which sensor can see which object

        Args:
        ----------
            frames: Dict[str, Dict[str, Dict[str, plt.Axes]]],  a dictionary of frames,
                                                                the key is the sensor id,
                                                                the value is a dictionary of objects,
                                                                the key is the object id,
                                                                the value is a dictionary of axes,
                                                                the key is the axis type,
                                                                the value is the axis
            frame_primitive: np.ndarray(np.float32, (4, 4)),    the primitive of the frame,
                                                                the frame primitive is a 4 x 4 matrix,
                                                                column-wise view [o, x, y, z],
            world_frame_id: str,   the id of the world frame,
                                the world frame is the frame you choose to remain stationary in the plot
            frame_type: int,    the type of the frame, 1 for object frame, 2 for sensor frame
        """
        # check whether the world frame is sensor frame or object frame
        frame_type_enum = FrameType(frame_type)
        axis_length = 0.3
        label_offset = 0.08
        world2sensor = np.identity(4)
        world2object = np.identity(4)
        frame_pm_temp = frame_primitive.copy()

        sCount = 0

        if frame_type_enum == FrameType.OBJECT:
            world_frame_id_prefix = "object" + "_" + world_frame_id

            if world_frame_id_prefix not in self.nodes:
                print(
                    "The world frame "
                    + world_frame_id_prefix
                    + " is not in the graph, please check the input"
                )
                return

            for sensor_prefix in self.sensor_id:
                if sensor_prefix not in self.edges[world_frame_id_prefix]:
                    frames[sensor_prefix][world_frame_id_prefix][
                        "sensor_tag"
                    ].set_alpha(0.0)
                    continue  # skip the sensor if the edge connected to world is not in the graph

                elif self.edges[world_frame_id_prefix][sensor_prefix][1] == False:
                    frames[sensor_prefix][world_frame_id_prefix][
                        "sensor_tag"
                    ].set_alpha(0.0)
                    continue  # skip the sensor if the edge connected to world is not active

                elif self.edges[world_frame_id_prefix][sensor_prefix][1] == True:
                    world2sensor[:, :] = self.edges[world_frame_id_prefix][
                        sensor_prefix
                    ][0]

                    for object_prefix in self.edges[sensor_prefix]:
                        if object_prefix == world_frame_id_prefix:
                            # hide the frame if the object is the world frame itself
                            frames[sensor_prefix][object_prefix]["x"].set_segments(
                                [
                                    [
                                        (
                                            0,
                                            0,
                                            0,
                                        ),
                                        (
                                            0,
                                            0,
                                            0,
                                        ),
                                    ]
                                ]
                            )
                            frames[sensor_prefix][object_prefix]["y"].set_segments(
                                [
                                    [
                                        (
                                            0,
                                            0,
                                            0,
                                        ),
                                        (
                                            0,
                                            0,
                                            0,
                                        ),
                                    ]
                                ]
                            )
                            frames[sensor_prefix][object_prefix]["z"].set_segments(
                                [
                                    [
                                        (
                                            0,
                                            0,
                                            0,
                                        ),
                                        (
                                            0,
                                            0,
                                            0,
                                        ),
                                    ]
                                ]
                            )
                            frames[sensor_prefix][object_prefix][
                                "sensor_tag"
                            ].set_alpha(0.0)

                            continue  # skip the world frame itself

                        if self.edges[sensor_prefix][object_prefix][1] == False:
                            # hide the frame if the edge is not active
                            frames[sensor_prefix][object_prefix]["x"].set_segments(
                                [
                                    [
                                        (
                                            0,
                                            0,
                                            0,
                                        ),
                                        (
                                            0,
                                            0,
                                            0,
                                        ),
                                    ]
                                ]
                            )
                            frames[sensor_prefix][object_prefix]["y"].set_segments(
                                [
                                    [
                                        (
                                            0,
                                            0,
                                            0,
                                        ),
                                        (
                                            0,
                                            0,
                                            0,
                                        ),
                                    ]
                                ]
                            )
                            frames[sensor_prefix][object_prefix]["z"].set_segments(
                                [
                                    [
                                        (
                                            0,
                                            0,
                                            0,
                                        ),
                                        (
                                            0,
                                            0,
                                            0,
                                        ),
                                    ]
                                ]
                            )
                            frames[sensor_prefix][object_prefix][
                                "sensor_tag"
                            ].set_alpha(0.0)
                            continue  # skip the object if the edge connected to the sensor is not active

                        # update the frame primitive pose
                        frame_pm_temp[:, :] = (
                            world2sensor
                            @ self.edges[sensor_prefix][object_prefix][0]
                            @ frame_primitive
                        )

                        # update the frame
                        frames[sensor_prefix][object_prefix]["x"].set_segments(
                            [
                                [
                                    (
                                        frame_pm_temp[0, 0],
                                        frame_pm_temp[1, 0],
                                        frame_pm_temp[2, 0],
                                    ),
                                    (
                                        frame_pm_temp[0, 1],
                                        frame_pm_temp[1, 1],
                                        frame_pm_temp[2, 1],
                                    ),
                                ]
                            ]
                        )
                        frames[sensor_prefix][object_prefix]["y"].set_segments(
                            [
                                [
                                    (
                                        frame_pm_temp[0, 0],
                                        frame_pm_temp[1, 0],
                                        frame_pm_temp[2, 0],
                                    ),
                                    (
                                        frame_pm_temp[0, 2],
                                        frame_pm_temp[1, 2],
                                        frame_pm_temp[2, 2],
                                    ),
                                ]
                            ]
                        )
                        frames[sensor_prefix][object_prefix]["z"].set_segments(
                            [
                                [
                                    (
                                        frame_pm_temp[0, 0],
                                        frame_pm_temp[1, 0],
                                        frame_pm_temp[2, 0],
                                    ),
                                    (
                                        frame_pm_temp[0, 3],
                                        frame_pm_temp[1, 3],
                                        frame_pm_temp[2, 3],
                                    ),
                                ]
                            ]
                        )
                        tag_position = np.column_stack(
                            (
                                frame_pm_temp[0, 0],
                                frame_pm_temp[1, 0],
                                frame_pm_temp[2, 0],
                            )
                        )

                        frames[sensor_prefix][object_prefix][
                            "sensor_tag"
                        ]._offsets3d = (
                            tag_position[:, 0],
                            tag_position[:, 1],
                            tag_position[:, 2] - label_offset * sCount,
                        )
                        frames[sensor_prefix][object_prefix]["sensor_tag"].set_alpha(
                            1.0
                        )

                        frames[sensor_prefix][object_prefix]["frame_id"].set_position(
                            [
                                frame_pm_temp[0, 0],
                                frame_pm_temp[1, 0],
                            ]
                        )
                        frames[sensor_prefix][object_prefix][
                            "frame_id"
                        ].set_3d_properties(
                            frame_pm_temp[2, 0] + 3 * label_offset,
                            zdir="x",
                        )
                        frames[sensor_prefix][object_prefix]["frame_id"].set_alpha(1.0)

                sCount += 1

        elif frame_type_enum == FrameType.SENSOR:
            world_frame_id_prefix = "sensor" + "_" + world_frame_id

            if world_frame_id_prefix not in self.nodes:
                print(
                    "The world frame "
                    + world_frame_id_prefix
                    + " is not in the graph, please check the input"
                )
                return

            for object_prefix in self.object_id:
                if object_prefix not in self.edges[world_frame_id_prefix]:
                    frames[world_frame_id_prefix][object_prefix]["x"].set_segments(
                        [
                            [
                                (
                                    0,
                                    0,
                                    0,
                                ),
                                (
                                    0,
                                    0,
                                    0,
                                ),
                            ]
                        ]
                    )
                    frames[world_frame_id_prefix][object_prefix]["y"].set_segments(
                        [
                            [
                                (
                                    0,
                                    0,
                                    0,
                                ),
                                (
                                    0,
                                    0,
                                    0,
                                ),
                            ]
                        ]
                    )
                    frames[world_frame_id_prefix][object_prefix]["z"].set_segments(
                        [
                            [
                                (
                                    0,
                                    0,
                                    0,
                                ),
                                (
                                    0,
                                    0,
                                    0,
                                ),
                            ]
                        ]
                    )
                    frames[world_frame_id_prefix][object_prefix][
                        "sensor_tag"
                    ].set_alpha(0.0)
                    continue

                elif self.edges[world_frame_id_prefix][object_prefix][1] == False:
                    # hide the frame if the edge is not active
                    frames[world_frame_id_prefix][object_prefix]["x"].set_segments(
                        [
                            [
                                (
                                    0,
                                    0,
                                    0,
                                ),
                                (
                                    0,
                                    0,
                                    0,
                                ),
                            ]
                        ]
                    )
                    frames[world_frame_id_prefix][object_prefix]["y"].set_segments(
                        [
                            [
                                (
                                    0,
                                    0,
                                    0,
                                ),
                                (
                                    0,
                                    0,
                                    0,
                                ),
                            ]
                        ]
                    )
                    frames[world_frame_id_prefix][object_prefix]["z"].set_segments(
                        [
                            [
                                (
                                    0,
                                    0,
                                    0,
                                ),
                                (
                                    0,
                                    0,
                                    0,
                                ),
                            ]
                        ]
                    )
                    frames[world_frame_id_prefix][object_prefix][
                        "sensor_tag"
                    ].set_alpha(0.0)
                    continue

                elif self.edges[world_frame_id_prefix][object_prefix][1] == True:
                    world2object[:, :] = self.edges[world_frame_id_prefix][
                        object_prefix
                    ][0]

                    # update the frame primitive pose
                    frame_pm_temp[:, :] = world2object @ frame_primitive

                    # update the frame
                    frames[world_frame_id_prefix][object_prefix]["x"].set_segments(
                        [
                            [
                                (
                                    frame_pm_temp[0, 0],
                                    frame_pm_temp[1, 0],
                                    frame_pm_temp[2, 0],
                                ),
                                (
                                    frame_pm_temp[0, 1],
                                    frame_pm_temp[1, 1],
                                    frame_pm_temp[2, 1],
                                ),
                            ]
                        ]
                    )
                    frames[world_frame_id_prefix][object_prefix]["y"].set_segments(
                        [
                            [
                                (
                                    frame_pm_temp[0, 0],
                                    frame_pm_temp[1, 0],
                                    frame_pm_temp[2, 0],
                                ),
                                (
                                    frame_pm_temp[0, 2],
                                    frame_pm_temp[1, 2],
                                    frame_pm_temp[2, 2],
                                ),
                            ]
                        ]
                    )
                    frames[world_frame_id_prefix][object_prefix]["z"].set_segments(
                        [
                            [
                                (
                                    frame_pm_temp[0, 0],
                                    frame_pm_temp[1, 0],
                                    frame_pm_temp[2, 0],
                                ),
                                (
                                    frame_pm_temp[0, 3],
                                    frame_pm_temp[1, 3],
                                    frame_pm_temp[2, 3],
                                ),
                            ]
                        ]
                    )
                    tag_position = np.column_stack(
                        (
                            frame_pm_temp[0, 0],
                            frame_pm_temp[1, 0],
                            frame_pm_temp[2, 0],
                        )
                    )

                    frames[world_frame_id_prefix][object_prefix][
                        "sensor_tag"
                    ]._offsets3d = (
                        tag_position[:, 0],
                        tag_position[:, 1],
                        tag_position[:, 2] - label_offset * sCount,
                    )
                    frames[world_frame_id_prefix][object_prefix][
                        "sensor_tag"
                    ].set_alpha(1.0)

                    frames[world_frame_id_prefix][object_prefix][
                        "frame_id"
                    ].set_position(
                        [
                            frame_pm_temp[0, 0],
                            frame_pm_temp[1, 0],
                        ]
                    )
                    frames[world_frame_id_prefix][object_prefix][
                        "frame_id"
                    ].set_3d_properties(
                        frame_pm_temp[2, 0] + axis_length + label_offset, zdir="x"
                    )
                    frames[world_frame_id_prefix][object_prefix]["frame_id"].set_alpha(
                        1.0
                    )
            sCount += 1

        else:
            raise RuntimeError("frame type error when visualizing the graph")

        return

    def transform_vec(homogeneous_matrix: np.ndarray, point: np.ndarray) -> np.ndarray:
        """
        Transform a point using a homogeneous matrix

        Args:
        ----------
            homogeneous_matrix: np.ndarray(np.float32, (4, 4)), homogeneous matrix
            point: np.ndarray(np.float32, (4, 1)) or np.ndarray(np.float32, (1, 4)),
                        homogeneous form of the point to be transformed

        Return:
        ----------
            transformed_point: np.ndarray(np.float32, (4, 1)), transformed point
        """
        if point.shape == (4, 1):
            pass
        elif point.shape == (1, 4):
            point = point.T
        else:
            raise RuntimeError(
                "Unexpected error when transforming the point, please check the input"
            )

        transformed_point = homogeneous_matrix @ point

        return transformed_point


if __name__ == "__main__":
    pass
