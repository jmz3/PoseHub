import numpy as np
from typing import Dict, Optional, Tuple
import time
from enum import Enum
from copy import deepcopy
from scipy.optimize import least_squares

from utils import hat, se3_exp, se3_log
from path_find import PathFind
from data_recorder import DataRecorder


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
                         e.g. nodes = [[sensor, "sensor_id"], [obejct, "object_id"], [object, "object_id"]]
            edges: dict, all the edges in the graph, each edge is a transformation between a sensor and an object
                         e.g. edges = {parent_id: {child_id: [transformation, isActive]}, ...}
                              edges[parent_id] = {child_id, [transformation, isActive]}
            sensor_id: list[str] of all the sensor ids
            object_id: list[str] of all the object ids

        """
        self.nodes = []
        self.edges = dict([])
        self.edges_opt = dict([])
        self.sensor_id = []
        self.object_id = []
        self.path_finder = PathFind()
        self.data_recorder = DataRecorder()

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
        # TODO: need to consider the case when the object is no longer visible for the sensor
        # Is it necessary to remove the edge between the sensor and the object?
        # Is a flag enough to indicate if the transformation is active?
        self.global_optimize()
        self.complete_scene(path_finding_method="BFS")
        self.data_recorder.update(self.edges, self.edges_opt)

    def global_optimize(self):
        """
        Global optimization that uses all active edges across the graph.
        It optimizes over the sensor and object poses such that for every active edge (s, o)
            Y_{s,o} ≈ T_s⁻¹ X_o,
        using the residual r = se3_log( Y⁻¹ (T_s⁻¹ X_o) ).
        The first sensor in self.sensor_id is fixed to identity.
        This function uses the previously optimized T_s and X_o (stored in self.sensor_est and self.object_est)
        as the initial guess. The optimized relative transformation for each active edge is stored in self.edges_opt.
        """
        # Check that we have sensors and objects.
        if len(self.sensor_id) == 0 or len(self.object_id) == 0:
            print("Not enough nodes to optimize.")
            return

        fixed_sensor = self.sensor_id[0]
        sensor_nonfixed = [s for s in self.sensor_id if s != fixed_sensor]
        n_sensor = len(sensor_nonfixed)
        n_object = len(self.object_id)
        total_vars = 6 * (n_sensor + n_object)

        # Create mappings: sensor and object indices in the optimization variable.
        sensor_idx = {s: i for i, s in enumerate(sensor_nonfixed)}
        object_idx = {o: i + n_sensor for i, o in enumerate(self.object_id)}

        # Use previous optimization results if available, otherwise identity.
        if hasattr(self, "sensor_est"):
            sensor_init = self.sensor_est.copy()
        else:
            sensor_init = {s: np.eye(4) for s in self.sensor_id}

        if hasattr(self, "object_est"):
            object_init = self.object_est.copy()
        else:
            object_init = {o: np.eye(4) for o in self.object_id}

        # Gather active measurements: (sensor, object, measured Y)
        measurements = []
        for s in self.sensor_id:
            for o, (T_meas, isActive) in self.edges.get(s, {}).items():
                if not isActive:
                    continue
                # Only consider sensor -> object edges.
                if s.startswith("sensor_") and o.startswith("object_"):
                    measurements.append((s, o, T_meas))

        if len(measurements) == 0:
            print("No active measurements to optimize.")
            return

        # Residual function: builds error vector from all active measurements.
        def residual_func(x):
            res = []
            # Sensor updates: fixed sensor remains unchanged.
            sensor_updates = {}
            for s in self.sensor_id:
                if s == fixed_sensor:
                    sensor_updates[s] = np.eye(4)
                else:
                    idx = sensor_idx[s]
                    delta = x[6 * idx : 6 * idx + 6]
                    sensor_updates[s] = se3_exp(delta)
            # Object updates:
            object_updates = {}
            for o in self.object_id:
                idx = object_idx[o]
                delta = x[6 * idx : 6 * idx + 6]
                object_updates[o] = se3_exp(delta)
            # For each active measurement, compute residual.
            for s, o, Y in measurements:
                T_s = sensor_init[s] @ sensor_updates[s]
                X_o = object_init[o] @ object_updates[o]
                pred = np.linalg.inv(T_s) @ X_o
                err = se3_log(np.linalg.inv(Y) @ pred)
                res.extend(err)
            return np.array(res)

        # Use previous result as the initial guess for x.
        # We initialize x as zero if previous result exists (i.e. current sensor_init and object_init are used).
        x0 = np.zeros(total_vars)

        # Solve for the updates
        # print the optimization time comsumption
        # print("Optimization started...")
        curr_time = time.time()
        sol = least_squares(residual_func, x0, verbose=0, ftol=1e-6, xtol=1e-6)
        print(f"Optimization takes {time.time() - curr_time:.3f} seconds. ")
        # Extract optimized sensor poses.
        sensor_opt = {}
        for s in self.sensor_id:
            if s == fixed_sensor:
                sensor_opt[s] = sensor_init[s]
            else:
                idx = sensor_idx[s]
                delta = sol.x[6 * idx : 6 * idx + 6]
                sensor_opt[s] = sensor_init[s] @ se3_exp(delta)

        # Extract optimized object poses.
        object_opt = {}
        for o in self.object_id:
            idx = object_idx[o]
            delta = sol.x[6 * idx : 6 * idx + 6]
            object_opt[o] = object_init[o] @ se3_exp(delta)

        # Store the current optimized poses for use as initial guess in the next call.
        self.sensor_est = sensor_opt.copy()
        self.object_est = object_opt.copy()

        # Update edges_opt: for each sensor-object edge (only active ones are re-optimized)
        self.edges_opt = deepcopy(self.edges)
        for s in self.sensor_id:
            for o, (T_meas, isActive) in self.edges[s].items():
                if s.startswith("sensor_") and o.startswith("object_"):
                    if isActive:
                        T_s_opt = sensor_opt[s]
                        X_o_opt = object_opt[o]
                        T_edge_opt = np.linalg.inv(T_s_opt) @ X_o_opt
                        self.edges_opt[s][o] = [T_edge_opt, True]
                        self.edges_opt[o][s] = [np.linalg.inv(T_edge_opt), True]
                    else:
                        self.edges_opt[s][o] = [T_meas, False]
                        self.edges_opt[o][s] = [np.linalg.inv(T_meas), False]

        print("Global optimization complete. Optimized edges stored in edges_opt.")

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
        # check parent and child type to be sensor or object
        # Four combinations are possible:
        # 1. sensor -> sensor; 2. sensor -> object; 3. object -> sensor; 4. object -> object
        # if parent_id not in self.nodes or child_id not in self.nodes:
        #     print(
        #         f"Parent node {parent_id} and Child node {child_id} is not in the graph"
        #     )
        #     return None
        # if parent_id == child_id:
        #     print("Parent and child node are the same")
        #     return None

        if self.edges_opt == {}:
            # print("The graph is not optimized yet, please run global_optimize() first")
            return None

        if parent_id == child_id:
            return np.identity(4)

        # print(self.edges_opt)

        if ("sensor_" + parent_id) in self.sensor_id:
            if ("sensor_" + child_id) in self.sensor_id:
                # sensor -> sensor
                return self.edges_opt["sensor_" + parent_id]["sensor_" + child_id][0]
            elif ("object_" + child_id) in self.object_id:
                # sensor -> object
                return self.edges_opt["sensor_" + parent_id]["object_" + child_id][0]
            else:
                RuntimeWarning(
                    "Child node is not a sensor or object, please check the input"
                )
                return None
        elif ("object_" + parent_id) in self.object_id:
            if ("sensor_" + child_id) in self.sensor_id:
                # object -> sensor
                return self.edges_opt["object_" + parent_id]["sensor_" + child_id][0]
            elif ("object_" + child_id) in self.object_id:
                # object -> object
                # print("parent_id: ", parent_id)
                # print("child_id: ", child_id)
                # print("edges_opt: ", self.edges_opt["object_" + parent_id])
                return self.edges_opt["object_" + parent_id]["object_" + child_id][0]
            else:
                RuntimeWarning(
                    "Child node is not a sensor or object, please check the input"
                )
                return None

        else:
            print(
                f"Parent node {parent_id} and Child node {child_id} is not in the graph"
            )
            RuntimeError(
                "Parent node is not a sensor or object, please check the input"
            )
            return None

    pass

    def complete_scene(self, path_finding_method: str = "BFS"):
        """
        Complete the scene by adding the missing edges between the sensors and the objects
        The function will add the edges between the sensors and the objects that are not connected or isActive is False
        """
        for object_parent in self.object_id:
            # Add edges inter objects using previously optimized global object pose X_o
            for object_child in self.object_id:
                if object_parent == object_child:
                    continue
                if object_child not in self.edges_opt[object_parent]:
                    self.edges_opt[object_parent][object_child] = [
                        np.linalg.inv(self.object_est[object_parent])
                        @ self.object_est[object_child],
                        True,
                    ]  # TODO: Is blindly setting the edge to True correct?
                    # Should consider whether this is a valid connection when the object is not visible to any sensor.
                else:
                    # The edge is already active
                    continue

        # Add edges inter sensors using previously optimized global sensor pose T_s
        for sensor_parent in self.sensor_id:
            # Add edges inter sensors using previously optimized global sensor pose T_s
            # self.edges_opt[sensor_parent] = {}
            for sensor_child in self.sensor_id:
                if sensor_parent == sensor_child:
                    continue
                if sensor_child not in self.edges_opt[sensor_parent]:
                    self.edges_opt[sensor_parent][sensor_child] = [
                        np.linalg.inv(self.sensor_est[sensor_parent])
                        @ self.sensor_est[sensor_child],
                        True,
                    ]
                else:
                    # The edge is already active
                    continue

        # Complete the edges between the sensors and the objects if the edge is not active
        for sensor in self.sensor_id:
            for object_child in self.object_id:
                if self.edges_opt[sensor][object_child][1]:
                    continue
                # The edge is not active, call the path solver to find the path
                path, transform = self.path_finder.solve(
                    sensor,
                    object_child,
                    nodes=self.nodes,
                    edges=self.edges_opt,
                    method=path_finding_method,
                )

                if len(path) == 0:
                    # No path found, continue to the next edge
                    continue
                # The path is found, add the edge to the graph
                self.edges_opt[sensor][object_child] = [transform, True]
                self.edges_opt[object_child][sensor] = [np.linalg.inv(transform), True]

        print("Scene completed. Missing edges added.")

    def save_to_json(self, filename: str):
        """
        Save the graph to a json file
        """
        self.data_recorder.save(filename)


if __name__ == "__main__":
    pass
