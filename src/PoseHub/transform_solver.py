from scipy.optimize import least_squares
import numpy as np
from utils import hat, se3_exp, se3_log


class TransformSolver:
    def __init__(self, nodes: list, edges: dict):
        """
        Initialize the TransformSolver class

        Members:
            graph: PoseGraph, the pose graph
        """

        self.nodes = nodes
        self.edges = edges

    def update(self, nodes: list, edges: dict):
        """
        Update the graph for the node and edge information

        Args:
        ----------
            nodes: list, all the nodes in the graph, each node is a sensor or an object
            edges: dict, described as an adjacency list, all the edges in the graph, each edge is a transformation between a sensor and an object
                        e.g. edges = { parent_id: {child_id: [transformation (4x4), isActive]},
                                        parent_id: {child_id: [transformation (4x4), isActive]},
                                        parent_id: {child_id: [transformation (4x4), isActive]},
                                        ...}
                             nodes = [[property, id_0], [property, id_1], [property, id_2], ...]
                        e.g. nodes = [[sensor, "sensor_id"], [obejct, "object_id"], [object, "object_id"]]


        """
        self.nodes = nodes.copy()
        self.edges = edges.copy()

        for node in self.nodes:
            self.edges[node] = {
                s: o for s, o in self.edges[node].items() if o[1]
            }  # filter out inactive edges, i.e. edges with isActive=False, o is the object value in the dict

        # self.optimize()
        # self._update_edges_from_optimization()

    def _update_edges_from_optimization(self):
        """
        Update each active edge transformation using the optimized sensor and object poses.
        For an edge from sensor s to object o, set
            Y_new = (optimized_sensor[s])^{-1} @ optimized_object[o]
        and update the inverse on the other direction.
        """
        # We assume that optimized_sensor and optimized_object were computed in optimize().
        if not hasattr(self, "optimized_sensor") or not hasattr(
            self, "optimized_object"
        ):
            print("Optimization has not been run, cannot update edges.")
            return

        # Update edges from every sensor to every object if measurement exists.
        for s in self.optimized_sensor.keys():
            for o in self.optimized_object.keys():
                # Only update if an edge existed originally (active edge)
                if s in self.edges and o in self.edges[s]:
                    # Compute predicted transformation from sensor s to object o.
                    T_s = self.optimized_sensor[s]
                    X_o = self.optimized_object[o]
                    Y_new = np.linalg.inv(T_s) @ X_o
                    # Update both directions.
                    self.edges[s][o] = [Y_new, True]
                    if o in self.edges and s in self.edges[o]:
                        self.edges[o][s] = [np.linalg.inv(Y_new), True]

    def solve(self, parent_id: str, child_id: str, method: str = "SET"):
        """
        Solve the transformation between the parent node and the child node.
        Returns a tuple (path, accumulative_transformation).
        """
        if method == "SET":
            result = self.SET(parent_id, child_id)
        elif method == "DFS":
            result = self.DFS(parent_id, child_id)
        elif method == "BFS":
            result = self.BFS(parent_id, child_id)
        else:
            raise ValueError("Invalid method; only SET, DFS, BFS are supported")
        return result

    def DFS(self, parent_id: str, child_id: str):
        """
        Depth-first search to find a path between nodes.
        Returns (path, accumulative_transform) if found.
        """
        visited = set()
        path = []

        def dfs_helper(node_id):
            visited.add(node_id)
            path.append(node_id)
            if node_id == child_id:
                return True
            for neighbor in self.edges.get(node_id, {}):
                if neighbor not in visited:
                    if dfs_helper(neighbor):
                        return True
            path.pop()
            return False

        if parent_id not in self.nodes or child_id not in self.nodes:
            return []
        if dfs_helper(parent_id):
            T_acc = self._accumulate_transform(path)
            return (path, T_acc)
        return []

    def BFS(self, parent_id: str, child_id: str) -> tuple:
        """
        Breadth-first search to find a path between nodes.
        Returns (path, accumulative_transform).
        """
        visited = set()
        queue = [(parent_id, [parent_id])]  # (node, path)
        visited.add(parent_id)

        while queue:
            node_id, curr_path = queue.pop(0)
            if node_id == child_id:
                T_acc = self._accumulate_transform(curr_path)
                return (curr_path, T_acc)
            for neighbor in self.edges.get(node_id, {}):
                if neighbor not in visited:
                    queue.append((neighbor, curr_path + [neighbor]))
                    visited.add(neighbor)
        return []

    def SET(self, parent_id: str, child_id: str) -> tuple:
        """
        A simple method (SET) to find a path.
        Returns (path, accumulative_transform) if found.
        """
        path = [parent_id]
        parent_visibles = set(self.edges[parent_id].keys())
        for sensor in self.edges.get(child_id, {}):
            extra_visibles = set(self.edges.get(sensor, {}).keys())
            shared_visibles = parent_visibles.intersection(extra_visibles)
            if shared_visibles:
                for visible_object in shared_visibles:
                    if (
                        self.edges[sensor][visible_object][1]
                        and self.edges[parent_id][visible_object][1]
                    ):
                        path.extend([visible_object, sensor, child_id])
                        T_acc = self._accumulate_transform(path)
                        return (path, T_acc)
        print("No path found using SET method.")
        return []

    def _accumulate_transform(self, path: list) -> np.ndarray:
        """
        Given a list of nodes forming a path, compute the accumulative transformation.
        For each consecutive pair (node_i, node_{i+1}), multiply the edge transformation.
        """
        T = np.eye(4)
        for i in range(len(path) - 1):
            if path[i] in self.edges and path[i + 1] in self.edges[path[i]]:
                T = T @ self.edges[path[i]][path[i + 1]][0]
            else:
                print(f"Edge missing between {path[i]} and {path[i+1]}.")
                return np.eye(4)
        return T

    def optimize(self):
        """
        Optimize the sensor and object poses based solely on the sensor–object measurements.
        We set up the cost function:
            J({X_i}, {T_s}) = sum_{s,i active} || Log( Y_{s,i}^{-1} (T_s^{-1} X_i) ) ||^2,
        fix the first sensor to identity (gauge fixation), and solve via least squares.
        Then, we marginalize sensor variables via the Schur complement.
        """
        # ----- Split nodes into sensors and objects -----
        sensors = [node for node in self.nodes if node.startswith("sensor_")]
        objects = [node for node in self.nodes if node.startswith("object_")]
        if not sensors or not objects:
            print("Not enough nodes for optimization.")
            return

        sensor_fixed = sensors[0]  # fix the first sensor
        sensor_init = {s: np.eye(4) for s in sensors}
        object_init = {o: np.eye(4) for o in objects}

        sensor_list = [s for s in sensors if s != sensor_fixed]
        num_sensor = len(sensor_list)
        num_object = len(objects)
        total_params = 6 * (num_sensor + num_object)
        sensor_idx = {s: i for i, s in enumerate(sensor_list)}
        object_idx = {o: i + num_sensor for i, o in enumerate(objects)}

        def residual_func(x):
            res = []
            sensor_updates = {}
            for s in sensors:
                if s == sensor_fixed:
                    sensor_updates[s] = np.eye(4)
                else:
                    idx_val = sensor_idx[s]
                    delta = x[6 * idx_val : 6 * idx_val + 6]
                    sensor_updates[s] = se3_exp(delta)
            object_updates = {}
            for o in objects:
                idx_val = object_idx[o]
                delta = x[6 * idx_val : 6 * idx_val + 6]
                object_updates[o] = se3_exp(delta)
            for s in sensors:
                for o in objects:
                    if s in self.edges and o in self.edges[s]:
                        edge = self.edges[s][o]
                        if not edge[1]:
                            continue
                        Y = edge[0]
                        T_s = sensor_init[s] @ sensor_updates[s]
                        X_o = object_init[o] @ object_updates[o]
                        pred = np.linalg.inv(T_s) @ X_o
                        err = se3_log(np.linalg.inv(Y) @ pred)
                        res.extend(err)
            return np.array(res)

        sol = least_squares(residual_func, np.zeros(total_params), verbose=2)
        optimized_sensor = {}
        for s in sensors:
            if s == sensor_fixed:
                optimized_sensor[s] = sensor_init[s]
            else:
                delta = sol.x[6 * sensor_idx[s] : 6 * sensor_idx[s] + 6]
                optimized_sensor[s] = sensor_init[s] @ se3_exp(delta)
        optimized_object = {}
        for o in objects:
            delta = sol.x[6 * object_idx[o] : 6 * object_idx[o] + 6]
            optimized_object[o] = object_init[o] @ se3_exp(delta)

        # ----- Marginalization (optional) via Schur complement -----
        J = sol.jac
        J_sensor = (
            J[:, : 6 * num_sensor] if num_sensor > 0 else np.empty((J.shape[0], 0))
        )
        J_object = J[:, 6 * num_sensor :]
        r = sol.fun
        H_sensor = J_sensor.T @ J_sensor
        H_object = J_object.T @ J_object
        H_cross = J_sensor.T @ J_object
        try:
            H_sensor_inv = np.linalg.inv(H_sensor)
        except np.linalg.LinAlgError:
            H_sensor_inv = np.linalg.pinv(H_sensor)
        H_sc = H_object - H_cross.T @ H_sensor_inv @ H_cross
        g_sensor = J_sensor.T @ r
        g_object = J_object.T @ r
        g_sc = g_object - H_cross.T @ H_sensor_inv @ g_sensor
        try:
            delta_eta = -np.linalg.inv(H_sc) @ g_sc
        except np.linalg.LinAlgError:
            delta_eta = -np.linalg.pinv(H_sc) @ g_sc
        for o in objects:
            idx_val = object_idx[o]
            delta = delta_eta[6 * idx_val : 6 * idx_val + 6]
            optimized_object[o] = optimized_object[o] @ se3_exp(delta)

        self.optimized_sensor = optimized_sensor
        self.optimized_object = optimized_object

        print("Optimized Object Poses (after marginalization):")
        for o, T in optimized_object.items():
            print(o, "\n", T)
