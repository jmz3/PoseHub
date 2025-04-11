from scipy.optimize import least_squares
import numpy as np
from utils import hat, se3_exp, se3_log


class PathFind:
    def __init__(self, nodes: list = None, edges: dict = None):
        """
        Initialize the PathFind class that finds viable paths between nodes in a graph.

        Param:
        -------
        nodes: list of nodes in the graph.
        edges: dictionary representing the edges of the graph.
        """

        self.nodes = nodes
        self.edges = edges

    def solve(
        self,
        parent_id: str,
        child_id: str,
        nodes: list,
        edges: dict,
        method: str = "SET",
    ):
        """
        Solve the transformation between the parent node and the child node.
        Returns a tuple (path, accumulative_transformation).
        """
        self.nodes = nodes
        self.edges = edges

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
