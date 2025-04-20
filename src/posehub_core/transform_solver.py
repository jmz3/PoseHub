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
            return path
        return None

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
                return curr_path
            for neighbor in self.edges.get(node_id, {}):
                if neighbor not in visited:
                    queue.append((neighbor, curr_path + [neighbor]))
                    visited.add(neighbor)
        return None

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

                        return path
        print("No path found using SET method.")
        return None
