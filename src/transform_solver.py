from pose_graph import PoseGraph


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
        """
        self.nodes = nodes
        self.edges = edges

    def solve(self, parent_id: str, child_id: str, method: str = "SET"):
        """
        Solve the transformation between the parent node and the child node
        """
        if method == "SET":
            return self.SET(parent_id, child_id)
        elif method == "DFS":
            return self.DFS(parent_id, child_id)
        elif method == "BFS":
            return self.BFS(parent_id, child_id)
        else:
            raise ValueError("Invalid method, \nOnly SET, DFS, BFS are supported")

    def DFS(self, parent_id: str, child_id: str):
        """
        Depth-first search algorithm to find a path between the parent node and the child node
        """
        visited = set()
        path = []

        def dfs_helper(node_id):
            visited.add(node_id)
            path.append(node_id)

            if node_id == child_id:
                return True

            for neighbor_node in self.edges[node_id]:
                if (
                    neighbor_node[0] not in visited
                ):  # neighbor_node[0] is the neighbor node id
                    if dfs_helper(neighbor_node[0]):
                        return True

            path.pop()
            return False

        if parent_id not in self.nodes or child_id not in self.nodes:
            return []

        if dfs_helper(parent_id):
            return path

        return []

    def BFS(self, parent_id: str, child_id: str):
        """
        Breadth-first search algorithm to find a path between the parent node and the child node
        """
        visited = set()
        path = []

        queue = [parent_id]
        visited.add(parent_id)

        while queue:
            node_id = queue.pop(0)
            path.append(node_id)

            if node_id == child_id:
                return path

            for neighbor_node in self.edges[node_id]:
                if neighbor_node[0] not in visited:
                    queue.append(neighbor_node[0])
                    visited.add(neighbor_node[0])

        return []

    def SET(self, parent_id: str, child_id: str):
        """
        SET algorithm to find a path between the parent node and the child node
        """
        visited = set()
        path = []

        # TODO: implement the SET algorithm to find a path between the parent node and the child node
        pass
