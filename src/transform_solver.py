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

        for node in self.nodes:
            self.edges[node] = {
                s: o for s, o in self.edges[node].items() if o[1]
            }  # filter out inactive edges, i.e. edges with isActive=False, o is the object value in the dict

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

        # TODO: implement the DFS algorithm to find a path between the parent node and the child node,
        # The current implementation is not correct
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

    def BFS(self, parent_id: str, child_id: str) -> list:
        """
        Breadth-first search algorithm to find a path between the parent node and the child node
        """
        visited = set()
        path = []

        queue = [(parent_id, [parent_id])]  # (node_id, path)
        visited.add(parent_id)

        while queue:
            node_id, path = queue.pop(0)
            # print("Node id", node_id)
            # print("Path", path)

            if node_id == child_id:
                return path

            if node_id not in visited:
                visited.add(node_id)
                path.append(node_id)

            # print("BFS", self.edges[node_id])
            for neighbor_node in self.edges[node_id].keys():
                # print("The child node is", neighbor_node)
                # print("BFS", self.edges[node_id].keys())

                if neighbor_node not in visited:
                    queue.append((neighbor_node, path + [neighbor_node]))
                    visited.add(neighbor_node)

        return []

    def SET(self, parent_id: str, child_id: str) -> list:
        """
        SET algorithm to find a path between the parent node and the child node
        """

        path = [parent_id]

        parent_visibles = set(self.edges[parent_id].keys())

        # find all the sensors that can see the child node
        for sensor in self.edges[child_id].keys():
            extra_visibles = set(
                self.edges[sensor].keys()
            )  # find all objects that can be seen by the sensor

            # find the intersection of the parent_visibles and the extra_visibles
            shared_visibles = parent_visibles.intersection(
                extra_visibles
            )  # if not empty

            if shared_visibles is not None:
                for visible_object in shared_visibles:
                    if (
                        self.edges[sensor][visible_object][1]
                        and self.edges[parent_id][visible_object][1]
                    ):
                        path.append(visible_object)
                        path.append(sensor)
                        path.append(child_id)
                        return path

        print("No path found, please try other methods")
        return []
