import numpy as np


class PoseGraph:
    """
    PoseGraph class stores the transformation between all the sensors and the objects in the scene
    the graph is a directled represented as a list of nodes and a list of edges

    """

    def __init__(self):
        """
        Initialize the PoseGraph class.

        Members:
            nodes: list of all the nodes in the graph, each node is a sensor or an object
            edges: list of all the edges in the graph, each edge is a transformation between two nodes
            sensor_id: list[str] of all the sensor ids
            object_id: list[str] of all the object ids
        """
        self.nodes = []
        self.edges = []
        self.sensor_id = None
        self.object_id = None

    def add_node(self, node_id):
        self.nodes.append(node_id)

    def add_edge(self, edge_id):
        self.edges.append(edge_id)

    def add_sensor(self, sensor_id):
        self.sensor_id.append(sensor_id)
        self.add_node(sensor_id)

    def add_object(self, object_id):
        self.add_node(object_id)


class Pose:
    def __init__(self, sensor_id, timestamp, pose):
        self.sensor_id = sensor_id
        self.timestamp = timestamp
        self.pose = pose


if __name__ == "__main__":
    sensor_id = ["cam0", "cam1", "imu0", "imu1"]

    pose_id = ["cam1", "cam2", "cam3", "cam4", "cam5"]

    for pose in pose_id:
        if pose in sensor_id:
            print("yes")
