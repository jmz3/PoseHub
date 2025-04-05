from PyQt5 import QtCore

from pose_graph import PoseGraph

# from pose_graph_optimize import PoseGraph


class PoseGraphManager(QtCore.QObject):
    # Signal that emits the updated PoseGraph.
    pose_graph_updated = QtCore.pyqtSignal(object)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.pose_graph = PoseGraph()

    @QtCore.pyqtSlot(str, dict)
    def update_pose(self, sensor_id, poseinfo):
        """
        Update the central PoseGraph with new data coming from a sensor.
        sensor_id: identifier string for the sensor.
        poseinfo: dictionary of poses received.
        """
        # Update the pose graph for the given sensor.
        self.pose_graph.update_graph(sensor_id, poseinfo)
        # Emit the updated PoseGraph.
        self.pose_graph_updated.emit(self.pose_graph)

    def get_pose_graph(self):
        return self.pose_graph
