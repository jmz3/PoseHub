import time
import numpy as np
from PyQt5 import QtCore
from pose_graph import PoseGraph
from comm.ZMQManager import ZMQManager  # Adjust the import as needed


class PoseGraphThread(QtCore.QThread):
    # Signal that emits the updated PoseGraph (or just the necessary data)
    pose_updated = QtCore.pyqtSignal(object)

    def __init__(self, args, parent=None):
        super(PoseGraphThread, self).__init__(parent)
        self.args = args
        self.running = True
        self.pose_graph = PoseGraph()

        # Initialize your ZMQManagers for different sensors
        self.zmq_manager = ZMQManager(
            sub_ip=args.sub_ip_1,
            sub_port="5588",
            pub_port="5589",
            sub_topic=args.sub_topic,
            pub_topic=args.pub_topic,
            sensor_name="h1",
        )
        self.zmq_manager.initialize()

    def run(self):
        while self.running:
            # Receive poses and update the graph
            poseinfo = self.zmq_manager.receive_poses()
            if poseinfo:
                self.pose_graph.update_graph("h1", poseinfo)

                # Optionally, send out poses after processing:
                for topic in self.args.pub_topic:
                    pose = self.pose_graph.get_transform(
                        "h1", topic, solver_method="BFS"
                    )
                    if pose is not None and np.linalg.norm(pose[:3, 3]) > 1e-5:
                        self.zmq_manager.send_poses(topic, pose)

            # Emit updated pose graph to update visualization
            self.pose_updated.emit(self.pose_graph)
            time.sleep(0.01)  # adjust sleep time to control update rate

    def stop(self):
        self.running = False
        self.zmq_manager.terminate()
        self.quit()
        self.wait()
