import numpy as np
from PyQt5 import QtCore
from comm.ZMQManager import ZMQManager, ZMQManagerNoParallel


class PoseGraphThread(QtCore.QThread):
    # This thread no longer emits pose_updated itself.
    pose_updated = QtCore.pyqtSignal(object)

    def __init__(self, args, manager, parent=None):
        super(PoseGraphThread, self).__init__(parent)
        self.args = args
        self.manager = manager  # Shared central manager
        self.running = True
        self.tracking_active = False  # New flag: initially tracking is off.
        self.sensor_name = getattr(self.args, "sensor_name", "h1")
        self.zmq_manager = ZMQManagerNoParallel(
            sub_ip=self.args.sub_ip,
            sub_port=self.args.sub_port,
            pub_port=self.args.pub_port,
            sub_topic=self.args.sub_topic,
            pub_topic=self.args.pub_topic,
            sensor_name=self.sensor_name,
        )

    def run(self):
        self.zmq_manager.initialize()
        print("Receiving poses...")
        while self.running:
            if self.tracking_active:
                # Receive pose info via ZMQ.
                try:
                    poseinfo = self.zmq_manager.receive_poses()
                except Exception as e:
                    print(f"Exception in receive_poses: {e}")
                    break
                if poseinfo:
                    # Forward the update to the central PoseGraphManager.
                    self.manager.update_pose(self.sensor_name, poseinfo)

                    for topic in self.args.pub_topic:
                        pose = self.manager.get_pose_graph().get_transform(
                            self.sensor_name, topic, solver_method="BFS"
                        )
                        if pose is not None and np.linalg.norm(pose[:3, 3]) > 1e-5:
                            self.zmq_manager.send_poses(topic, pose)
            self.msleep(10)  # Adjust update rate as needed.

    def start_tracking(self):
        self.tracking_active = True

    def stop(self):
        self.running = False
        self.msleep(100)
        self.zmq_manager.terminate()
        self.quit()
        self.wait()
