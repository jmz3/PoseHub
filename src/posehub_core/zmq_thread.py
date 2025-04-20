import os
import json
import time
import numpy as np
from PyQt5 import QtCore
from comm.ZMQManager import ZMQManager, ZMQManagerNoParallel
from posegraph_manager import PoseGraphManager
from utils import ZMQConfig, serialize_poseinfo
from numpy import random


class ZMQThread(QtCore.QThread):
    # This thread no longer emits pose_updated itself.
    pose_updated = QtCore.pyqtSignal(object)

    def __init__(self, args: ZMQConfig, pg_manager: PoseGraphManager, parent=None):
        super(ZMQThread, self).__init__(parent)
        self.args = args
        self.pg_manager = pg_manager  # Shared central manager
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
        # Create a dictionary to cache poseinfo with timestamp keys.
        self.cached_poses = {}

    def run(self):
        self.zmq_manager.initialize()
        print("Receiving poses...")
        while self.running:
            if self.tracking_active:
                try:
                    poseinfo = self.zmq_manager.receive_poses()
                except Exception as e:
                    print(f"Exception in receive_poses: {e}")
                    break
                if poseinfo:
                    # Append poseinfo with current timestamp key into the cache.
                    timestamp = str(time.time())
                    self.cached_poses[timestamp] = serialize_poseinfo(poseinfo)

                    # Use the new optimized manager.
                    self.pg_manager.update_pose(self.sensor_name, poseinfo)

                    for topic in self.args.pub_topic:
                        pose = self.pg_manager.pose_graph.get_transform(
                            self.sensor_name, topic, solver_method="BFS"
                        )
                        uncertainty = random.uniform(-1, 1, 3)
                        uncertainty = uncertainty * 0.01 + np.array([0.02, 0.02, 0.07])
                        # Normalize the uncertainty
                        uncertainty /= np.linalg.norm(uncertainty)
                        uncertainty *= 0.03

                        if pose is not None and np.linalg.norm(pose[:3, 3]) > 1e-5:
                            self.zmq_manager.send_poses(
                                topic,
                                pose,
                                isActive=True,
                                uncertainty=uncertainty.tolist(),
                            )
                        else:
                            self.zmq_manager.send_poses(
                                topic,
                                np.identity(4),
                                isActive=False,
                                uncertainty=[0.02, 0.02, 0.1],
                            )
            self.msleep(10)  # Adjust update rate as needed.

    def start_tracking(self):
        self.tracking_active = True

    def stop(self):
        self.running = False
        self.msleep(100)
        self.zmq_manager.terminate()
        # Dump the cached poses to disk before quitting.
        self.dump_cached_poses()
        self.quit()
        self.wait()



    def dump_cached_poses(self):
        """
        Dump the cached poses dictionary to a JSON file in a folder named after sensor_name.
        """
        folder_name = "data/" + self.sensor_name
        if not os.path.exists(folder_name):
            os.makedirs(folder_name)
        # Use current time as part of the filename to avoid overwriting previous dumps.
        filename = os.path.join(
            folder_name, f"pose_record_{time.strftime('%Y%m%d_%H%M%S')}.json"
        )
        try:
            with open(filename, "w") as f:
                json.dump(self.cached_poses, f, indent=4)
            print(f"Cached poses dumped to {filename}")
        except Exception as e:
            print(f"Error dumping cached poses: {e}")
