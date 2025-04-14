from PyQt5 import QtCore
from pose_graph import PoseGraph


class PoseGraphManager(QtCore.QThread):
    # Signal that emits the updated PoseGraph.
    pose_graph_updated = QtCore.pyqtSignal(object)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.pose_graph = PoseGraph()  # Use the optimized PoseGraph.
        self._queue = []  # Queue to store incoming updates.
        self._mutex = QtCore.QMutex()
        self.running = True

    def update_pose(self, sensor_id, poseinfo):
        """
        Thread-safe method to queue pose updates.

        Args:
            sensor_id (str): Identifier of the sensor.
            poseinfo (dict): Dictionary of poses received.
        """
        self._mutex.lock()
        try:
            # Append the update as a tuple.
            self._queue.append((sensor_id, poseinfo))
        finally:
            self._mutex.unlock()

    def run(self):
        """
        Main loop running on a separate thread.
        It continuously checks the queue for new pose updates,
        processes them via the optimized PoseGraph, and emits a signal.
        """
        print("PoseGraphManager thread started.")
        while self.running:
            update = None
            self._mutex.lock()
            try:
                # print(f"Queue size: {len(self._queue)}")
                if self._queue:
                    update = self._queue.pop(0)
                    if len(self._queue) > 10:
                        self._queue.clear()  # Clear the queue if it gets too large.

            finally:
                self._mutex.unlock()
            if update:
                sensor_id, poseinfo = update
                # Update the optimized pose graph.
                self.pose_graph.update_graph(sensor_id, poseinfo)
                # Emit the updated pose graph.
                self.pose_graph_updated.emit(self.pose_graph)
            else:
                self.msleep(10)  # Sleep briefly if no updates are available.

    def stop(self):
        """
        Gracefully stop the thread.
        """
        self.running = False
        self.quit()
        self.wait()
