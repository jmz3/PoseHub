import sys
from PyQt5 import QtCore, QtWidgets
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import numpy as np
from visualize.viz_frames import axis_init, generate_frames  # your existing functions

class PoseGraphCanvas(FigureCanvas):
    def __init__(self, parent=None, axis_length=1.5):
        self.fig = Figure()
        self.ax = self.fig.add_subplot(111, projection="3d")
        self.ax = axis_init(self.ax, axis_length, "Pose")
        super(PoseGraphCanvas, self).__init__(self.fig)
        self.setParent(parent)
        
        # Initialize frames and primitives
        self.frames, self.frame_pm = generate_frames(
            ax=self.ax,
            sensors=["h1", "h2"],
            objects=["artool", "reference_1", "phantom"],
            axis_length=0.5,
        )
    
    def update_graph(self, pose_graph):
        # Update the visualization using the latest PoseGraph data.
        pose_graph.viz_graph_update(
            frames=self.frames,
            frame_primitive=self.frame_pm,
            world_frame_id="reference_1",
            frame_type=1,
        )
        self.draw()


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, pose_worker, parent=None):
        super(MainWindow, self).__init__(parent)
        self.pose_worker = pose_worker  # store the worker so we can stop it on exit
        self.setWindowTitle("PoseHub Visualization")
        
        # Create the central canvas widget
        self.canvas = PoseGraphCanvas(self)
        self.setCentralWidget(self.canvas)
        
        # Connect the worker signal to update the UI
        self.pose_worker.pose_updated.connect(self.on_pose_updated)
        
        # Set up a timer for refreshing the canvas
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.canvas.draw)
        self.timer.start(50)  # update every 50 ms (adjust as needed)
        
        # Create an Exit button
        self.exit_button = QtWidgets.QPushButton("Exit")
        self.exit_button.clicked.connect(self.close)
        
        # Create a toolbar and add a spacer widget then the exit button
        toolbar = QtWidgets.QToolBar()
        toolbar.setMovable(False)
        spacer = QtWidgets.QWidget()
        spacer.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        toolbar.addWidget(spacer)
        toolbar.addWidget(self.exit_button)
        self.addToolBar(QtCore.Qt.TopToolBarArea, toolbar)
    
    @QtCore.pyqtSlot(object)
    def on_pose_updated(self, updated_pose_graph):
        self.canvas.update_graph(updated_pose_graph)

    def closeEvent(self, event):
        # Cleanly stop the worker thread before closing the application
        self.pose_worker.stop()
        event.accept()


def main_gui(pose_worker):
    app = QtWidgets.QApplication(sys.argv)
    main_window = MainWindow(pose_worker)
    main_window.show()
    sys.exit(app.exec_())
