import sys
import numpy as np
from PyQt5 import QtCore, QtWidgets
import pyqtgraph.opengl as gl
from scipy.spatial.transform import Rotation as Rot
from utils import load_instance_from_obj
from posegraph_manager import PoseGraphManager
from zmq_thread import ZMQThread
from time import time


class NetworkConfig:
    def __init__(
        self, ip="127.0.0.1", sub_port="5555", pub_port="5556", sensor_name="Camera"
    ):
        self.IP = ip
        self.SUB_PORT = sub_port
        self.PUB_PORT = pub_port
        self.SENSOR_NAME = sensor_name


class PoseGraphGUI(QtWidgets.QMainWindow):
    def __init__(
        self, tool_names, config: NetworkConfig, ref_frame="Hololens", parent=None
    ):
        super().__init__(parent)
        self.setWindowTitle("PoseHub Visualization")
        self.resize(1200, 700)

        self.tool_names = tool_names
        self.config = config
        self.ref_frame = self.tool_names[0] if self.tool_names else ""
        self.pose_graph_manager = PoseGraphManager()
        self.obj_file_path = "/home/jeremy/Research/PoseHub/ExpData/tinker.obj"
        self.sensor_obj_generated = False
        self.sensor_names = []
        self.worker_threads = []
        self.tool_labels = {}
        self.sensor_labels = {}

        self.initUI()
        self.pose_graph_manager.pose_graph_updated.connect(self.on_pose_updated)
        self.last_time = time()
        self.current_time = time()

    def initUI(self):
        main_widget = QtWidgets.QWidget()
        self.setCentralWidget(main_widget)
        # Horizontal layout: left = GL view, right = configuration panel.
        main_layout = QtWidgets.QHBoxLayout(main_widget)

        # Left side: OpenGL view.
        self.gl_view = gl.GLViewWidget()
        self.gl_view.opts["distance"] = 100
        main_layout.addWidget(self.gl_view, 4)
        grid = gl.GLGridItem()
        grid.scale(1, 1, 1)
        self.gl_view.addItem(grid)

        # Right side: Configuration panel.
        config_widget = QtWidgets.QWidget()
        config_layout = QtWidgets.QFormLayout(config_widget)

        self.ip_edit = QtWidgets.QLineEdit(self.config.IP)
        self.sub_port_edit = QtWidgets.QLineEdit(self.config.SUB_PORT)
        self.pub_port_edit = QtWidgets.QLineEdit(self.config.PUB_PORT)
        self.sensor_name_edit = QtWidgets.QLineEdit(self.config.SENSOR_NAME)

        config_layout.addRow("Subscriber IP:", self.ip_edit)
        config_layout.addRow("Subscriber Port:", self.sub_port_edit)
        config_layout.addRow("Publisher Port:", self.pub_port_edit)
        config_layout.addRow("Sensor Name:", self.sensor_name_edit)

        self.add_conn_button = QtWidgets.QPushButton("Add Connection")
        self.add_conn_button.clicked.connect(self.connect_worker)
        self.start_tracking_button = QtWidgets.QPushButton("Start Tracking")
        self.start_tracking_button.clicked.connect(self.start_tracking)

        config_layout.addRow(self.add_conn_button)
        config_layout.addRow(self.start_tracking_button)

        # Add a drop-down for selecting the reference frame.
        self.ref_combo = QtWidgets.QComboBox()
        # Initially populate with tool names.
        self.ref_combo.addItems(self.tool_names)
        self.ref_combo.currentIndexChanged.connect(self.on_ref_changed)
        config_layout.addRow("Reference Frame:", self.ref_combo)

        main_layout.addWidget(config_widget, 1)

        self.load_tool_instances(self.obj_file_path)

    def on_ref_changed(self, index):
        # Update the reference frame based on the drop-down selection.
        self.ref_frame = self.ref_combo.currentText()
        print(f"Reference frame changed to: {self.ref_frame}")

    def load_tool_instances(self, obj_path):
        self.tool_instance_data = []
        for tool in self.tool_names:
            base_translation = np.random.uniform(-5, 5, size=3).astype(np.float32)
            base_rotation = 0
            inst_items = load_instance_from_obj(
                obj_path, base_translation, base_rotation
            )
            for frame_item in inst_items:
                self.gl_view.addItem(frame_item)
            self.tool_instance_data.append(
                {
                    "items": inst_items,
                    "translation": base_translation,
                    "rotation": base_rotation,
                }
            )
            print(f"Loaded tool instance {tool} at {base_translation}.")
            frame_name_textitem = gl.GLTextItem(
                pos=base_translation + np.array([0, 0, 1]), text=tool
            )
            self.gl_view.addItem(frame_name_textitem)
            self.tool_labels[tool] = frame_name_textitem

    def connect_worker(self):
        ip = self.ip_edit.text()
        sub_port = self.sub_port_edit.text()
        pub_port = self.pub_port_edit.text()
        sensor_name = self.sensor_name_edit.text()

        class Args:
            pass

        args = Args()
        args.sub_ip = ip
        args.sub_topic = self.tool_names  # using tool names as topics
        args.pub_topic = self.tool_names
        args.sub_port = sub_port
        args.pub_port = pub_port
        args.sensor_name = sensor_name

        new_thread = ZMQThread(args, self.pose_graph_manager)
        self.worker_threads.append(new_thread)
        print(f"Added connection for sensor: {sensor_name}")

        # If this sensor is new, create its visual instance and add it to the reference drop-down.
        if sensor_name not in self.sensor_names:
            self.sensor_names.append(sensor_name)
            frame_items = load_instance_from_obj(self.obj_file_path)
            for item in frame_items:
                self.gl_view.addItem(item)
            if not hasattr(self, "sensor_instance_data"):
                self.sensor_instance_data = {}
            self.sensor_instance_data[sensor_name] = {
                "items": frame_items,
                "translation": np.array([0, 0, 0], dtype=np.float32),
                "rotation": 0,
            }
            frame_name_textitem = gl.GLTextItem(
                pos=(0, 0, 0), text=sensor_name, color=(255, 255, 255, 255)
            )
            self.gl_view.addItem(frame_name_textitem)
            self.sensor_labels[sensor_name] = frame_name_textitem

            # Also add the new sensor to the reference frame drop-down.
            self.ref_combo.addItem(sensor_name)

    def start_tracking(self):
        for thread in self.worker_threads:
            thread.start()
            thread.start_tracking()
        print("Tracking started on all connections.")

    def on_pose_updated(self, pose_graph):
        # Update tool instances and their GLTextItems using the selected reference frame.
        for idx, tool in enumerate(self.tool_names):
            transform = pose_graph.get_transform(
                self.ref_frame, tool, solver_method="BFS"
            )
            if transform is None:
                continue
            t = transform[:3, 3]
            R_mat = transform[:3, :3]
            rot = Rot.from_matrix(R_mat)
            rotvec = rot.as_rotvec()
            angle_rad = np.linalg.norm(rotvec)
            angle_deg = np.degrees(angle_rad)
            axis = (
                (rotvec / angle_rad)
                if angle_rad > 1e-6
                else np.array([0, 1, 0], dtype=np.float32)
            )
            for item in self.tool_instance_data[idx]["items"]:
                item.resetTransform()
                if angle_deg > 1e-3:
                    item.rotate(angle_deg, axis[0], axis[1], axis[2])
                item.translate(t[0], t[1], t[2])

            if tool in self.tool_labels:
                new_pos = t + np.array([0, -0.1, 0])
                self.tool_labels[tool].setData(pos=new_pos, text=tool)
                # print(f"Updated tool {tool} at {t}.")

        # Update sensor instances and their GLTextItems.
        for sensor in self.sensor_names:
            transform = pose_graph.get_transform(
                self.ref_frame, sensor, solver_method="BFS"
            )
            if transform is None:
                continue
            t = transform[:3, 3]
            R_mat = transform[:3, :3]
            rot = Rot.from_matrix(R_mat)
            rotvec = rot.as_rotvec()
            angle_rad = np.linalg.norm(rotvec)
            angle_deg = np.degrees(angle_rad)
            axis = (
                (rotvec / angle_rad)
                if angle_rad > 1e-6
                else np.array([0, 1, 0], dtype=np.float32)
            )
            sensor_data = self.sensor_instance_data.get(sensor, None)
            if sensor_data is not None:
                for sensor_item in sensor_data["items"]:
                    sensor_item.resetTransform()
                    sensor_item.translate(t[0], t[1], t[2])
                    if angle_deg > 1e-3:
                        sensor_item.rotate(angle_deg, axis[0], axis[1], axis[2])
            if sensor in self.sensor_labels:
                new_pos = t + np.array([0, -2, 0])
                self.sensor_labels[sensor].setData(pos=new_pos, text=sensor)

        # calculate the update frequency
        self.current_time = time()
        elapsed_time = self.current_time - self.last_time
        self.last_time = self.current_time
        print(f"Update frequency: {1 / elapsed_time:.2f} Hz")

    def closeEvent(self, event):
        for thread in self.worker_threads:
            thread.stop()
        final_pose_graph = self.pose_graph_manager.get_pose_graph()
        # Save the pose graph data to a file.
        pose_graph_file = "scenegraph_0409_1.json"
        final_pose_graph.save_to_json(pose_graph_file)
        print(f"Pose graph data saved to {pose_graph_file}.")
        self.pose_graph_manager.deleteLater()
        event.accept()


if __name__ == "__main__":
    tool_names = ["artool", "reference_1", "phantom", "extra_tool"]
    app = QtWidgets.QApplication(sys.argv)
    win = PoseGraphGUI(tool_names)
    win.show()
    sys.exit(app.exec_())
