import os
import sys
import numpy as np
from time import time
from PyQt5 import QtCore, QtWidgets, QtGui
import pyqtgraph.opengl as gl
from scipy.spatial.transform import Rotation as Rot
from .utils import parse_obj_groups, parse_mtl
from posegraph_manager import PoseGraphManager
from pg_thread import PoseGraphThread


class PoseGraphGUI(QtWidgets.QMainWindow):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("PoseHub Visualization - pyqtgraph")
        self.resize(1200, 700)

        # Create the central PoseGraphManager.
        self.manager = PoseGraphManager()
        self.worker_threads = []

        # Define the tool names (order assumed: "artool", "reference_1", "phantom")
        self.tool_names = ["artool", "reference_1", "phantom"]
        self.sensor_names = []

        self.obj_file_path = "/home/jeremy/Research/PoseHub/ExpData/tinker.obj"
        self.sensor_obj_generated = False

        self.tool_labels = {}
        self.sensor_labels = {}

        self.initUI()
        self.manager.pose_graph_updated.connect(self.on_pose_updated)

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
        grid.scale(10, 10, 1)
        self.gl_view.addItem(grid)

        # Right side: Configuration panel.
        config_widget = QtWidgets.QWidget()
        config_layout = QtWidgets.QFormLayout(config_widget)
        self.ip_edit = QtWidgets.QLineEdit("10.0.0.108")
        self.sub_port_edit = QtWidgets.QLineEdit("5588")
        self.pub_port_edit = QtWidgets.QLineEdit("5589")
        self.sensor_name_edit = QtWidgets.QLineEdit("h1")
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
        main_layout.addWidget(config_widget, 1)

        # Load tool OBJ instances.
        self.load_tool_instances(self.obj_file_path)

    def load_tool_instances(self, obj_path):
        try:
            vertices, groups_tri, mtl_file = parse_obj_groups(obj_path)
        except Exception as e:
            QtWidgets.QMessageBox.critical(
                self, "Error", f"Failed to load OBJ file:\n{e}"
            )
            return

        materials = {}
        if mtl_file:
            mtl_path = os.path.join(os.path.dirname(obj_path), mtl_file)
            materials = parse_mtl(mtl_path)
        default_color = (0.8, 0.8, 0.8, 1.0)

        # Create one instance per tool.
        # The order of instance_defs corresponds to self.tool_names.
        instance_defs = [
            {"translation": (0, 0, 0), "rotation": 0},
            {"translation": (20, 0, 0), "rotation": 0},
            {"translation": (0, 20, 0), "rotation": 0},
        ]

        self.tool_instance_data = []
        for idx, inst in enumerate(instance_defs):
            inst_items = []
            base_translation = np.array(inst["translation"], dtype=np.float32)
            base_rotation = inst["rotation"]
            for mat, faces in groups_tri.items():
                meshdata = gl.MeshData(vertexes=vertices, faces=faces)
                mesh_item = gl.GLMeshItem(
                    meshdata=meshdata,
                    smooth=True,
                    drawFaces=True,
                    drawEdges=False,
                    edgeColor=(1, 1, 1, 0),
                )
                color = materials.get(mat, default_color)
                mesh_item.setColor(color)
                mesh_item.translate(*base_translation)
                if base_rotation != 0:
                    mesh_item.rotate(base_rotation, 0, 1, 0)
                self.gl_view.addItem(mesh_item)
                inst_items.append(mesh_item)
            self.tool_instance_data.append(
                {
                    "items": inst_items,
                    "translation": base_translation,
                    "rotation": base_rotation,
                }
            )
            # Create a QLabel for this tool.
            label = QtWidgets.QLabel(self)
            label.setText(self.tool_names[idx])
            label.setStyleSheet("color: white; background-color: transparent;")
            label.adjustSize()
            self.tool_labels[self.tool_names[idx]] = label

    def connect_worker(self):
        # Read settings from the configuration fields.
        ip = self.ip_edit.text()
        sub_port = self.sub_port_edit.text()
        pub_port = self.pub_port_edit.text()
        sensor_name = self.sensor_name_edit.text()

        # Create a simple args object to pass to PoseGraphThread.
        class Args:
            pass

        args = Args()
        args.sub_ip_1 = ip
        args.sub_topic = self.tool_names  # using tool names as topics
        args.pub_topic = self.tool_names
        args.sub_port = sub_port
        args.pub_port = pub_port
        args.sensor_name = sensor_name

        # Create a new PoseGraphThread using the shared manager.
        new_thread = PoseGraphThread(args, self.manager)
        new_thread.start()
        self.worker_threads.append(new_thread)
        print(f"Added connection for sensor: {sensor_name}")

        # If this sensor is new, create its visual instance and label.
        if sensor_name not in self.sensor_names:
            self.sensor_names.append(sensor_name)
            # Create a simple sensor model using a cube.
            cube_meshdata = gl.MeshData.cube()  # built-in cube mesh
            sensor_item = gl.GLMeshItem(
                meshdata=cube_meshdata,
                smooth=True,
                drawFaces=True,
                drawEdges=True,
                edgeColor=(1, 1, 1, 1),
            )
            # Set an initial size and position.
            sensor_item.scale(5, 5, 5)
            sensor_item.translate(0, 0, 0)
            self.gl_view.addItem(sensor_item)
            # Save sensor instance data in a dict.
            if not hasattr(self, "sensor_instance_data"):
                self.sensor_instance_data = {}
            self.sensor_instance_data[sensor_name] = {
                "item": sensor_item,
                "translation": np.array([0, 0, 0], dtype=np.float32),
                "rotation": 0,
            }
            # Create a QLabel for the sensor.
            label = QtWidgets.QLabel(self)
            label.setText(sensor_name)
            label.setStyleSheet("color: white; background-color: transparent;")
            label.adjustSize()
            self.sensor_labels[sensor_name] = label

    def start_tracking(self):
        # Activate tracking on all worker threads.
        for thread in self.worker_threads:
            thread.start_tracking()
        print("Tracking started on all connections.")

    def project_to_screen(self, pos):
        """
        A stub projection function.
        Given a 3D position (np.array), return 2D screen coordinates.
        You should implement this using your GLViewWidget's projection and view matrices.
        """
        # For demonstration, we'll do a simple scaling and offset.
        x = int(pos[0] * 20 + 300)
        y = int(-pos[1] * 20 + 300)
        return x, y

    def on_pose_updated(self, pose_graph):
        # Update tool labels.
        for idx, tool in enumerate(self.tool_names):
            transform = pose_graph.get_transform("h1", tool, solver_method="BFS")
            if transform is None:
                continue
            t = transform[:3, 3]
            R_mat = transform[:3, :3]
            rot = Rot.from_matrix(R_mat)
            rotvec = rot.as_rotvec()
            angle_rad = np.linalg.norm(rotvec)
            angle_deg = np.degrees(angle_rad)
            axis = (
                rotvec / angle_rad
                if angle_rad > 1e-6
                else np.array([0, 1, 0], dtype=np.float32)
            )
            for item in self.tool_instance_data[idx]["items"]:
                item.resetTransform()
                item.translate(t[0], t[1], t[2])
                if angle_deg > 1e-3:
                    item.rotate(angle_deg, axis[0], axis[1], axis[2])
            # Update the tool label position.
            screen_x, screen_y = self.project_to_screen(t)
            if tool in self.tool_labels:
                self.tool_labels[tool].move(screen_x - 20, screen_y + 20)

        # Update sensor labels and sensor instances.
        for sensor in self.sensor_names:
            transform = pose_graph.get_transform(sensor, "artool", solver_method="BFS")
            # Here we assume the sensor transform can be obtained by using the sensor name as parent.
            # Adjust as needed.
            if transform is None:
                continue
            t = transform[:3, 3]
            R_mat = transform[:3, :3]
            rot = Rot.from_matrix(R_mat)
            rotvec = rot.as_rotvec()
            angle_rad = np.linalg.norm(rotvec)
            angle_deg = np.degrees(angle_rad)
            axis = (
                rotvec / angle_rad
                if angle_rad > 1e-6
                else np.array([0, 1, 0], dtype=np.float32)
            )
            sensor_data = self.sensor_instance_data.get(sensor, None)
            if sensor_data is not None:
                sensor_item = sensor_data["item"]
                sensor_item.resetTransform()
                sensor_item.translate(t[0], t[1], t[2])
                if angle_deg > 1e-3:
                    sensor_item.rotate(angle_deg, axis[0], axis[1], axis[2])
                screen_x, screen_y = self.project_to_screen(t)
                if sensor in self.sensor_labels:
                    self.sensor_labels[sensor].move(screen_x - 20, screen_y + 20)

    def closeEvent(self, event):
        for thread in self.worker_threads:
            thread.stop()
        event.accept()
