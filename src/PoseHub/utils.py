import numpy as np
from scipy.spatial.transform import Rotation as R
import os
import sys
import pyqtgraph.opengl as gl


def hat(omega):
    """Return the hat operator of a 3-vector."""
    return np.array(
        [
            [0, -omega[2], omega[1]],
            [omega[2], 0, -omega[0]],
            [-omega[1], omega[0], 0],
        ]
    )


def se3_exp(x):
    """
    Exponential map for SE(3).
    x: 6-vector, with x[0:3] = rotation (angle-axis) and x[3:6] = translation.
    Returns a 4x4 transformation matrix.
    """
    omega = x[0:3]
    v = x[3:6]
    theta = np.linalg.norm(omega)
    if theta < 1e-8:
        R_mat = np.eye(3)
        V = np.eye(3)
    else:
        R_mat = R.from_rotvec(omega).as_matrix()
        omega_hat = hat(omega)
        V = (
            np.eye(3)
            + (1 - np.cos(theta)) / (theta**2) * omega_hat
            + (theta - np.sin(theta)) / (theta**3) * (omega_hat @ omega_hat)
        )
    T = np.eye(4)
    T[:3, :3] = R_mat
    T[:3, 3] = V @ v
    return T


def se3_log(T):
    """
    Logarithm map for SE(3).
    Returns a 6-vector: first 3 for rotation, last 3 for translation.
    """
    R_mat = T[:3, :3]
    t = T[:3, 3]
    cos_theta = (np.trace(R_mat) - 1) / 2
    cos_theta = np.clip(cos_theta, -1.0, 1.0)
    theta = np.arccos(cos_theta)
    if abs(theta) < 1e-8:
        omega = np.zeros(3)
        V_inv = np.eye(3)
    else:
        omega = (
            theta
            / (2 * np.sin(theta))
            * np.array(
                [
                    R_mat[2, 1] - R_mat[1, 2],
                    R_mat[0, 2] - R_mat[2, 0],
                    R_mat[1, 0] - R_mat[0, 1],
                ]
            )
        )
        omega_hat = hat(omega)
        V_inv = (
            np.eye(3)
            - 0.5 * omega_hat
            + (1 / (theta**2) - (1 + np.cos(theta)) / (2 * theta * np.sin(theta)))
            * (omega_hat @ omega_hat)
        )
    v = V_inv @ t
    return np.concatenate((omega, v))


def parse_mtl(mtl_path):
    materials = {}
    current_mat = None
    try:
        with open(mtl_path, "r") as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith("#"):
                    continue
                parts = line.split()
                if parts[0] == "newmtl":
                    current_mat = parts[1]
                elif parts[0] == "Kd" and current_mat is not None:
                    r, g, b = float(parts[1]), float(parts[2]), float(parts[3])
                    materials[current_mat] = (r, g, b, 1.0)
    except Exception as e:
        print(f"Failed to parse MTL file {mtl_path}: {e}")
    return materials


def parse_obj_groups(obj_path):
    vertices = []
    groups = {}
    current_material = None
    mtl_file = None

    with open(obj_path, "r") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if parts[0] == "mtllib" and len(parts) > 1:
                mtl_file = parts[1]
            elif parts[0] == "v":
                vertices.append(
                    [float(parts[1]) / 10, float(parts[2]) / 10, float(parts[3]) / 10]
                )
            elif parts[0] == "usemtl":
                current_material = parts[1]
                if current_material not in groups:
                    groups[current_material] = []
            elif parts[0] == "f":
                face = []
                for token in parts[1:]:
                    idx = token.split("/")[0]
                    face.append(int(idx) - 1)
                if current_material is None:
                    current_material = "default"
                    if current_material not in groups:
                        groups[current_material] = []
                groups[current_material].append(face)

    groups_tri = {}
    for mat, faces in groups.items():
        tri_faces = []
        for face in faces:
            if len(face) == 3:
                tri_faces.append(face)
            elif len(face) > 3:
                for i in range(1, len(face) - 1):
                    tri_faces.append([face[0], face[i], face[i + 1]])
        groups_tri[mat] = np.array(tri_faces, dtype=np.int32)
    vertices = np.array(vertices, dtype=np.float32)
    return vertices, groups_tri, mtl_file


def load_instance_from_obj(obj_path, translate: np.ndarray = None, rotate: float = 0):
    """
    load an instance from an OBJ file as a GLMeshItem, with its material properties.
    """

    try:
        vertices, groups_tri, mtl_file = parse_obj_groups(obj_path)
    except Exception as e:
        print(f"Failed to load OBJ file:\n{e}")
        return None

    materials = {}
    if mtl_file:
        mtl_path = os.path.join(os.path.dirname(obj_path), mtl_file)
        materials = parse_mtl(mtl_path)
    default_color = (0.8, 0.8, 0.8, 1.0)

    inst_items = []
    for mat, faces in groups_tri.items():
        frame_mesh = gl.MeshData(vertexes=vertices, faces=faces)

        # Scale the mesh to 1/10th of its size
        for vertex in frame_mesh.vertexes():
            vertex *= 0.5

        frame_item = gl.GLMeshItem(
            meshdata=frame_mesh, smooth=True, drawFaces=True, drawEdges=False
        )
        color = materials.get(mat, default_color)
        frame_item.setColor(color)
        if translate is not None:
            frame_item.translate(*translate)
        if rotate != 0:
            frame_item.rotate(rotate, 0, 1, 0)
        inst_items.append(frame_item)

    print(f"Loaded instance from {obj_path}")
    return inst_items


class ZMQConfig:
    """
    Configuration for ZMQ communication.
    """

    def __init__(self, sub_ip, sub_port, pub_port, topic, sensor_name):
        self.sub_ip = sub_ip
        self.sub_topic = topic  # using tool names as topics
        self.pub_topic = topic
        self.sub_port = sub_port
        self.pub_port = pub_port
        self.sensor_name = sensor_name


if __name__ == "__main__":
    obj_path = "/home/jeremy/Research/PoseHub/ExpData/tinker.obj"
    load_instance_from_obj(obj_path)
