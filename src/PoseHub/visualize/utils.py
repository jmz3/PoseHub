import numpy as np
import os
import sys


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
