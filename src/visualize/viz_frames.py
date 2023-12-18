import numpy as np
from typing import List
from matplotlib import pyplot as plt


def axis_init(ax, limit: float, title: str = "3D plot"):
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_zlabel("z")
    ax.set_xlim(-limit, limit)
    ax.set_ylim(-limit, limit)
    ax.set_zlim(-limit, limit)
    ax.set_title(title, fontsize=20)

    return ax


def generate_frames(
    ax: plt.Axes, sensors: List[str], objects: List[str], axis_length=1.0
):
    """
    Generate frames for visualization
    the number of frames is equal to the number of sensors * objects

    Args:
    ----------
    ax: plt.Axes
        The axis for visualization
    sensors: List[str]
        The list of sensor names
    objects: List[str]
        The list of object names
    axis_length: float
        The length of the axis


    Returns:
    ----------
    frame: dict
        The dictionary containing the frames for visualization
    frame_primitive: np.ndarray
        The primitive axis for visualization
    """
    frame = dict()

    # add prefix to the sensor and object names
    sensors = ["sensor_" + sensor for sensor in sensors]
    objects = ["object_" + obj for obj in objects]

    # Initialize the frames with empty values
    for sensor in sensors:
        frame[sensor] = {}  # Initialize nested dictionary for each sensor
        for obj in objects:
            frame[sensor][obj] = {
                "x": None,
                "y": None,
                "z": None,
                "sensor_tag": None,
                "frame_id": None,
            }

    marker_styles = [
        "o",
        "s",
        "^",
        "v",
        "*",
        "+",
        "x",
        "D",
        "|",
        "_",
    ]  # 10 styles, can be extended if needed
    marker_colors = [
        "r",
        "g",
        "b",
        "c",
        "m",
        "y",
        "k",
        "w",
    ]  # 8 colors, can be extended if needed

    sCount = 0
    for sensor in sensors:
        for obj in objects:
            frame[sensor][obj] = {
                "x": ax.quiver(
                    0, 0, 0, axis_length, 0, 0, color="r", arrow_length_ratio=0.2
                ),
                "y": ax.quiver(
                    0, 0, 0, 0, axis_length, 0, color="g", arrow_length_ratio=0.2
                ),
                "z": ax.quiver(
                    0, 0, 0, 0, 0, axis_length, color="b", arrow_length_ratio=0.2
                ),
                "sensor_tag": ax.scatter(
                    0,
                    0,
                    0,
                    marker=marker_styles[sCount % len(marker_styles)],
                    color=marker_colors[sCount % len(marker_colors)],
                    s=50,
                    alpha=0,  # invisible at first
                ),
                "frame_id": ax.text(
                    0, 0, 0, obj, color="black", fontsize=10, alpha=0
                ),  # invisible at first
            }

        sCount += 1

    frame["world"] = {}
    frame["world"]["world"] = {
        "x": ax.quiver(0, 0, 0, axis_length, 0, 0, color="r", arrow_length_ratio=0.2),
        "y": ax.quiver(0, 0, 0, 0, axis_length, 0, color="g", arrow_length_ratio=0.2),
        "z": ax.quiver(0, 0, 0, 0, 0, axis_length, color="b", arrow_length_ratio=0.2),
        "sensor_tag": ax.scatter(
            0,
            0,
            0,
            marker=marker_styles[sCount % len(marker_styles)],
            color=marker_colors[sCount % len(marker_colors)],
            s=50,
            alpha=0,  # invisible at first
        ),
        "frame_id": ax.text(
            0, 0, 0, "World", color="black", fontsize=10, alpha=1
        ),  # invisible at first
    }

    frame_primitive = np.transpose(
        np.array(
            [
                [0, 0, 0, 1],  # origin
                [axis_length, 0, 0, 1],  # x-axis
                [0, axis_length, 0, 1],  # y-axis
                [0, 0, axis_length, 1],  # z-axis
            ],
            dtype=np.float32,
        )
    )  # 4 x 4 matrix, column-wise view [o, x, y, z],
    # each column is a point for a primitive axis, we need 4 points to define a frame (origin + 3 axes)

    return frame, frame_primitive
