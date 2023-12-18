import numpy as np
from matplotlib import pyplot as plt


def axis_init(ax, limit: float, title: str = "3D plot"):
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_zlabel("z")
    ax.set_xlim(-1, 1)
    ax.set_ylim(-1, 1)
    ax.set_zlim(-1, 1)
    ax.set_aspect("equal")
    ax.set_title(title, fontsize=20)

    return ax
