import json
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.patches import FancyArrowPatch
from mpl_toolkits.mplot3d import proj3d
from time import time

class Arrow3D(FancyArrowPatch):
    def __init__(self, xs, ys, zs, *args, **kwargs):
        super().__init__((0,0), (0,0), *args, **kwargs)
        self._verts3d = xs, ys, zs

    def do_3d_projection(self, renderer=None):
        xs3d, ys3d, zs3d = self._verts3d
        xs, ys, zs = proj3d.proj_transform(xs3d, ys3d, zs3d, self.axes.M)
        self.set_positions((xs[0], ys[0]), (xs[1], ys[1]))
        return np.min(zs)

# Load data
with open('scenegraph_0409_1.json') as f:
    data = json.load(f)

# Convert string timestamps to float and sort chronologically
timestamps = sorted([float(ts) for ts in data.keys()])

# Objects to plot
objects = ['object_Probe', 'object_StaticRef', 'object_Anatomy']
colors = {'object_Probe': 'red', 'object_StaticRef': 'blue', 'object_Anatomy': 'green'}
arrow_colors = ['darkred', 'darkgreen', 'navy']
scale = 0.1  # Arrow length

# Create separate figures for each object
for obj_name in objects:
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    
            
    # Store positions for trajectory line
    positions = []
    
    for ts in timestamps:
        timestamp_str = f"{ts}"  # Convert back to string format
        raw_graph = data[timestamp_str]['raw_graph']
        
        if obj_name in raw_graph.get('sensor_Holo_1', {}):
            transform = raw_graph['sensor_Holo_1'][obj_name][0]
            
            # Extract position (translation)
            pos = np.array([transform[0][3], transform[1][3], transform[2][3]])
            positions.append(pos)
            
            # Extract rotation matrix
            rot_matrix = np.array([
                transform[0][:3],
                transform[1][:3],
                transform[2][:3]
            ])

            # Plot orientation arrows
            for i, (color, vec) in enumerate(zip(arrow_colors, rot_matrix.T)):
                arrow = Arrow3D(
                    [pos[0], pos[0] + vec[0] * scale],
                    [pos[1], pos[1] + vec[1] * scale],
                    [pos[2], pos[2] + vec[2] * scale],
                    mutation_scale=15, lw=1, arrowstyle="-|>", color=color
                )
                ax.add_artist(arrow)
    
    # Plot trajectory line connecting all positions
    ax.set_title(f"{obj_name} Pose Over Time")
    if positions:
        positions = np.array(positions)
        ax.plot(positions[:, 0], positions[:, 1], positions[:, 2], 
                c=colors[obj_name], linestyle='--', alpha=0.5, label='Trajectory')
   
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend()

plt.show()