import json
import numpy as np
import matplotlib.pyplot as plt


def parse_relative_translations(data, sensor):
    """Extract relative translations for both raw and opt graphs"""
    results = {
        "raw": {
            "Probe": {"t": [], "x": [], "y": [], "z": []},
            "Anatomy": {"t": [], "x": [], "y": [], "z": []},
        },
        "opt": {
            "Probe": {"t": [], "x": [], "y": [], "z": []},
            "Anatomy": {"t": [], "x": [], "y": [], "z": []},
        },
    }

    # Sort timestamps numerically
    timestamps = sorted(data.keys(), key=lambda x: float(x))

    for ts in timestamps:
        t = float(ts)
        entry = data[ts]

        for graph_type in ["raw_graph", "opt_graph"]:
            key = "raw" if graph_type == "raw_graph" else "opt"

            try:
                graph = entry[graph_type]
                # Get StaticRef transform
                static_ref = np.array(graph[sensor]["object_StaticRef"][0])
                T_static_ref = np.vstack(static_ref[:4])
                static_pos = T_static_ref[:3, 3]

                # Process Anatomy
                anatomy = np.array(graph[sensor]["object_Anatomy"][0])
                T_anatomy = np.vstack(anatomy[:4])
                rel_anatomy = T_anatomy[:3, 3] - static_pos

                # Process Probe
                probe = np.array(graph[sensor]["object_Probe"][0])
                T_probe = np.vstack(probe[:4])
                rel_probe = T_probe[:3, 3] - static_pos

                # Store results - single timestamp per entry
                for obj, vec in [("Anatomy", rel_anatomy), ("Probe", rel_probe)]:
                    results[key][obj]["t"].append(t)
                    results[key][obj]["x"].append(vec[0])
                    results[key][obj]["y"].append(vec[1])
                    results[key][obj]["z"].append(vec[2])

            except KeyError as e:
                print(f"Skipping {ts} in {graph_type}: {e}")
                continue

    return results


def filter_outliers(x_list, y_list, z_list, m=3):
    """
    Remove outliers based on the Euclidean distance from the median point.
    Points with a distance greater than m times the median absolute deviation (MAD) are removed.
    """
    xs = np.array(x_list)
    ys = np.array(y_list)
    zs = np.array(z_list)
    points = np.vstack([xs, ys, zs]).T
    # Compute the median point.
    median_point = np.median(points, axis=0)
    # Compute Euclidean distances from the median.
    distances = np.linalg.norm(points - median_point, axis=1)
    # Compute Median Absolute Deviation (MAD)
    mad = np.median(np.abs(distances - np.median(distances)))
    # If MAD is 0 (all points nearly equal), use standard deviation instead.
    if mad < 1e-8:
        mad = np.std(distances)
    cutoff = m * mad
    # Only keep points with distance below cutoff.
    mask = distances < cutoff
    return xs[mask], ys[mask], zs[mask]


def plot_3d_translations(results):
    """Plot 3D trajectories comparing raw and optimized data (with outlier removal)"""
    plt.figure(figsize=(14, 8))

    # Create two subplots: one for Anatomy, one for Probe
    for idx, obj in enumerate(["Anatomy", "Probe"], 1):
        ax = plt.subplot(1, 2, idx, projection="3d")
        ax.set_title(f"{obj} Relative to StaticRef")

        # Filter outliers for raw data
        raw_x, raw_y, raw_z = (
            results["raw"][obj]["y"],
            results["raw"][obj]["z"],
            results["raw"][obj]["x"],
        )
        # Filter outliers for optimized data
        opt_x, opt_y, opt_z = filter_outliers(
            results["opt"][obj]["y"],
            results["opt"][obj]["z"],
            results["opt"][obj]["x"],
            m=3,
        )

        # Plot raw data (using red circles)
        ax.scatter(
            raw_x,
            raw_y,
            raw_z,
            c="r",
            label="Raw",
            alpha=0.2,
            s=5,
        )

        # Plot optimized data (using blue triangles)
        ax.scatter(
            opt_x,
            opt_y,
            opt_z,
            c="b",
            marker="*",
            label="Optimized",
            alpha=0.7,
            s=20,
        )

        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
        ax.set_zlabel("Z (m)")
        ax.legend()
        ax.grid(True)

        # Set equal aspect ratio based on raw data bounds
        if len(raw_x) > 0:
            max_range = np.max(
                [
                    np.max(raw_x) - np.min(raw_x),
                    np.max(raw_y) - np.min(raw_y),
                    np.max(raw_z) - np.min(raw_z),
                ]
            )
            mid_x = (np.max(raw_x) + np.min(raw_x)) * 0.5
            mid_y = (np.max(raw_y) + np.min(raw_y)) * 0.5
            mid_z = (np.max(raw_z) + np.min(raw_z)) * 0.5

            ax.set_xlim(mid_x - max_range / 2, mid_x + max_range / 2)
            ax.set_ylim(mid_y - max_range / 2, mid_y + max_range / 2)
            ax.set_zlim(mid_z - max_range / 2, mid_z + max_range / 2)

    plt.tight_layout()
    plt.show()


# Load and process data
with open("scenegraph_0411_quant_holo_ndi_atracsys_4.json") as f:
    data = json.load(f)

results = parse_relative_translations(data, sensor="sensor_Holo_1")
plot_3d_translations(results)
