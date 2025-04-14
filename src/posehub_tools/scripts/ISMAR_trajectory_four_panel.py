import json
import numpy as np
import matplotlib.pyplot as plt


def parse_relative_translations(data, sensor, pivot=np.array([0.0, 0.0, 0.0, 1.0])):
    """Extract relative translations for Probe (raw and opt graphs) for the given sensor."""
    results = {
        "raw": {"Probe": {"t": [], "x": [], "y": [], "z": []}},
        "opt": {"Probe": {"t": [], "x": [], "y": [], "z": []}},
    }

    # Sort timestamps numerically.
    timestamps = sorted(data.keys(), key=lambda x: float(x))

    for ts in timestamps:
        t = float(ts)
        entry = data[ts]
        for graph_type in ["raw_graph", "opt_graph"]:
            key = "raw" if graph_type == "raw_graph" else "opt"
            try:
                graph = entry[graph_type]
                # Get StaticRef transform for the current sensor.
                static_ref = np.array(graph[sensor]["object_StaticRef"][0])
                T_static_ref = np.vstack(static_ref[:4])
                static_pos = T_static_ref[:3, 3]

                # Process Probe.
                probe = np.array(graph[sensor]["object_Probe"][0])
                T_probe = np.vstack(probe[:4])
                T_rel = np.linalg.inv(T_static_ref) @ T_probe
                rel_probe = T_rel @ pivot

                # Swap x and z for better visualization.
                results[key]["Probe"]["t"].append(t)
                results[key]["Probe"]["x"].append(rel_probe[2])
                results[key]["Probe"]["y"].append(rel_probe[1])
                results[key]["Probe"]["z"].append(rel_probe[0])
            except KeyError as e:
                print(f"Skipping timestamp {ts} in {graph_type} for {sensor}: {e}")
                continue

    return results


def filter_outliers(x_list, y_list, z_list, m=3):
    """
    Remove outliers based on the Euclidean distance from the median point.
    Points farther than m times the median absolute deviation (MAD) are removed.
    """
    xs = np.array(x_list)
    ys = np.array(y_list)
    zs = np.array(z_list)
    points = np.vstack([xs, ys, zs]).T

    median_point = np.median(points, axis=0)
    distances = np.linalg.norm(points - median_point, axis=1)
    mad = np.median(np.abs(distances - np.median(distances)))
    if mad < 1e-8:
        mad = np.std(distances)
    cutoff = m * mad
    mask = distances < cutoff
    return xs[mask], ys[mask], zs[mask]


def plot_probe_data(results_dict, results_opt):
    """
    Plot four subplots:
      1. Raw Probe data for sensor_Holo_1 (red)
      2. Raw Probe data for sensor_NDI (red)
      3. Optimized Probe data for sensor_Holo_1 (blue)
      4. Raw Probe data for sensor_Atracsys (green)
    """
    fig = plt.figure(figsize=(20, 5))

    # Subplot 1: Raw Probe for sensor_Holo_1 in red.
    sensor = "sensor_Holo_1"
    ax1 = fig.add_subplot(1, 4, 1, projection="3d")
    raw_x = np.array(results_dict[sensor]["raw"]["Probe"]["x"])
    raw_y = np.array(results_dict[sensor]["raw"]["Probe"]["y"])
    raw_z = np.array(results_dict[sensor]["raw"]["Probe"]["z"])
    f_x, f_y, f_z = raw_x, raw_y, raw_z  # Optionally you can filter here if needed.
    ax1.scatter(f_x, f_y, f_z, c="r", alpha=0.3, s=5)
    ax1.set_title("Raw Probe - sensor_Holo_1")
    ax1.set_xlabel("X (m)")
    ax1.set_ylabel("Y (m)")
    ax1.set_zlabel("Z (m)")
    ax1.grid(True)
    if len(f_x) > 0:
        max_range = np.max([np.ptp(f_x), np.ptp(f_y), np.ptp(f_z)])
        mid_x = (np.max(f_x) + np.min(f_x)) / 2
        mid_y = (np.max(f_y) + np.min(f_y)) / 2
        mid_z = (np.max(f_z) + np.min(f_z)) / 2
        ax1.set_xlim(mid_x - max_range / 2, mid_x + max_range / 2)
        ax1.set_ylim(mid_y - max_range / 2, mid_y + max_range / 2)
        ax1.set_zlim(mid_z - max_range / 2, mid_z + max_range / 2)

    # Subplot 2: Raw Probe for sensor_NDI in red.
    sensor = "sensor_NDI"
    ax2 = fig.add_subplot(1, 4, 2, projection="3d")
    raw_x = np.array(results_dict[sensor]["raw"]["Probe"]["x"])
    raw_y = np.array(results_dict[sensor]["raw"]["Probe"]["y"])
    raw_z = np.array(results_dict[sensor]["raw"]["Probe"]["z"])
    f_x, f_y, f_z = raw_x, raw_y, raw_z
    ax2.scatter(f_x, f_y, f_z, c="r", alpha=0.3, s=5)
    ax2.set_title("Raw Probe - sensor_NDI")
    ax2.set_xlabel("X (m)")
    ax2.set_ylabel("Y (m)")
    ax2.set_zlabel("Z (m)")
    ax2.grid(True)
    if len(f_x) > 0:
        max_range = np.max([np.ptp(f_x), np.ptp(f_y), np.ptp(f_z)])
        mid_x = (np.max(f_x) + np.min(f_x)) / 2
        mid_y = (np.max(f_y) + np.min(f_y)) / 2
        mid_z = (np.max(f_z) + np.min(f_z)) / 2
        ax2.set_xlim(mid_x - max_range / 2, mid_x + max_range / 2)
        ax2.set_ylim(mid_y - max_range / 2, mid_y + max_range / 2)
        ax2.set_zlim(mid_z - max_range / 2, mid_z + max_range / 2)

    # Subplot 3: Optimized Probe for sensor_Holo_1 in blue.
    ax3 = fig.add_subplot(1, 4, 3, projection="3d")
    opt_x = np.array(results_opt["opt"]["Probe"]["x"])
    opt_y = np.array(results_opt["opt"]["Probe"]["y"])
    opt_z = np.array(results_opt["opt"]["Probe"]["z"])
    f_opt_x, f_opt_y, f_opt_z = filter_outliers(opt_x, opt_y, opt_z, m=10)
    ax3.scatter(f_opt_x, f_opt_y, f_opt_z, c="b", marker="*", alpha=0.3, s=10)
    ax3.set_title("Optimized Probe - sensor_Holo_1")
    ax3.set_xlabel("X (m)")
    ax3.set_ylabel("Y (m)")
    ax3.set_zlabel("Z (m)")
    ax3.grid(True)
    if len(f_opt_x) > 0:
        max_range = np.max([np.ptp(f_opt_x), np.ptp(f_opt_y), np.ptp(f_opt_z)])
        mid_x = (np.max(f_opt_x) + np.min(f_opt_x)) / 2
        mid_y = (np.max(f_opt_y) + np.min(f_opt_y)) / 2
        mid_z = (np.max(f_opt_z) + np.min(f_opt_z)) / 2
        ax3.set_xlim(mid_x - max_range / 2, mid_x + max_range / 2)
        ax3.set_ylim(mid_y - max_range / 2, mid_y + max_range / 2)
        ax3.set_zlim(mid_z - max_range / 2, mid_z + max_range / 2)

    # Subplot 4: Raw Probe for sensor_Atracsys in green.
    sensor = "sensor_Atracsys"
    ax4 = fig.add_subplot(1, 4, 4, projection="3d")
    raw_x = np.array(results_dict[sensor]["raw"]["Probe"]["x"])
    raw_y = np.array(results_dict[sensor]["raw"]["Probe"]["y"])
    raw_z = np.array(results_dict[sensor]["raw"]["Probe"]["z"])
    f_x, f_y, f_z = raw_x, raw_y, raw_z
    ax4.scatter(f_x, f_y, f_z, c="g", alpha=0.3, s=5)
    ax4.set_title("Raw Probe - sensor_Atracsys")
    ax4.set_xlabel("X (m)")
    ax4.set_ylabel("Y (m)")
    ax4.set_zlabel("Z (m)")
    ax4.grid(True)
    if len(f_x) > 0:
        max_range = np.max([np.ptp(f_x), np.ptp(f_y), np.ptp(f_z)])
        mid_x = (np.max(f_x) + np.min(f_x)) / 2
        mid_y = (np.max(f_y) + np.min(f_y)) / 2
        mid_z = (np.max(f_z) + np.min(f_z)) / 2
        ax4.set_xlim(mid_x - max_range / 2, mid_x + max_range / 2)
        ax4.set_ylim(mid_y - max_range / 2, mid_y + max_range / 2)
        ax4.set_zlim(mid_z - max_range / 2, mid_z + max_range / 2)

    plt.tight_layout()
    plt.show()


def load_data(filepath):
    """Load JSON data from a file."""
    with open(filepath) as f:
        return json.load(f)


def main():
    # Filepath to the JSON file.
    filepath = "scenegraph_0411_quant_holo_ndi_atracsys_4.json"
    data = load_data(filepath)

    # Sensors for raw Probe display.
    sensors = ["sensor_Holo_1", "sensor_NDI", "sensor_Atracsys"]

    # Parse raw Probe data for each sensor.
    pivot = np.array([0.0482, -0.1298, 0.0085, 1.0])

    results_dict = {}
    for sensor in sensors:
        results_dict[sensor] = parse_relative_translations(data, sensor, pivot)

    # Parse optimized Probe data for sensor_Holo_1.
    results_opt = parse_relative_translations(data, sensor="sensor_Holo_1", pivot=pivot)

    # Plot the data with the required subplots.
    plot_probe_data(results_dict, results_opt)


if __name__ == "__main__":
    main()
