import json
import numpy as np
import matplotlib.pyplot as plt

def parse_relative_translations(data):
    """Extract relative translations for both raw and opt graphs"""
    results = {
        'raw': {'Probe': {'t': [], 'x': [], 'y': [], 'z': []},
                'Anatomy': {'t': [], 'x': [], 'y': [], 'z': []}},
        'opt': {'Probe': {'t': [], 'x': [], 'y': [], 'z': []},
                'Anatomy': {'t': [], 'x': [], 'y': [], 'z': []}}
    }
    
    # Sort timestamps numerically
    timestamps = sorted(data.keys(), key=lambda x: float(x))
    
    for ts in timestamps:
        t = float(ts)
        entry = data[ts]
        
        for graph_type in ['raw_graph', 'opt_graph']:
            key = 'raw' if graph_type == 'raw_graph' else 'opt'
            
            try:
                graph = entry[graph_type]
                # Get StaticRef transform
                static_ref = np.array(graph['sensor_Holo_1']['object_StaticRef'][0])
                T_static_ref = np.vstack(static_ref[:4])
                static_pos = T_static_ref[:3, 3]
                
                # Process Anatomy
                anatomy = np.array(graph['sensor_Holo_1']['object_Anatomy'][0])
                T_anatomy = np.vstack(anatomy[:4])
                rel_anatomy = T_anatomy[:3, 3] - static_pos
                
                # Process Probe
                probe = np.array(graph['sensor_Holo_1']['object_Probe'][0])
                T_probe = np.vstack(probe[:4])
                rel_probe = T_probe[:3, 3] - static_pos
                
                # Store results - single timestamp per entry
                for obj, vec in [('Anatomy', rel_anatomy), ('Probe', rel_probe)]:
                    results[key][obj]['t'].append(t)
                    results[key][obj]['x'].append(vec[0])
                    results[key][obj]['y'].append(vec[1])
                    results[key][obj]['z'].append(vec[2])
                    
            except KeyError as e:
                print(f"Skipping {ts} in {graph_type}: {e}")
                continue
                
    return results

def plot_3d_translations(results):
    """Plot 3D trajectories comparing raw and optimized data"""
    plt.figure(figsize=(14, 8))
    
    # Create two subplots: one for Anatomy, one for Probe
    for idx, obj in enumerate(['Anatomy', 'Probe'], 1):
        ax = plt.subplot(1, 2, idx, projection='3d')
        ax.set_title(f'{obj} Relative to StaticRef')
        
        # Plot raw data
        ax.plot(results['raw'][obj]['x'], 
                results['raw'][obj]['y'], 
                results['raw'][obj]['z'],
                'r-', label='Raw', alpha=0.7, linewidth=1)
        
        # Plot optimized data
        ax.plot(results['opt'][obj]['x'], 
                results['opt'][obj]['y'], 
                results['opt'][obj]['z'],
                'b--', label='Optimized', alpha=0.7, linewidth=1)
        
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Z (m)')
        ax.legend()
        ax.grid(True)
        
        # Set equal aspect ratio
        max_range = np.max([np.max(results['raw'][obj]['x']) - np.min(results['raw'][obj]['x']),
                           np.max(results['raw'][obj]['y']) - np.min(results['raw'][obj]['y']),
                           np.max(results['raw'][obj]['z']) - np.min(results['raw'][obj]['z'])])
        mid_x = (np.max(results['raw'][obj]['x']) + np.min(results['raw'][obj]['x'])) * 0.5
        mid_y = (np.max(results['raw'][obj]['y']) + np.min(results['raw'][obj]['y'])) * 0.5
        mid_z = (np.max(results['raw'][obj]['z']) + np.min(results['raw'][obj]['z'])) * 0.5
        
        ax.set_xlim(mid_x - max_range/2, mid_x + max_range/2)
        ax.set_ylim(mid_y - max_range/2, mid_y + max_range/2)
        ax.set_zlim(mid_z - max_range/2, mid_z + max_range/2)

    plt.tight_layout()
    plt.show()

# Load and process data
with open('scenegraph_0409_1.json') as f:
    data = json.load(f)

results = parse_relative_translations(data)
plot_3d_translations(results)