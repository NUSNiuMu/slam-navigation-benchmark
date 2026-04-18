import numpy as np
from PIL import Image
import os
import sys

def convert_pcd_to_pgm(pcd_path, output_prefix, resolution=0.05, min_z=None, max_z=None):
    print(f"Reading PCD: {pcd_path}")
    
    with open(pcd_path, 'rb') as f:
        header = ""
        while True:
            line = f.readline().decode('ascii', errors='ignore')
            header += line
            if line.startswith('DATA'):
                data_mode = line.strip().split()[1]
                break
        
        # Parse fields
        fields = {}
        for line in header.split('\n'):
            line = line.strip()
            if line.startswith('FIELDS'):
                fields['names'] = line.split()[1:]
            if line.startswith('WIDTH'):
                fields['width'] = int(line.split()[1])
            if line.startswith('HEIGHT'):
                fields['height'] = int(line.split()[1])
            if line.startswith('POINTS'):
                fields['points'] = int(line.split()[1])
                
        num_fields = len(fields['names'])
        print(f"Fields: {fields['names']}, Points: {fields['points']}, Mode: {data_mode}")
        
        if data_mode == 'binary':
            # Fast binary read
            raw_data = f.read()
            expected_size = fields['points'] * num_fields * 4
            if len(raw_data) < expected_size:
                print(f"Warning: File smaller than expected. Read {len(raw_data)} bytes, expected {expected_size}")
            # Ensure we only use the expected number of points
            data = np.frombuffer(raw_data, dtype=np.float32)
            data = data[:fields['points'] * num_fields].reshape(-1, num_fields)
        else:
            print("Only binary PCD is supported in this script version.")
            return

    # Extract X, Y, Z
    x = data[:, 0]
    y = data[:, 1]
    z = data[:, 2]

    z_min_actual, z_max_actual = np.min(z), np.max(z)
    print(f"Actual Z range in PCD: {z_min_actual:.3f} to {z_max_actual:.3f}")

    # Default Z filter if not provided
    # Narrowing to avoid ground (0.0) and ceiling
    if min_z is None: min_z = 0.2 
    if max_z is None: max_z = 1.0
    
    print(f"Applying Z filter: {min_z} < z < {max_z}")
    mask = (z > min_z) & (z < max_z)
    x_filtered, y_filtered = x[mask], y[mask]

    if len(x_filtered) == 0:
        print("Error: No points left after Z filtering!")
        return

    # Grid mapping
    x_min, x_max = np.min(x_filtered), np.max(x_filtered)
    y_min, y_max = np.min(y_filtered), np.max(y_filtered)
    
    res = resolution
    width = int((x_max - x_min) / res) + 1
    height = int((y_max - y_min) / res) + 1
    
    print(f"Grid size: {width}x{height}, Origin: ({x_min:.2f}, {y_min:.2f})")
    
    # Use a count logic to reduce noise: only cells with > N points are obstacles
    # 255 is free, 0 is occupied
    grid = np.full((height, width), 255, dtype=np.uint8)
    
    ix = ((x_filtered - x_min) / res).astype(int)
    iy = ((y_filtered - y_min) / res).astype(int)
    
    # Boundary check
    ix = np.clip(ix, 0, width - 1)
    iy = np.clip(iy, 0, height - 1)

    # Simple noise filter: use a 2D histogram
    counts, xedges, yedges = np.histogram2d(x_filtered, y_filtered, bins=[
        np.arange(x_min, x_max + res, res),
        np.arange(y_min, y_max + res, res)
    ])
    
    # Threshold: more than 2 points in a 5cm cell to be a real obstacle
    # This removes floating dust/noise.
    threshold = 2
    occupied = counts.T > threshold # Transpose to match image [y, x]
    
    # Fill grid
    grid[:occupied.shape[0], :occupied.shape[1]][occupied] = 0
    
    # Save image (Vertical flip for ROS parity)
    img = Image.fromarray(grid[::-1, :])
    img.save(f"{output_prefix}.pgm")
    
    # Save YAML
    with open(f"{output_prefix}.yaml", 'w') as f:
        f.write(f"image: {os.path.basename(output_prefix)}.pgm\n")
        f.write(f"resolution: {res}\n")
        f.write(f"origin: [{x_min}, {y_min}, 0.0]\n")
        f.write("negate: 0\n")
        f.write("occupied_thresh: 0.65\n")
        f.write("free_thresh: 0.196\n")

    print(f"Success! Map saved with noise reduction to {output_prefix}.pgm/yaml")

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python3 pcd_to_grid.py <input.pcd> <output_prefix> [min_z] [max_z]")
    else:
        p_min_z = float(sys.argv[3]) if len(sys.argv) > 3 else None
        p_max_z = float(sys.argv[4]) if len(sys.argv) > 4 else None
        convert_pcd_to_pgm(sys.argv[1], sys.argv[2], min_z=p_min_z, max_z=p_max_z)
