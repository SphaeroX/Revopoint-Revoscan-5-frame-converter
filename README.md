# Revopoint 3D Scanner Depth to Point Cloud Conversion

This project is designed to help users convert depth frames captured by the Revopoint 3D scanner into point cloud files. The frames are stored as `.dph` files in the cache directory of the Revopoint project, and this script converts them into `.ply` format, which can be visualized or processed further using 3D software.

## Project Structure

Below is an overview of the relevant files and what they contain. The files come from a typical Revopoint project folder.

### Cache Folder

- **`cache/`**: This folder contains depth frames in `.dph` format, and their corresponding `.inf` files. The `.dph` files store depth information, while the `.inf` files seem to contain metadata about the depth frames.

### Param Folder

- **`param/Distort.bin`**: Contains distortion coefficients for correcting radial and tangential distortion of the depth data.
    - Example coefficients: `k1 = 0.2549`, `k2 = -0.9970`, `p1 = -0.000676`, `p2 = 0.001375`, `k3 = 1.8080`
- **`param/mapparamL.bin`** and **`param/mapparamR.bin`**: These files contain mapping parameters for the left and right sides of the depth frames. They likely hold optical center positions and scaling factors for each side of the stereo camera setup.
- **`param/Q.bin`**: This file holds the **disparity-to-depth** matrix, which is crucial for converting disparity (from stereo images) into depth information. It's a 4x4 matrix that transforms pixel coordinates into 3D space.
    - Example matrix:
      ```
      [[ 1.00000000e+00,  0.00000000e+00,  0.00000000e+00, -5.21716766e+01],
       [ 0.00000000e+00,  1.00000000e+00,  0.00000000e+00, -1.87756104e+02],
       [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  8.83338623e+02],
       [ 0.00000000e+00,  0.00000000e+00,  1.13238180e-02,  5.77776909e+00]]
      ```
- **`param/LC_RT.bin`**: This file might contain rotation and translation matrices for aligning the camera or synchronizing depth and color frames. The exact structure wasn't fully analyzed.

### Other Files

- **`property.rvproj`**: This is the main project file for the Revopoint 3D scanner, storing metadata about the project.
- **Other binary files**: Files like `Pl.bin` and `Prgb.bin` seem to contain camera calibration data such as focal lengths and projection matrices.

## What We've Tried

### Initial Attempts

1. **Distortion Correction**: Using the distortion coefficients from `Distort.bin`, we applied radial and tangential corrections to the depth frames. However, this alone was not enough to fix the distortions in the point clouds.
2. **Mapping Parameters (`mapparamL.bin` and `mapparamR.bin`)**: We attempted to correct left and right side depth frame data separately using mapping parameters. While this provided some improvement, it still wasn't sufficient to fully remove the distortions.
3. **Disparity-to-Depth Matrix (`Q.bin`)**: The `Q.bin` file contains a 4x4 matrix that maps pixel coordinates to real-world 3D coordinates. Applying this matrix to the depth frames yielded the best results and successfully corrected the point cloud distortions.

### Final Approach

The final script reads the depth frames (`.dph` files), applies the disparity-to-depth transformation using the `Q.bin` matrix, and saves the corrected point clouds as `.ply` files. These `.ply` files can be visualized using software like Meshlab or CloudCompare.

## How to Use the Script

1. **Copy the Script**: Place the provided Python script in the project directory of your Revopoint 3D scanner project. The script should be copied to the folder containing the `cache` and `param` directories.

    Example path (adjust based on your project location):
    
    ```
    C:\Users\xxxxx\AppData\Roaming\RevoScan5\Projects\Project10172024181138\data\26009d86-2567-45c3-8325-a072830c4ff7\
    ```

2. **Run the Script**: You can run the script using Python. It will automatically detect the depth frames in the `cache` folder and convert them into point clouds.

3. **Install Required Libraries**: If you don't have the necessary libraries installed, you can install them using pip:
    ```bash
    pip install numpy plyfile
    ```

4. **Output**: The point clouds will be saved in a new folder named `pointclouds` within the project directory, with each `.ply` file corresponding to one `.dph` frame.

### Python Script

```python
import os
import struct
import numpy as np
from plyfile import PlyData, PlyElement

# Function to read the Q matrix from Q.bin
def read_q_matrix(file_path):
    with open(file_path, 'rb') as f:
        q_content = f.read()
    q_matrix = struct.unpack('f' * (len(q_content) // 4), q_content)
    q_matrix = np.array(q_matrix).reshape((4, 4))  # Reshape as 4x4 matrix
    return q_matrix

# Function to read .dph files and convert them into point clouds
def dph_to_point_cloud(dph_path):
    with open(dph_path, 'rb') as f:
        depth_data = np.fromfile(f, dtype=np.uint16)
        width, height = 640, 400
        depth_data = depth_data.reshape((height, width)).astype(np.float32)  # Convert to float32
        depth_data[depth_data == 0] = np.nan  # Remove background (depth = 0)
    return depth_data

# Function to apply Q matrix to the point cloud for disparity-to-depth conversion
def apply_q_matrix(x, y, z, q_matrix):
    w = z * q_matrix[3, 2] + q_matrix[3, 3]
    x_transformed = (x * q_matrix[0, 0] + q_matrix[0, 3]) / w
    y_transformed = (y * q_matrix[1, 1] + q_matrix[1, 3]) / w
    z_transformed = q_matrix[2, 3] / w
    return x_transformed, y_transformed, z_transformed

# Function to save point cloud to .ply file
def save_point_cloud_to_ply(x, y, z, output_path):
    height, width = z.shape
    vertex = []
    for i in range(height):
        for j in range(width):
            if not np.isnan(z[i, j]):
                vertex.append((x[i, j], y[i, j], z[i, j]))  # Valid depth points
    vertex = np.array(vertex, dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4')])
    
    el = PlyElement.describe(vertex, 'vertex')
    PlyData([el]).write(output_path)

# Function to process all frames and save as .ply files
def process_and_save_all_frames(cache_folder, q_matrix, output_folder):
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)
    
    for file_name in os.listdir(cache_folder):
        if file_name.endswith('.dph'):
            dph_path = os.path.join(cache_folder, file_name)
            depth_data = dph_to_point_cloud(dph_path)
            x, y = np.meshgrid(np.arange(depth_data.shape[1]), np.arange(depth_data.shape[0]))
            z = depth_data
            x_transformed, y_transformed, z_transformed = apply_q_matrix(x, y, z, q_matrix)
            
            output_file_name = file_name.replace('.dph', '.ply')
            output_path = os.path.join(output_folder, output_file_name)
            save_point_cloud_to_ply(x_transformed, y_transformed, z_transformed, output_path)
            print(f"Saved: {output_path}")

# Main script execution
if __name__ == "__main__":
    script_dir = os.path.dirname(os.path.abspath(__file__))  # Automatically detect the current working directory
    cache_folder = os.path.join(script_dir, 'cache')
    q_bin_path = os.path.join(script_dir, 'param', 'Q.bin')
    output_folder = os.path.join(script_dir, 'pointclouds')
    
    # Read the Q matrix from Q.bin
    q_matrix = read_q_matrix(q_bin_path)
    
    # Process all frames and save them as .ply files
    process_and_save_all_frames(cache_folder, q_matrix, output_folder)
```

### Running the Script
```bash
python convert.py
```

The script will generate `.ply` files for each frame and save them in the `pointclouds` folder. 


