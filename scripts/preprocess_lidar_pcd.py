import os
import numpy as np
import open3d as o3d
from tqdm import tqdm

def apply_transformations(ply_file_path, transformations):
    # Load the point cloud from the PLY file
    pcd = o3d.io.read_point_cloud(ply_file_path)

    # Convert the point cloud to a numpy array
    points = np.asarray(pcd.points)

    # Convert points to homogeneous coordinates
    points_h = np.hstack((points, np.ones((points.shape[0], 1))))

    # Apply each transformation in sequence
    for T in transformations:
        points_h = points_h @ T.T

    # Convert back to non-homogeneous coordinates
    points_transformed = points_h[:, :3]

    # Create a new point cloud from the transformed points
    pcd_transformed = o3d.geometry.PointCloud()
    pcd_transformed.points = o3d.utility.Vector3dVector(points_transformed)

    return pcd_transformed

def process_ply_files(input_folder, output_folder):

    # Define the custom transformations
    transformations = [
        np.array([[1, 0, 0, 0],
                  [0, -1, 0, 0],
                  [0, 0, 1, 0],
                  [0, 0, 0, 1]]),
        np.array([[0, -1, 0, 0],
                  [0, 0, -1, 0],
                  [1, 0, 0, 0],
                  [0, 0, 0, 1]])
    ]

    # Ensure the output folder exists
    os.makedirs(output_folder, exist_ok=True)

    # Process each PLY file in the input folder
    for filename in tqdm(os.listdir(input_folder), desc="Preprocessing PLY files"):
        if filename.lower().endswith('.ply'):
            input_path = os.path.join(input_folder, filename)
            output_path = os.path.join(output_folder, filename)

            # Apply transformations
            transformed_pcd = apply_transformations(input_path, transformations)

            # Save the transformed point cloud to the output folder
            o3d.io.write_point_cloud(output_path, transformed_pcd)

if __name__ == "__main__":
    input_folder = 'data/carla_12_20_rgb_2_1/lidar'  
    output_folder = 'data/carla_12_20_rgb_2_1/aligned_lidar'
    process_ply_files(input_folder, output_folder)
