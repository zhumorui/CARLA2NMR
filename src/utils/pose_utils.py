import numpy as np
from scipy.spatial.transform import Rotation as R


def kitti_to_colmap(kitti_file_path, colmap_file_path):
    # Read KITTI formatted data
    with open(kitti_file_path, 'r') as f:
        kitti_data = f.readlines()

    kitti_data = [line.strip().split() for line in kitti_data]

    # Function: Extract rotation matrix and translation vector from KITTI data and convert rotation matrix to quaternion
    def process_kitti_data(kitti_data):
        colmap_data = []
        for i, kitti_line in enumerate(kitti_data):
            rotation_matrix = np.array(kitti_line[:9]).reshape(3, 3)
            translation_vector = np.array(kitti_line[9:]).reshape(3)
            
            # Convert rotation matrix to quaternion
            rotation = R.from_matrix(rotation_matrix)
            quaternion = rotation.as_quat()  # [qx, qy, qz, qw]
            
            # COLMAP requires the format [qw, qx, qy, qz]
            quaternion = np.roll(quaternion, 1)
            
            # Add data to the result
            image_id = i + 1
            camera_id = 1  # Assume camera ID is 1
            image_name = f"{i+1:04d}.png"  # Image names from 0001.png to 0150.png
            colmap_data.append([
                image_id,
                quaternion[0], quaternion[1], quaternion[2], quaternion[3],
                translation_vector[0], translation_vector[1], translation_vector[2],
                camera_id,
                image_name
            ])
        return colmap_data

    # Generate COLMAP formatted data
    colmap_data = process_kitti_data(kitti_data)

    # Save COLMAP formatted data
    with open(colmap_file_path, 'w') as f:
        for line in colmap_data:
            f.write(' '.join(map(str, line)) + '\n')
            # Add blank line
            f.write('\n')