import open3d as o3d
import cv2
import numpy as np



def filter_point_cloud_with_image(pcd_path, image_path, camera):
    # Load the image
    img = cv2.imread(image_path)
    img_height, img_width, _ = img.shape

    # Load the point cloud
    pcd = o3d.io.read_point_cloud(pcd_path)

    # Optional: Perform necessary point cloud transformation if needed
    # Reverse y
    T = np.array([[1, 0, 0, 0],
                  [0, -1, 0, 0],
                  [0, 0, 1, 0],
                  [0, 0, 0, 1]])
    pcd.transform(T)

    # (x, y, z) -> (z, -x, -y) coordinate transformation
    T = np.array([[0, -1, 0, 0],
                  [0, 0, -1, 0],
                  [1, 0, 0, 0],
                  [0, 0, 0, 1]])
    pcd.transform(T)

    # Get the camera intrinsics
    cam = camera

    if cam.model in ("SIMPLE_PINHOLE", "SIMPLE_RADIAL", "RADIAL"):
        fx = fy = cam.params[0]
        cx = cam.params[1]
        cy = cam.params[2]
    elif cam.model in (
        "PINHOLE",
        "OPENCV",
        "OPENCV_FISHEYE",
        "FULL_OPENCV",
    ):
        fx = cam.params[0]
        fy = cam.params[1]
        cx = cam.params[2]
        cy = cam.params[3]
    else:
        raise Exception("Camera model not supported")

    # Intrinsics
    K = np.identity(3)
    K[0, 0] = float(fx)
    K[1, 1] = float(fy)
    K[0, 2] = float(cx)
    K[1, 2] = float(cy)

    # Project point cloud to image plane and filter points
    points = np.asarray(pcd.points)

    # No need to apply transformation if points are already in camera coordinates
    points_cam = points

    # Filter points behind the camera
    mask = points_cam[:, 2] > 0
    points_cam = points_cam[mask]
    points = points[mask]

    # Project to image plane
    points_img = points_cam @ K.T
    points_img /= points_img[:, 2].reshape(-1, 1)
    points_img = points_img[:, :2]

    # Filter points within the image boundaries
    mask = (points_img[:, 0] >= 0) & (points_img[:, 0] < img_width) & \
           (points_img[:, 1] >= 0) & (points_img[:, 1] < img_height)

    filtered_points = points[mask]
    filtered_points_img = points_img[mask].astype(int)

    # Ensure filtered points' image coordinates are within the image boundaries
    filtered_points_img[:, 0] = np.clip(filtered_points_img[:, 0], 0, img_width - 1)
    filtered_points_img[:, 1] = np.clip(filtered_points_img[:, 1], 0, img_height - 1)

    # Assign RGB values from the image to the point cloud
    filtered_colors = img[filtered_points_img[:, 1], filtered_points_img[:, 0]] / 255.0

    # Update point cloud with filtered points and colors
    pcd_filtered = o3d.geometry.PointCloud()
    pcd_filtered.points = o3d.utility.Vector3dVector(filtered_points)
    pcd_filtered.colors = o3d.utility.Vector3dVector(filtered_colors)

    return pcd_filtered
