import open3d as o3d
from src.utils.colmap.python.read_write_model import qvec2rotmat
import numpy as np
import cv2

# Filter lidar point cloud within camera view
def filter_point_cloud(point_cloud, cam, pose):
    """
    cam (Camera Object):
        Camera(id=1, model='SIMPLE_PINHOLE', width=1280, height=720, params=array([369.50417228, 640. , 360. ]))

    pose:
        (img.qvec, img.tvec, img.camera_id)
    """
    width = cam.width
    height = cam.height

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

    R = qvec2rotmat(pose[0])
    t = pose[1]

    # invert
    t = -R.T @ t
    R = R.T

    # intrinsics
    K = np.identity(3)
    K[0, 0] = fx
    K[1, 1] = fy
    K[0, 2] = cx
    K[1, 2] = cy

    points_2d, _ = cv2.projectPoints(np.array(point_cloud.points), R, t, K, None)

    print(points_2d)

    # camera = o3d.camera.PinholeCameraIntrinsic(width, height, fx, fy, cx, cy)
    # camera_extrinsic = o3d.camera.PinholeCameraParameters()
    # camera_extrinsic.extrinsic = T
    # camera.set_parameters(camera_extrinsic)
    # frustum = camera.get_view_frustum()
    # frustum_mesh = o3d.geometry.LineSet.create_from_frustum(frustum)
    # frustum_mesh.paint_uniform_color([1, 0, 0])
    # points_in_frustum = point_cloud.select_by_index(
    #     frustum_mesh, invert=False)
    # return points_in_frustum
