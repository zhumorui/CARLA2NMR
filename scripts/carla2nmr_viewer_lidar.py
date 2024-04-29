import os
import cv2
import sys
import argparse
import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation as R
from src.utils.colmap.python.read_write_model import read_model, write_model, qvec2rotmat, rotmat2qvec


class Model:
    def __init__(self):
        self.cameras = []
        self.images = []
        self.points3D = []
        self.__vis = None

    def read_model(self, path, ext=""):
        path = os.path.join(path, "sparse/0")
        self.cameras, self.images, self.points3D = read_model(path, ext)
    
    def lidar_pcd(self, path, transform=None):
        pcd = o3d.io.read_point_cloud(path)

        # reverse y
        T = np.array([[1, 0, 0, 0],
                      [0, -1, 0, 0],
                      [0, 0, 1, 0],
                      [0, 0, 0, 1]])
        pcd.transform(T)



        # (x, y, z) -> (z, -x, -y) coordinate transformation
        T = np.array([[0, -1, 0, 0],          
                      [0, 0, -1, 0],
                      [1, 0, 0, 0],
                      [0, 0, 0, 1]]
            )

        pcd.transform(T)

        # apply world to camera transformation
        transform = np.array([[-9.99967710e-01, -3.51656027e-05, 8.03600210e-03, -1.90825958e+01],
                                [-1.74406981e-05, 9.99997567e-01, 2.20574634e-03, -2.11552334e+00],
                                [-8.03606011e-03, 2.20553496e-03, -9.99965278e-01, -2.84187126e+00],
                                [0, 0, 0, 1]])
        
        pcd.transform(transform)

        self.__vis.add_geometry(pcd)
        self.__vis.poll_events()
        self.__vis.update_renderer()
    
    def add_coordinates(self):
        mesh = o3d.geometry.TriangleMesh.create_coordinate_frame(size=10.0)

        self.__vis.add_geometry(mesh)
        self.__vis.poll_events()
        self.__vis.update_renderer()

    def add_points(self, min_track_len=3, remove_statistical_outlier=True):
        pcd = o3d.geometry.PointCloud()

        xyz = []
        rgb = []
        for point3D in self.points3D.values():
            track_len = len(point3D.point2D_idxs)
            if track_len < min_track_len:
                continue
            xyz.append(point3D.xyz)
            rgb.append(point3D.rgb / 255)

        pcd.points = o3d.utility.Vector3dVector(xyz)
        pcd.colors = o3d.utility.Vector3dVector(rgb)

        # remove obvious outliers
        if remove_statistical_outlier:
            [pcd, _] = pcd.remove_statistical_outlier(
                nb_neighbors=20, std_ratio=2.0
            )

        # o3d.visualization.draw_geometries([pcd])
        self.__vis.add_geometry(pcd)
        self.__vis.poll_events()
        self.__vis.update_renderer()

    def add_pcd(self, path):
        pcd = o3d.io.read_point_cloud(path)
        self.__vis.add_geometry(pcd)
        self.__vis.poll_events()
        self.__vis.update_renderer()

    def add_cameras(self, path, scale=1):
        frames = []
        for img in self.images.values():
            # rotation
            R = qvec2rotmat(img.qvec)


            # imgae file
            img_path = os.path.join(path, 'images', img.name)

            # translation
            t = img.tvec

            # invert
            t = -R.T @ t
            R = R.T

            # intrinsics
            cam = self.cameras[img.camera_id]

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

            # intrinsics
            K = np.identity(3)
            K[0, 0] = fx
            K[1, 1] = fy
            K[0, 2] = cx
            K[1, 2] = cy

            # create axis, plane and pyramed geometries that will be drawn
            cam_model = draw_camera(K, R, t, cam.width, cam.height, scale, img_path=img_path)
            frames.extend(cam_model)

        # add geometries to visualizer
        for i in frames:
            self.__vis.add_geometry(i)

    def create_window(self):
        self.__vis = o3d.visualization.Visualizer()
        self.__vis.create_window()
        self.__vis.get_render_option().mesh_show_back_face = True

    def show(self):
        self.__vis.poll_events()
        self.__vis.update_renderer()
        self.__vis.run()
        self.__vis.destroy_window()


def draw_camera(K, R, t, w, h, scale=1, color=[0.8, 0.2, 0.8], img_path=None):
    """Create axis, plane and pyramed geometries in o3d format.
    :param K: calibration matrix (camera intrinsics)
    :param R: rotation matrix
    :param t: translation
    :param w: image width
    :param h: image height
    :param scale: camera model scale
    :param color: color of the image plane and pyramid lines
    :param img_path: path to the image file
    :return: camera model geometries (axis, plane and pyramid)
    """

    # intrinsics
    K = K.copy() / scale
    Kinv = np.linalg.inv(K)

    # 4x4 transformation
    T = np.column_stack((R, t))
    T = np.vstack((T, (0, 0, 0, 1)))

    # axis
    axis = o3d.geometry.TriangleMesh.create_coordinate_frame(
        size=0.5 * scale
    )
    
    axis.transform(T)

    # points in pixel
    points_pixel = [
        [0, 0, 0],
        [0, 0, 1],
        [w, 0, 1],
        [0, h, 1],
        [w, h, 1],
    ]

    # pixel to camera coordinate system
    points = [Kinv @ p for p in points_pixel]


    
    img = cv2.cvtColor(cv2.imread(img_path), cv2.COLOR_BGR2RGB)

    # image plane
    width = abs(points[1][0]) + abs(points[3][0])
    height = abs(points[1][1]) + abs(points[3][1])

    w, h, = width, height

    # Normalize the vertex positions by the maximum dimension to maintain aspect ratio
    vertices = np.array([
        [0, 0, 0],   # Bottom-left
        [w, 0, 0],   # Bottom-right
        [w, h, 0],   # Top-right
        [0, h, 0]    # Top-left
    ])  # Normalize to keep aspect ratio

    faces = np.array([
        [0, 1, 2],  # Lower triangle
        [2, 3, 0],  # Upper triangle
    ])
    
    # Set UV coordinates
    v_uv = np.array([
        [0, 0],  # UV for Top-left
        [1, 0],  # UV for Top-right
        [1, 1],  # UV for Bottom-right
        [0, 1]   # UV for Bottom-left
    ])


    # Correct faces' UV indexing, each face needs 3 UV coordinates, one for each vertex
    face_uvs = np.array([
        [0, 1, 2],  # UVs for the first triangle
        [2, 3, 0]   # UVs for the second triangle
    ])

    # Create the mesh
    mesh = o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(vertices), o3d.utility.Vector3iVector(faces))
    mesh.textures = [o3d.geometry.Image(img)]
    mesh.triangle_uvs = o3d.utility.Vector2dVector(v_uv[face_uvs.flatten()])
    mesh.triangle_material_ids = o3d.utility.IntVector([0] * len(faces))
        
    mesh.translate([points[1][0], points[1][1], scale])
    mesh.transform(T)


    # pyramid
    points_in_world = [(R @ p + t) for p in points]
    lines = [
        [0, 1],
        [0, 2],
        [0, 3],
        [0, 4],
    ]
    colors = [color for i in range(len(lines))]
    line_set = o3d.geometry.LineSet(
        points=o3d.utility.Vector3dVector(points_in_world),
        lines=o3d.utility.Vector2iVector(lines),
    )
    line_set.colors = o3d.utility.Vector3dVector(colors)

    # return as list in o3d format
    return [axis, mesh, line_set]


def parse_args():
    parser = argparse.ArgumentParser(
        description="Visualize COLMAP binary and text models"
    )
    parser.add_argument(
        "--input_model", default='/home/morin/ZHITAI_SSD/projects/monopointcloud/_out/sequences/04_23_15_38_23/carla_12_20_rgb_1_1/', help="path to input model folder"
    )
    parser.add_argument(
        "--input_format",
        choices=[".bin", ".txt"],
        help="input model format",
        default="",
    )
    args = parser.parse_args()
    return args


def main():
    args = parse_args()

    # read COLMAP model
    model = Model()
    model.read_model(args.input_model, ext=args.input_format)

    print("num_cameras:", len(model.cameras))
    print("num_images:", len(model.images))
    print("num_points3D:", len(model.points3D))

    # display using o3d visualization tools
    model.create_window()
    model.add_points()
    model.add_cameras(args.input_model, scale=1)


    # load local lidar point cloud
    lidar_pcd_path = ("/home/morin/ZHITAI_SSD/projects/monopointcloud/_out/sequences/04_23_15_38_23/carla_12_20_rgb_1_1/lidar/0150.ply")
    model.lidar_pcd(lidar_pcd_path)
    # add coordinate frame
    # model.add_coordinates()
    
    model.show()
if __name__ == "__main__":
    main()
