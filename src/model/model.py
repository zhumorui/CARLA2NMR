import os
import cv2
from glob import glob
import argparse
import numpy as np
import open3d as o3d
from src.utils.colmap.python.read_write_model import read_model, write_model, qvec2rotmat, rotmat2qvec
from src.utils.vis_utils import draw_camera
from src.utils.pcd_utils import filter_point_cloud_with_image

class Model:
    def __init__(self,model_path, ext=None):
        self.cameras = []
        self.poses = []
        self.images = []
        self.points3D = []
        self.masks = []
        self.depth16 = []
        self.lidars = []

        self.read_model(model_path, ext)

    def __str__(self):
        model_info = f"num_cameras: {len(self.cameras)}\n"
        model_info += f"num_poses: {len(self.poses)}\n"
        model_info += f"num_images: {len(self.images)}\n"
        model_info += f"num_points3D: {len(self.points3D)}\n"
        model_info += f"num_masks: {len(self.masks)}\n"
        model_info += f"num_depth16: {len(self.depth16)}\n"
        model_info += f"num_lidars: {len(self.lidars)}\n"

        return model_info


    def read_model(self, path, ext=".txt"):
        self.cameras, images_info, self.points3D = read_model(os.path.join(path, "sparse/0"), ext)
        self.images = [os.path.join(path, "images", img.name) for img in images_info.values()]
        self.poses = [(img.qvec, img.tvec, img.camera_id) for img in images_info.values()]
        self.masks = sorted(glob(os.path.join(path, "mask", "*.png")))
        self.depth16 = sorted(glob(os.path.join(path, "depth_16", "*.png")))
        self.lidars = sorted(glob(os.path.join(path, "lidar", "*.ply")))

    # add camera trajectories, add image textures if available
    def add_cameras(self, scale=1):
        cams_axis =[]
        cams_mesh = []
        cams_line_set = []
        for img, pose in zip(self.images, self.poses):
            # rotation
            R = qvec2rotmat(pose[0])

            # translation
            t = pose[1]

            # invert
            t = -R.T @ t
            R = R.T

            # intrinsics
            cam = self.cameras[pose[2]]

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
            axis, mesh, line_set = draw_camera(K, R, t, cam.width, cam.height, scale, img_path=img)
            cams_axis.append(axis)
            cams_mesh.append(mesh)
            cams_line_set.append(line_set)
        
        return cams_axis, cams_mesh, cams_line_set
    
    def add_lidar(self, idx):
        if idx is None:
            idx = 0


        #  completed lidar point cloud (unfiltered) 

        # pcd = o3d.io.read_point_cloud(self.lidars[idx])

        # # reverse y
        # T = np.array([[1, 0, 0, 0],
        #               [0, -1, 0, 0],
        #               [0, 0, 1, 0],
        #               [0, 0, 0, 1]])
        # pcd.transform(T)

        # # (x, y, z) -> (z, -x, -y) coordinate transformation
        # T = np.array([[0, -1, 0, 0],          
        #               [0, 0, -1, 0],
        #               [1, 0, 0, 0],
        #               [0, 0, 0, 1]]
        #     )

        # pcd.transform(T)


        # # apply w2c transformation
        # T = self.get_transform(idx)
        # pcd.transform(T)

        if True:
            filtered_pcd = filter_point_cloud_with_image(self.lidars[idx], self.images[idx], self.cameras[self.poses[idx][2]])
            T = self.get_transform(idx)
            filtered_pcd.transform(T)
        return filtered_pcd

    def get_transform(self, idx):
        pose = self.poses[idx]
        R = qvec2rotmat(pose[0])
        t = pose[1]

        # invert
        t = -R.T @ t
        R = R.T

        
        # 4x4 transformation
        T = np.column_stack((R, t))
        T = np.vstack((T, (0, 0, 0, 1)))

        return T

    # get camera intrinsics
    def get_intrinsics(self, idx):
        cam = self.cameras[self.poses[idx][2]]

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

        return K
    

    def __getitem__(self, index):
        color_data = cv2.imread(self.images[index])
        color_data = cv2.cvtColor(color_data, cv2.COLOR_BGR2RGB)

        mask_data = cv2.imread(self.masks[index])

        # apply mask
        masked_color_data = cv2.bitwise_and(color_data, mask_data * 255)

        lidar = self.add_lidar(index)

        return index, masked_color_data, lidar
    

    def __len__(self):
        return len(self.images)
    

if __name__ == "__main__":
    model = Model('/Users/morin/carla_12_20_rgb_1_1/', ext=".txt")
    cameras = model.add_cameras()
    print(model)
    

