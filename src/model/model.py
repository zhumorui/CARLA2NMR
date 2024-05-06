import os
import cv2
from glob import glob
import argparse
import numpy as np
import open3d as o3d
from src.utils.colmap.python.read_write_model import read_model, write_model, qvec2rotmat, rotmat2qvec
from src.utils.vis_utils import draw_camera

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
        frames = []
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
            cam_model = draw_camera(K, R, t, cam.width, cam.height, scale, img_path=img)
            frames.extend(cam_model)
        
        return frames
    
if __name__ == "__main__":
    model = Model('/Users/morin/carla_12_20_rgb_1_1/', ext=".txt")
    cameras = model.add_cameras()
    print(model)
    

