import os
import cv2
from glob import glob
import argparse
import numpy as np
import open3d as o3d
from src.utils.colmap.python.read_write_model import read_model, write_model, qvec2rotmat, rotmat2qvec


class Model:
    def __init__(self,model_path, ext=None):
        self.cameras = []
        self.images = []
        self.points3D = []
        self.masks = []
        self.depth16 = []
        self.lidars = []

        self.read_model(model_path, ext)

    def __str__(self):
        model_info = f"num_cameras: {len(self.cameras)}\n"
        model_info += f"num_images: {len(self.images)}\n"
        model_info += f"num_points3D: {len(self.points3D)}\n"
        model_info += f"num_masks: {len(self.masks)}\n"
        model_info += f"num_depth16: {len(self.depth16)}\n"
        model_info += f"num_lidars: {len(self.lidars)}\n"

        return model_info


    def read_model(self, path, ext=".txt"):
        self.cameras, self.images, self.points3D = read_model(os.path.join(path, "sparse/0"), ext)
        self.masks = sorted(glob(os.path.join(path, "mask", "*.png")))
        self.depth16 = sorted(glob(os.path.join(path, "depth_16", "*.png")))
        self.lidars = sorted(glob(os.path.join(path, "lidar", "*.ply")))


if __name__ == "__main__":
    model = Model('/home/morin/ZHITAI_SSD/projects/monopointcloud/_out/sequences/04_23_15_38_23/carla_12_20_rgb_1_1/', ext=".txt")
    print(model)

