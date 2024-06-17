""" This module includes the Mapper class, which is responsible scene mapping: Paragraph 3.2  """
import time
from argparse import ArgumentParser

import cv2
import numpy as np
import torch
import torchvision

from src.GS_SLAM.entities.arguments import OptimizationParams
from src.GS_SLAM.entities.datasets import TUM_RGBD, BaseDataset, ScanNet
from src.GS_SLAM.entities.gaussian_model import GaussianModel
from src.GS_SLAM.entities.logger import Logger
from src.GS_SLAM.entities.losses import isotropic_loss, l1_loss, ssim
from src.GS_SLAM.utils.mapper_utils import (calc_psnr, compute_camera_frustum_corners,
                                    compute_frustum_point_ids,
                                    compute_new_points_ids,
                                    compute_opt_views_distribution,
                                    create_point_cloud, geometric_edge_mask,
                                    sample_pixels_based_on_gradient)
from src.GS_SLAM.utils.utils import (get_render_settings, np2ptcloud, np2torch,
                             render_gaussian_model, torch2np)
from src.GS_SLAM.utils.vis_utils import *  # noqa - needed for debugging


class Mapper(object):
    def __init__(self, config: dict, dataset: BaseDataset, logger: Logger) -> None:
        """ Sets up the mapper parameters
        Args:
            config: configuration of the mapper
            dataset: The dataset object used for extracting camera parameters and reading the data
            logger: The logger object used for logging the mapping process and saving visualizations
        """
        self.config = config
        self.logger = logger
        self.dataset = dataset
        self.iterations = config["iterations"]
        self.new_submap_iterations = config["new_submap_iterations"]
        self.new_submap_points_num = config["new_submap_points_num"]
        self.new_submap_gradient_points_num = config["new_submap_gradient_points_num"]
        self.new_frame_sample_size = config["new_frame_sample_size"]
        self.new_points_radius = config["new_points_radius"]
        self.alpha_thre = config["alpha_thre"]
        self.pruning_thre = config["pruning_thre"]
        self.current_view_opt_iterations = config["current_view_opt_iterations"]
        self.opt = OptimizationParams(ArgumentParser(description="Training script parameters"))
        self.keyframes = []


    def seed_new_gaussians_with_lidar(self, lidar_point_cloud: o3d.geometry.PointCloud) -> np.ndarray:
        """
        Seeds means for the new 3D Gaussian based on a LiDAR point cloud with color information.
        Args:
            lidar_point_cloud: The input Open3D point cloud with color information.
        Returns:
            np.ndarray: An array of 3D points where new Gaussians will be initialized, with shape (N, 6),
                        where each point contains XYZ and RGB information.
        """
        # Convert the Open3D point cloud to NumPy arrays
        points = np.asarray(lidar_point_cloud.points)
        colors = np.asarray(lidar_point_cloud.colors)

        # Combine points and colors
        pts_with_colors = np.hstack((points, colors))

        return pts_with_colors.astype(np.float32)


    def optimize_submap(self, keyframes: list, gaussian_model: GaussianModel, iterations: int = 100) -> dict:
        """
        Optimizes the submap by refining the parameters of the 3D Gaussian based on the observations
        from keyframes observing the submap.
        Args:
            keyframes: A list of tuples consisting of frame id and keyframe dictionary
            gaussian_model: An instance of the GaussianModel class representing the initial state
                of the Gaussian model to be optimized.
            iterations: The number of iterations to perform the optimization process. Defaults to 100.
        Returns:
            losses_dict: Dictionary with the optimization statistics
        """

        iteration = 0
        losses_dict = {}

        current_frame_iters = self.current_view_opt_iterations * iterations
        distribution = compute_opt_views_distribution(len(keyframes), iterations, current_frame_iters)
        start_time = time.time()
        while iteration < iterations + 1:
            gaussian_model.optimizer.zero_grad(set_to_none=True)
            keyframe_id = np.random.choice(np.arange(len(keyframes)), p=distribution)

            frame_id, keyframe = keyframes[keyframe_id]
            render_pkg = render_gaussian_model(gaussian_model, keyframe["render_settings"])

            image, depth = render_pkg["color"], render_pkg["depth"]
            gt_image = keyframe["color"]
            # gt_depth = keyframe["depth"]

            # mask = (gt_depth > 0) & (~torch.isnan(depth)).squeeze(0)
            # color_loss = (1.0 - self.opt.lambda_dssim) * l1_loss(
            #     image[:, mask], gt_image[:, mask]) + self.opt.lambda_dssim * (1.0 - ssim(image, gt_image))

            color_loss = (1.0 - self.opt.lambda_dssim) * l1_loss(
                        image, gt_image) + self.opt.lambda_dssim * (1.0 - ssim(image, gt_image))

            # depth_loss = l1_loss(depth[:, mask], gt_depth[mask])
            reg_loss = isotropic_loss(gaussian_model.get_scaling())
            total_loss = color_loss + reg_loss
            total_loss.backward()

            losses_dict[frame_id] = {"color_loss": color_loss.item(),
                                     "total_loss": total_loss.item()}

            with torch.no_grad():

                if iteration == iterations // 2 or iteration == iterations:
                    prune_mask = (gaussian_model.get_opacity()
                                  < self.pruning_thre).squeeze()
                    gaussian_model.prune_points(prune_mask)

                # Optimizer step
                if iteration < iterations:
                    gaussian_model.optimizer.step()
                gaussian_model.optimizer.zero_grad(set_to_none=True)

            iteration += 1
        optimization_time = time.time() - start_time
        losses_dict["optimization_time"] = optimization_time
        losses_dict["optimization_iter_time"] = optimization_time / iterations
        return losses_dict

    def grow_submap(self, gaussian_model: GaussianModel, pts: np.ndarray, use_color: bool = True) -> int:
        """
        Expands the submap by integrating new points from the current keyframe
        Args:
            gaussian_model (GaussianModel): The Gaussian model representing the current state of the submap.
            pts: The current set of 3D points in the keyframe of shape (N, 6), where each point contains XYZ and RGB information.
            use_color: A boolean flag indicating whether to use color information.
        Returns:
            int: The number of points added to the submap
        """
        if use_color:
            # 将新点云的XYZ和RGB信息合并
            cloud_to_add = np2ptcloud(pts[:, :3], pts[:, 3:] / 255.0)
        else:
            # 仅使用XYZ信息
            cloud_to_add = np2ptcloud(pts[:, :3])
        
        # 将新点云添加到高斯模型中
        gaussian_model.add_points(cloud_to_add)
        
        # 设置模型中的特征不需要计算梯度
        gaussian_model._features_dc.requires_grad = False
        gaussian_model._features_rest.requires_grad = False
        
        # 打印高斯模型的大小
        print("Gaussian model size", gaussian_model.get_size())
        
        return pts.shape[0]


    def map(self, frame_id: int, estimate_c2w: np.ndarray, gaussian_model: GaussianModel, is_new_submap: bool) -> dict:
        """ Calls out the mapping process described in paragraph 3.2
        The process goes as follows: seed new gaussians -> add to the submap -> optimize the submap
        Args:
            frame_id: current keyframe id
            estimate_c2w (np.ndarray): The estimated camera-to-world transformation matrix of shape (4x4)
            gaussian_model (GaussianModel): The current Gaussian model of the submap
            is_new_submap (bool): A boolean flag indicating whether the current frame initiates a new submap
        Returns:
            opt_dict: Dictionary with statistics about the optimization process
        """
        _, gt_color, gt_lidar = self.dataset[frame_id]
        estimate_w2c = np.linalg.inv(estimate_c2w)

        # get dataset image width and height
        self.dataset.width, self.dataset.height = gt_color.shape[1], gt_color.shape[0]

        color_transform = torchvision.transforms.ToTensor()
        keyframe = {
            "color": color_transform(gt_color).cuda(),
            "render_settings": get_render_settings(
                self.dataset.width, self.dataset.height, self.dataset.get_intrinsics(frame_id), estimate_w2c)
        }

        # Directly use the lidar point cloud for seeding
        pts = self.seed_new_gaussians_with_lidar(gt_lidar)

        filter_cloud = isinstance(self.dataset, (TUM_RGBD, ScanNet)) and not is_new_submap

        new_pts_num = self.grow_submap(gaussian_model, pts, use_color=True)

        max_iterations = self.iterations
        if is_new_submap:
            max_iterations = self.new_submap_iterations
        start_time = time.time()
        opt_dict = self.optimize_submap([(frame_id, keyframe)] + self.keyframes, gaussian_model, max_iterations)
        optimization_time = time.time() - start_time
        print("Optimization time: ", optimization_time)

        self.keyframes.append((frame_id, keyframe))

        # Visualise the mapping for the current frame
        with torch.no_grad():
            render_pkg_vis = render_gaussian_model(gaussian_model, keyframe["render_settings"])
            image_vis, depth_vis = render_pkg_vis["color"], render_pkg_vis["depth"]
            psnr_value = calc_psnr(image_vis, keyframe["color"]).mean().item()
            opt_dict["psnr_render"] = psnr_value
            print(f"PSNR this frame: {psnr_value}")
            # self.logger.vis_mapping_iteration(
            #     frame_id, max_iterations,
            #     image_vis.clone().detach().permute(1, 2, 0),
            #     depth_vis.clone().detach().permute(1, 2, 0),
            #     keyframe["color"].permute(1, 2, 0),
            #     keyframe["color"].unsqueeze(-1),
            #     seeding_mask=None)

        # # Log the mapping numbers for the current frame
        # self.logger.log_mapping_iteration(frame_id, new_pts_num, gaussian_model.get_size(),
        #                                 optimization_time / max_iterations, opt_dict)


        # visualize the image
        img = image_vis.clone().detach().permute(1, 2, 0).cpu().numpy()
        img = (img * 255).astype(np.uint8)

        cv2.imshow("image", img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

        return opt_dict
