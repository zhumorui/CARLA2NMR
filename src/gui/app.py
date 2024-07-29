import open3d as o3d
import open3d.visualization.gui as gui  # type: ignore
import open3d.visualization.rendering as rendering  # type: ignore
import platform
import threading
import os
import numpy as np
from src.model.model import Model
from src.utils.colmap.python.read_write_model import rotmat2qvec, qvec2rotmat

isMacOS = (platform.system() == "Darwin")

class CARLA2NMR_App:
    MENU_QUIT = 1
    MENU_LOAD_FILE = 2
    MENU_SHOW_CAMERA = 3
    MENU_SHOW_LIDAR = 4
    MENU_SHOW_SETTING = 5

    def __init__(self):
        self.working_dir = os.getcwd()
        self.model = None
        self.window = gui.Application.instance.create_window(
            "CARLA2NMR Viewer", 1280, 720)
        self.scene = gui.SceneWidget()
        self.scene.scene = rendering.Open3DScene(self.window.renderer)

        # Disable lighting to avoid reflections
        self.scene.scene.scene.set_sun_light([0, 0, 0], [0, 0, 0], 0)
        self.scene.scene.scene.enable_sun_light(False)

        self.window.add_child(self.scene)

        # set lidar show
        self.lidar_shown = False

        # set camera show
        self.camera_shown = False
        self.camera_params = None

        # The menu is global (because the macOS menu is global), so only create
        # it once, no matter how many windows are created
        if gui.Application.instance.menubar is None:
            if isMacOS:
                app_menu = gui.Menu()
                app_menu.add_item("Quit", CARLA2NMR_App.MENU_QUIT)
            file_menu = gui.Menu()
            file_menu.add_item("Open Dir...", CARLA2NMR_App.MENU_LOAD_FILE)
            view_menu = gui.Menu()
            view_menu.add_item("Show Camera", CARLA2NMR_App.MENU_SHOW_CAMERA)
            view_menu.add_item("Show Lidar", CARLA2NMR_App.MENU_SHOW_LIDAR)
            actions_menu = gui.Menu()
            actions_menu.add_item("Show Settings", CARLA2NMR_App.MENU_SHOW_SETTING)
            if not isMacOS:
                file_menu.add_separator()
                file_menu.add_item("Quit", CARLA2NMR_App.MENU_QUIT)

            menu = gui.Menu()
            if isMacOS:
                # macOS will name the first menu item for the running application
                # (in our case, probably "Python"), regardless of what we call
                # it. This is the application menu, and it is where the
                # About..., Preferences..., and Quit menu items typically go.
                menu.add_menu("Python", app_menu) 
                menu.add_menu("File", file_menu)
                menu.add_menu("View", view_menu)
            else:
                menu.add_menu("File", file_menu)
                menu.add_menu("View", view_menu)
            gui.Application.instance.menubar = menu

        # The menubar is global, but we need to connect the menu items to the
        # window, so that the window can call the appropriate function when the
        # menu item is activated.
        self.window.set_on_menu_item_activated(CARLA2NMR_App.MENU_QUIT,
                                               self._on_menu_quit)
        self.window.set_on_menu_item_activated(CARLA2NMR_App.MENU_LOAD_FILE,
                                               self._on_menu_filedlg)
        self.window.set_on_menu_item_activated(CARLA2NMR_App.MENU_SHOW_CAMERA,
                                                self._on_menu_show_camera)
        self.window.set_on_menu_item_activated(CARLA2NMR_App.MENU_SHOW_LIDAR,
                                                self._on_menu_show_lidar)
        self.window.set_on_menu_item_activated(CARLA2NMR_App.MENU_SHOW_SETTING,
                                                self._on_menu_show_settings)

        # Set widgets
        em = self.window.theme.font_size
        self._pannel = gui.Vert(0, gui.Margins(0.25*em,0.25*em,0.25*em,0.254*em))

        
        # Set Slider 
        slider = gui.Slider(gui.Slider.INT)
        slider.set_limits(0, 149)
        slider.set_on_value_changed(self._on_slider)
        self._pannel.add_child(slider)

        # Set default camera view button        
        self._button = gui.Button("Set Default Camera View")
        self._button.set_on_clicked(self._set_default_camera_view)
        self._pannel.add_child(self._button)

        # Set Gaussian SLAM button
        self._button = gui.Button("Run Gaussian SLAM")
        self._button.set_on_clicked(self._run_gaussian_slam)
        self._pannel.add_child(self._button)

        # Set Kiss-ICP button
        self._button = gui.Button("Run Lidar Odometry")
        self._button.set_on_clicked(self._run_kiss_icp)
        self._pannel.add_child(self._button)

        # Set gsplat training button
        self._button = gui.Button("Run gsplat Training")
        self._button.set_on_clicked(self._run_gsplat_training)
        self._pannel.add_child(self._button)


        # 布局回调函数
        self.window.set_on_layout(self._on_layout)
        self.window.add_child(self._pannel)

    def _on_layout(self, layout_context):
        #   在on_layout回调函数中应正确设置所有子对象的框架(position + size)，
        #   回调结束之后才会布局孙子对象。

        r = self.window.content_rect
        self.scene.frame = r

        pannel_width = 17*layout_context.theme.font_size
        pannel_height = min(
            r.height, self._pannel.calc_preferred_size(
                layout_context, gui.Widget.Constraints()).height
        )
        self._pannel.frame = gui.Rect(r.get_right()-pannel_width,r.y,pannel_width,pannel_height)

        button_pref = self._button.calc_preferred_size(
            layout_context, gui.Widget.Constraints())
        self._button.frame = gui.Rect(r.x,r.get_bottom()-button_pref.height, button_pref.width,button_pref.height)
        print(r.x,r.y)


    def _on_slider(self, new_val):
        self._on_menu_show_lidar(int(new_val), if_slider=True)

    def _on_menu_quit(self):
        gui.Application.instance.quit()

    def _on_menu_filedlg(self):
        filedlg = gui.FileDialog(gui.FileDialog.OPEN_DIR, "Select dir",
                                 self.window.theme)
        filedlg.set_on_cancel(self._on_filedlg_cancel)
        filedlg.set_on_done(self._on_filedlg_done)
        self.window.show_dialog(filedlg)

    def _on_filedlg_cancel(self):
        self.window.close_dialog()

    def _on_filedlg_done(self, path):
        try:
            self.model = Model(path, ext=".txt")
            print(f"Load model successfully!\n{self.model}")
            self.window.close_dialog()
        except:
            self._show_error_dialog("Error", f"Failed to load model!")
        
        # change working directory to original
        os.chdir(self.working_dir)

    def _on_menu_show_camera(self):
        if self.model is None:
            self._show_error_dialog("Error", "Please load model first!")
            return
        
        if self.camera_params is None:
            cams_axis, cams_mesh, cams_line_set = self.model.add_cameras()
            self.camera_params = (cams_axis, cams_mesh, cams_line_set)

            mat_axis = rendering.MaterialRecord()
            mat_axis.shader = "defaultLit"
            mat_axis.base_color = [1, 1, 1, 1]
            for id, axis in enumerate(cams_axis):
                self.scene.scene.add_geometry(f"axis{id}", axis, mat_axis)
            
            mat_mesh = rendering.MaterialRecord()
            for id, mesh in enumerate(cams_mesh):
                texture = np.asarray(mesh.textures[0]).copy()
                texture = o3d.geometry.Image(texture)
                mat_mesh.albedo_img = texture 
                self.scene.scene.add_geometry(f"mesh{id}", mesh, mat_mesh)

            mat_line_set = rendering.MaterialRecord()
            mat_line_set.shader = "unlitLine"
            mat_line_set.line_width = 1 

            for id, line_set in enumerate(cams_line_set):
                self.scene.scene.add_geometry(f"line_set{id}", line_set, mat_line_set)

            self.camera_shown = True

            return 

        if not self.camera_shown:
            cams_axis, cams_mesh, cams_line_set = self.camera_params
            for id, axis in enumerate(cams_axis):
                self.scene.scene.show_geometry(f"axis{id}", True)
            
            for id, mesh in enumerate(cams_mesh):
                self.scene.scene.show_geometry(f"mesh{id}", True)

            for id, line_set in enumerate(cams_line_set):
                self.scene.scene.show_geometry(f"line_set{id}", True)

            self.camera_shown = True
        else:
            cams_axis, cams_mesh, cams_line_set = self.camera_params
            for id, axis in enumerate(cams_axis):
                self.scene.scene.show_geometry(f"axis{id}", False)

            for id, mesh in enumerate(cams_mesh):
                self.scene.scene.show_geometry(f"mesh{id}", False)

            for id, line_set in enumerate(cams_line_set):
                self.scene.scene.show_geometry(f"line_set{id}", False)

            self.camera_shown = False

    def _on_menu_show_lidar(self, idx=None, if_slider=False):
        if self.model is None:
            self._show_error_dialog("Error", "Please load model first!")
            return
        if not self.lidar_shown or if_slider:
            pcd = self.model.add_lidar(idx)
            mat = rendering.MaterialRecord()
            mat.shader = "defaultLit"
            self.scene.scene.remove_geometry("lidar")
            self.scene.scene.add_geometry("lidar", pcd, mat)

            self.lidar_shown = True
        else:
            self.scene.scene.show_geometry("lidar", False)
            self.lidar_shown = False
        
        # import random
        # random_name = random.randint(0, 1000)
        # self.scene.scene.add_geometry(f"lidar_{random_name}", pcd, mat)

    def _on_menu_show_settings(self):
        pass

    def _set_default_camera_view(self):
        try:
            W2C = self.model.get_transform(0)
            K = self.model.get_intrinsics(0)
        except:
            W2C = None
            K = None

        if W2C is not None and K is not None:
            # Extract rotation (R) and translation (t) from the W2C matrix
            R = W2C[:3, :3]
            t = W2C[:3, 3]

            # Create extrinsic matrix for Open3D
            extrinsic = np.eye(4)
            extrinsic[:3, :3] = R
            extrinsic[:3, 3] = t

            # Get intrinsic parameters from K
            fx = K[0, 0]
            fy = K[1, 1]
            cx = K[0, 2]
            cy = K[1, 2]

            # Create intrinsic matrix for Open3D
            intrinsic = o3d.camera.PinholeCameraIntrinsic()
            intrinsic.set_intrinsics(
                width=1280, height=720, fx=fx, fy=fy, cx=cx, cy=cy
            )

            # Get bounding box of the scene
            bounds = self.scene.scene.bounding_box

            # Setup camera with the extrinsic matrix (W2C should be the inverse of the camera pose)
            self.scene.setup_camera(intrinsic, np.linalg.inv(W2C), bounds)
        else:
            # Default behavior if no W2C matrix is provided
            bounds = self.scene.scene.bounding_box
            self.scene.setup_camera(60.0, bounds, bounds.get_center())
            self.scene.look_at(bounds.get_center(), bounds.get_center() + [0, 0, -1], [0, 1, 0])


    def _show_error_dialog(self, title, message):
        dialog = gui.Dialog(title)
        em = self.window.theme.font_size
        margins = gui.Margins(em, em, em, em)
        layout = gui.Vert(0, margins)
        
        label = gui.Label(message)
        layout.add_child(label)
        
        button_layout = gui.Horiz(0.25 * em)
        button_layout.add_stretch()

        ok_button = gui.Button("OK")
        ok_button.set_on_clicked(self.window.close_dialog)
        button_layout.add_child(ok_button)

        layout.add_child(button_layout)
        
        dialog.add_child(layout)
        self.window.show_dialog(dialog)


    def _show_warning_dialog(self, title, message):
        dialog = gui.Dialog(title)
        em = self.window.theme.font_size
        margins = gui.Margins(em, em, em, em)
        layout = gui.Vert(0, margins)
        
        label = gui.Label(message)
        layout.add_child(label)
        
        button_layout = gui.Horiz(0.25 * em)
        button_layout.add_stretch()

        ok_button = gui.Button("OK")
        ok_button.set_on_clicked(self.window.close_dialog)
        button_layout.add_child(ok_button)

        layout.add_child(button_layout)
        
        dialog.add_child(layout)
        self.window.show_dialog(dialog)


    def _run_gaussian_slam(self):
        from src.GS_SLAM.entities.gaussian_slam import GaussianSLAM
        print(os.getcwd())
        if self.model is None:
            self._show_error_dialog("Error", "Please load model first!")
            return

        config = {'project_name': 'Gaussian_SLAM_CARLA2NMR', 'dataset_name': 'CARLA2NMR',
                  'checkpoint_path': None, 'use_wandb': False, 'frame_limit': -1, 'seed': 0,
                  'mapping': {'new_submap_every': 50, 'map_every': 5, 'iterations': 4000,
                              'new_submap_iterations': 1000, 'new_submap_points_num': 600000,
                              'new_submap_gradient_points_num': 50000, 'new_frame_sample_size': -1,
                              'new_points_radius': 1e-07, 'current_view_opt_iterations': 0.4,
                              'alpha_thre': 0.05, 'pruning_thre': 0.1, 'submap_using_motion_heuristic': True},
                  'tracking': {'gt_camera': False, 'w_color_loss': 0.95, 'iterations': 60, 'cam_rot_lr': 0.0002,
                               'cam_trans_lr': 0.002, 'odometry_type': 'gt', 'help_camera_initialization': False,
                               'init_err_ratio': 5, 'odometer_method': 'hybrid', 'filter_alpha': False,
                               'filter_outlier_depth': True, 'alpha_thre': 0.98, 'soft_alpha': True, 'mask_invalid_depth': False},
                  'cam': {'H': 720, 'W': 1280, 'fx': 369.504, 'fy': 369.504, 'cx': 640, 'cy': 360, 'depth_scale': 25.6},
                  'inherit_from': 'configs/CARLA2NMR/CARLA2NMR.yaml',
                  'data': {'scene_name': 'carla_1_1', 'input_path': 'data/carla_12_20_rgb_1_1', 'output_path': 'output/CARLA2NMR/carla_12_20_rgb_1_1'}}

        def update_gaussian_model(gaussian_model):
            gui.Application.instance.post_to_main_thread(self.window, lambda: self.visualize_point_cloud(gaussian_model))

        def run_slam():
            print("Running Gaussian SLAM...")
            gslam = GaussianSLAM(config, dataset=self.model)
            gslam.run(update_callback=update_gaussian_model)

        threading.Thread(target=run_slam).start()

    def visualize_point_cloud(self, gaussian_model):
        point_cloud = gaussian_model.get_point_cloud()
        mat = rendering.MaterialRecord()
        mat.shader = "defaultLit"

        self.scene.scene.remove_geometry("3DGS")
        self.scene.scene.add_geometry("3DGS", point_cloud, mat)


    def _run_kiss_icp(self):
        print(os.getcwd())
        if self.model is None:
            self._show_error_dialog("Error", "Please load model first!")
            return

        from kiss_icp.pipeline import OdometryPipeline
        from kiss_icp.datasets import dataset_factory

        # get a dir path from a file
        dataset = dataset_factory(
            dataloader="generic",
            data_dir=os.path.join(self.model.lidars[0], ".."),
            sequence=None,
            topic=None,
            meta=None,
        )
        pipeline = OdometryPipeline(dataset, config="kiss_icp.yaml", visualize=False)

        transform = self.model.get_transform(0)

        def run_icp():
            for idx, result in enumerate(pipeline.run()):
                # draw the estimated camera pose
                est_pose = result["pose"]

                # Optional: Perform necessary point cloud transformation if needed
                # Reverse y
                T = np.array([[1, 0, 0, 0],
                            [0, -1, 0, 0],
                            [0, 0, 1, 0],
                            [0, 0, 0, 1]])
                
                est_pose = T @ est_pose

                # (x, y, z) -> (z, -x, -y) coordinate transformation
                T = np.array([[0, -1, 0, 0],
                            [0, 0, -1, 0],
                            [1, 0, 0, 0],
                            [0, 0, 0, 1]])

                est_pose = T @ est_pose

                # apply w2c transformation
                est_pose = transform @ est_pose

                # Rotation to quaternion
                R = est_pose[:3, :3]
                
                qvec = rotmat2qvec(R)
                tvec = est_pose[:3, 3]

                #TODO: add camera id to the estimated poses
                # Save the estimated poses
                self.model.estimated_poses.append((qvec, tvec, 1))

                # draw the estimated pose with coordinate
                mesh = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5)
                mesh.transform(est_pose)
                mat = rendering.MaterialRecord()
                mat.shader = "defaultLit"
                
                # Update the pose in the main thread
                gui.Application.instance.post_to_main_thread(self.window, lambda idx=idx, mesh=mesh, mat=mat: self.update_pose(idx, mesh, mat))
        
        threading.Thread(target=run_icp).start()

    #TODO: Implement gsplat training 
    def _run_gsplat_training(self):
        if self.model is None:
            self._show_error_dialog("Error", "Please load model first!")
            return
        
        def run_training():
            from utils.gsplat_utils.gsplat_training import Runner, Config
            import tyro
            import torch
            import time

            cfg = tyro.cli(Config)
            cfg.data_dir = self.model.path
            cfg.adjust_steps(cfg.steps_scaler)

            # get lidar point cloud
            lidar_pcd = self.model.add_lidar(0)

            #TODO: support training 3DGS with estimated poses
            if self.model.estimated_poses == []:
                self._show_warning_dialog("Warning", "Estimated camera poses couldn't be found, \nUsing groud truth poeses instead!")
                runner = Runner(cfg, lidar_pcd=lidar_pcd, poses=self.model.poses)
            else:
                runner = Runner(cfg, lidar_pcd=lidar_pcd, poses=self.model.estimated_poses)

            if cfg.ckpt is not None:
                # run eval only
                ckpt = torch.load(cfg.ckpt, map_location=runner.device)
                for k in runner.splats.keys():
                    runner.splats[k].data = ckpt["splats"[k]]
                runner.eval(step=ckpt["step"])
                runner.render_traj(step=ckpt["step"])
            else:
                runner.train()

            if not cfg.disable_viewer:
                print("Viewer running... Ctrl+C to exit.")
                time.sleep(1000000)
        
        # threading.Thread(target=run_training).start()

        #TEST: vis poses in viser
        import viser
        server = viser.ViserServer()


        # vis point cloud
        lidar_pcd = self.model.add_lidar(0)
        points = np.asarray(lidar_pcd.points)
        colors = np.asarray(lidar_pcd.colors)

        server.scene.add_point_cloud(name="Lidar",
            points=points,
            colors=colors,
            point_size=0.01,
            visible=True
        )


        # vis poses
        if self.model.estimated_poses == []:

            batched_wxyzs = []
            batched_positions = []
        
            for pose in self.model.poses:
                R = qvec2rotmat(pose[0])
                t = pose[1]
                
                # invert
                t = -R.T @ t
                R = R.T

                wxyz = rotmat2qvec(R)
                batched_positions.append(t)
                batched_wxyzs.append(wxyz)

            batched_wxyzs = np.array(batched_wxyzs)
            batched_positions = np.array(batched_positions)

        else:
            batched_wxyzs = np.array([pose[0] for pose in self.model.estimated_poses])
            batched_positions = np.array([pose[1] for pose in self.model.estimated_poses])

        server.scene.add_batched_axes(name="BatchedAxes",
            batched_wxyzs=batched_wxyzs,
            batched_positions=batched_positions,
            axes_length=0.5,
            axes_radius=0.025,
            wxyz=(1.0, 0.0, 0.0, 0.0),  # Optional, default value
            position=(0.0, 0.0, 0.0),  # Optional, default value
            visible=True  # Optional, default value
        )


    def update_pose(self, idx, mesh, mat):
        self.scene.scene.remove_geometry(f"est_pose_{idx}")
        self.scene.scene.add_geometry(f"est_pose_{idx}", mesh, mat)




        
        

